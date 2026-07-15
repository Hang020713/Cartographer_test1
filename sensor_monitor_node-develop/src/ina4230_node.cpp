#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

namespace
{
    std::string to_hex_string(uint8_t val)
    {
        const char hex[] = "0123456789abcdef";
        return {hex[val >> 4], hex[val & 0x0F]};
    }
} // namespace

class INA4230Node : public rclcpp::Node
{
public:
    INA4230Node() : Node("ina4230_node"), i2c_fd_(-1)
    {
        load_parameters();
        open_i2c_bus();
        probe_devices();
        initialize_devices();
        create_publishers();

        timer_ = this->create_wall_timer(kTimerPeriod, std::bind(&INA4230Node::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "INA4230 node startup successful; reading %zu sensor devices...", i2c_addrs_.size());
    }

    ~INA4230Node()
    {
        if (i2c_fd_ >= 0)
        {
            close(i2c_fd_);
        }
    }

private:
    // ── Constants ──────────────────────────────────────────────────────
    static constexpr auto kTimerPeriod = 2000ms;

    static constexpr uint8_t kRegConfig = 0x20;
    static constexpr uint8_t kRegCurrent[4] = {0x02, 0x0A, 0x12, 0x1A};
    static constexpr uint8_t kRegCal[4] = {0x05, 0x0D, 0x15, 0x1D};

    // CONFIG1: AVG=128, VBUSCT/VSHCT=1.1ms, MODE=continuous shunt+bus
    // Reserved bits default to 0xF___
    static constexpr uint16_t kConfigInit = 0xF427;

    // ── Initialization steps ───────────────────────────────────────────
    static bool parse_i2c_address(const std::string &token, uint8_t &addr)
    {
        try
        {
            size_t idx = 0;
            int value = std::stoi(token, &idx, 0);
            if (idx != token.size() || value < 0 || value > 0x7F)
            {
                return false;
            }
            addr = static_cast<uint8_t>(value);
            return true;
        }
        catch (const std::exception &)
        {
            return false;
        }
    }

    void load_parameters()
    {
        this->declare_parameter<std::string>("i2c_device", "/dev/i2c-3");
        this->declare_parameter<std::vector<int64_t>>("i2c_addresses", {0x40, 0x41, 0x44});
        this->declare_parameter<double>("current_lsb", 0.001);
        this->declare_parameter<double>("shunt_resistance", 0.01);

        i2c_device_ = this->get_parameter("i2c_device").as_string();
        current_lsb_ = this->get_parameter("current_lsb").as_double();
        r_shunt_ = this->get_parameter("shunt_resistance").as_double();

        i2c_addrs_.clear();
        rclcpp::Parameter i2c_addresses_param;
        if (!this->get_parameter("i2c_addresses", i2c_addresses_param))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load 'i2c_addresses' parameter");
            throw std::runtime_error("Missing i2c_addresses parameter");
        }

        if (i2c_addresses_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            auto int_addrs = i2c_addresses_param.as_integer_array();
            for (auto addr : int_addrs)
            {
                if (addr < 0 || addr > 0x7F)
                {
                    RCLCPP_WARN(this->get_logger(), "Skipping out-of-range I2C address %d", addr);
                    continue;
                }
                i2c_addrs_.push_back(static_cast<uint8_t>(addr));
            }
        }
        else if (i2c_addresses_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            for (auto &addr_str : i2c_addresses_param.as_string_array())
            {
                uint8_t addr = 0;
                if (!parse_i2c_address(addr_str, addr))
                {
                    RCLCPP_WARN(this->get_logger(), "Skipping invalid I2C address string '%s'", addr_str.c_str());
                    continue;
                }
                i2c_addrs_.push_back(addr);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unsupported type for 'i2c_addresses'; expected integer array or string array");
        }

        if (i2c_addrs_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'i2c_addresses' is empty or invalid");
            throw std::runtime_error("Invalid i2c_addresses parameter");
        }

        std::ostringstream addr_list;
        addr_list << "Configured I2C addresses:";
        for (auto addr : i2c_addrs_)
        {
            addr_list << " 0x" << to_hex_string(addr);
        }
        RCLCPP_INFO(this->get_logger(), "%s", addr_list.str().c_str());
    }

    void open_i2c_bus()
    {
        i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
        if (i2c_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open the I2C bus device: %s", i2c_device_.c_str());
            throw std::runtime_error("Failed to open I2C bus device: " + i2c_device_);
        }
    }

    void probe_devices()
    {
        // Probe via real register read (CONFIG) instead of zero-length write —
        // some I2C bus drivers don't support zero-length messages.
        std::vector<uint8_t> detected;
        for (auto addr : i2c_addrs_)
        {
            if (ioctl(i2c_fd_, I2C_SLAVE, addr) < 0)
            {
                RCLCPP_WARN(this->get_logger(), "ioctl(I2C_SLAVE, 0x%02X) failed: %s (errno=%d)", addr, strerror(errno), errno);
                continue;
            }
            uint8_t reg = kRegConfig;
            uint8_t buf[2] = {0};
            if (write(i2c_fd_, &reg, 1) != 1)
            {
                RCLCPP_WARN(this->get_logger(), "No device found at I2C address 0x%02X — skipping", addr);
                continue;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            if (read(i2c_fd_, buf, 2) != 2)
            {
                RCLCPP_WARN(this->get_logger(), "No device found at I2C address 0x%02X — skipping", addr);
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "Detected INA4230 at I2C address 0x%02X", addr);
            detected.push_back(addr);
        }

        if (detected.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No INA4230 devices found on bus %s — check wiring and power", i2c_device_.c_str());
            throw std::runtime_error("No INA4230 devices found on " + i2c_device_);
        }
        i2c_addrs_ = std::move(detected);
    }

    void initialize_devices()
    {
        for (auto addr : i2c_addrs_)
        {
            if (!init_device(addr))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize INA4230 at address 0x%02X", addr);
                throw std::runtime_error("Failed to init INA4230 at 0x" + to_hex_string(addr));
            }
        }
    }

    void create_publishers()
    {
        current_pubs_.resize(i2c_addrs_.size());
        raw_pubs_.resize(i2c_addrs_.size());
        for (size_t d = 0; d < i2c_addrs_.size(); ++d)
        {
            uint8_t addr = i2c_addrs_[d];
            current_pubs_[d].resize(4);
            raw_pubs_[d].resize(4);
            for (int ch = 0; ch < 4; ++ch)
            {
                std::string topic = "ina4230_0x" + to_hex_string(addr) + "/channel_" + std::to_string(ch + 1);
                current_pubs_[d][ch] = this->create_publisher<std_msgs::msg::Float64>(topic, 10);
                raw_pubs_[d][ch] = this->create_publisher<std_msgs::msg::Float64>(topic + "/raw", 10);
            }
        }
    }

    // ── I2C register access ────────────────────────────────────────────
    bool write_reg16(uint8_t dev_addr, uint8_t reg_addr, uint16_t value)
    {
        if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "write_reg16 ioctl failed for device 0x%02X: %s (errno=%d)", dev_addr, strerror(errno), errno);
            return false;
        }
        uint8_t buf[3] = {reg_addr,
                          static_cast<uint8_t>(value >> 8),
                          static_cast<uint8_t>(value & 0xFF)};
        if (write(i2c_fd_, buf, 3) != 3)
        {
            RCLCPP_ERROR(this->get_logger(), "write_reg16 write failed for device 0x%02X reg 0x%02X: %s (errno=%d)", dev_addr, reg_addr, strerror(errno), errno);
            return false;
        }
        return true;
    }

    bool read_reg16(uint8_t dev_addr, uint8_t reg_addr, int16_t &value)
    {
        if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr) < 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "ioctl(I2C_SLAVE) failed for device 0x%02X, reg 0x%02X: %s (errno=%d)", dev_addr, reg_addr, strerror(errno), errno);
            return false;
        }
        if (write(i2c_fd_, &reg_addr, 1) != 1)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "I2C write reg pointer failed for device 0x%02X, reg 0x%02X: %s (errno=%d)", dev_addr, reg_addr, strerror(errno), errno);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        uint8_t buf[2] = {0};
        if (read(i2c_fd_, buf, 2) != 2)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "I2C read data failed for device 0x%02X, reg 0x%02X: %s (errno=%d)", dev_addr, reg_addr, strerror(errno), errno);
            return false;
        }
        value = static_cast<int16_t>((buf[0] << 8) | buf[1]);
        return true;
    }

    // ── Per-device configuration ───────────────────────────────────────
    bool init_device(uint8_t addr)
    {
        if (!write_reg16(addr, kRegConfig, kConfigInit))
        {
            return false;
        }

        // Cal = 0.00512 / (current_lsb × r_shunt)
        double cal = 0.00512 / (current_lsb_ * r_shunt_);
        auto cal_value = static_cast<uint16_t>(cal + 0.5); // round to nearest
        RCLCPP_INFO(this->get_logger(), "INA4230 0x%02X: R_shunt=%.3fΩ, LSB=%.4fA, Cal=%d (0x%04X)", addr, r_shunt_, current_lsb_, cal_value, cal_value);

        for (int ch = 0; ch < 4; ++ch)
        {
            if (!write_reg16(addr, kRegCal[ch], cal_value))
            {
                return false;
            }
        }
        return true;
    }

    // ── Periodic read ──────────────────────────────────────────────────
    void timer_callback()
    {
        for (size_t d = 0; d < i2c_addrs_.size(); ++d)
        {
            uint8_t addr = i2c_addrs_[d];
            std::ostringstream log_oss;
            log_oss << "INA4230 0x" << to_hex_string(addr);
            int ok_count = 0;

            for (int ch = 0; ch < 4; ++ch)
            {
                int16_t raw = 0;
                if (read_reg16(addr, kRegCurrent[ch], raw))
                {
                    double current_a = static_cast<double>(raw) * current_lsb_;
                    log_oss << " | ch" << (ch + 1) << ": " << raw << " " << current_a << "A";
                    ++ok_count;

                    auto current_msg = std::make_unique<std_msgs::msg::Float64>();
                    current_msg->data = current_a;
                    current_pubs_[d][ch]->publish(std::move(current_msg));

                    auto raw_msg = std::make_unique<std_msgs::msg::Float64>();
                    raw_msg->data = static_cast<double>(raw);
                    raw_pubs_[d][ch]->publish(std::move(raw_msg));
                }
            }

            if (ok_count > 0)
            {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", log_oss.str().c_str());
            }
        }
    }

    // ── Member variables ───────────────────────────────────────────────
    // I2C
    int i2c_fd_{-1};
    std::string i2c_device_;
    std::vector<uint8_t> i2c_addrs_;

    // Calibration
    double current_lsb_{0.0};
    double r_shunt_{0.0};

    // ROS
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>> current_pubs_;
    std::vector<std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>> raw_pubs_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<INA4230Node>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ina4230_main"), "Node creation failed: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

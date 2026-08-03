#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

namespace
{
    /// CRC-8/MAXIM for SHT3x: polynomial x^8 + x^5 + x^4 + 1
    uint8_t sht3x_crc8(const uint8_t *data, uint8_t nbytes)
    {
        uint8_t crc = 0xFF;
        for (uint8_t i = 0; i < nbytes; ++i)
        {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit)
            {
                if (crc & 0x80)
                {
                    crc = (crc << 1) ^ 0x31;
                }
                else
                {
                    crc = (crc << 1);
                }
            }
        }
        return crc;
    }
} // namespace

class SHT3xNode : public rclcpp::Node
{
public:
    SHT3xNode() : Node("sht3x_node"), i2c_fd_(-1)
    {
        // ── Parameters ──
        this->declare_parameter<std::string>("i2c_device", "/dev/i2c-3");
        this->declare_parameter<int>("i2c_address", 0x45);

        std::string i2c_device = this->get_parameter("i2c_device").as_string();
        i2c_addr_ = this->get_parameter("i2c_address").as_int();

        // ── I2C init & probe ──
        i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
        if (i2c_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s: %s", i2c_device.c_str(), strerror(errno));
            throw std::runtime_error("Failed to open I2C device: " + i2c_device);
        }
        if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr_) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "ioctl I2C_SLAVE failed: %s", strerror(errno));
            throw std::runtime_error("Failed to set I2C slave address");
        }
        // Probe via zero-length write — unlike INA4230, SHT3x has no
        // always-readable register, so this is the standard detection method.
        {
            uint8_t dummy;
            if (write(i2c_fd_, &dummy, 0) != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "No device at I2C address 0x%02X — check wiring and address", i2c_addr_);
                throw std::runtime_error("No SHT3x device found on " + i2c_device);
            }
        }

        // ── Publishers ──
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("~/temperature", 10);
        temp_raw_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/temperature/raw", 10);
        hum_pub_ = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("~/humidity", 10);
        hum_raw_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/humidity/raw", 10);

        // ── Timer ──
        timer_ = this->create_wall_timer(kTimerPeriod, std::bind(&SHT3xNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "SHT3x Node has started. Reading from 0x%02X", i2c_addr_);
    }

    ~SHT3xNode()
    {
        if (i2c_fd_ >= 0)
        {
            close(i2c_fd_);
        }
    }

private:
    // ── Constants ──────────────────────────────────────────────────────
    static constexpr auto kTimerPeriod = 2000ms;

    // Measurement command: high repeatability, clock stretching enabled
    static constexpr uint8_t kMeasCmd[2] = {0x24, 0x00};

    // Conversion formulas (datasheet §4.13)
    static constexpr double kTempOffset = -45.0;
    static constexpr double kTempScale = 175.0;
    static constexpr double kHumScale = 100.0;
    static constexpr double kRawMax = 65535.0;

    // ── Timer callback ─────────────────────────────────────────────────
    void timer_callback()
    {
        if (write(i2c_fd_, kMeasCmd, 2) != 2)
        {
            RCLCPP_WARN(this->get_logger(), "I2C write command failed: %s (errno=%d)", strerror(errno), errno);
            return;
        }

        std::this_thread::sleep_for(kMeasWait);

        uint8_t buf[6] = {0};
        if (read(i2c_fd_, buf, 6) != 6)
        {
            RCLCPP_WARN(this->get_logger(), "I2C read data failed.");
            return;
        }

        bool temp_crc_ok = (sht3x_crc8(&buf[0], 2) == buf[2]);
        bool hum_crc_ok = (sht3x_crc8(&buf[3], 2) == buf[5]);

        if (!temp_crc_ok || !hum_crc_ok)
        {
            RCLCPP_ERROR(this->get_logger(), "CRC Checksum Failed! (Temp CRC: %s | Hum CRC: %s)", temp_crc_ok ? "OK" : "BAD", hum_crc_ok ? "OK" : "BAD");
            return;
        }

        uint16_t raw_temp = (buf[0] << 8) | buf[1];
        uint16_t raw_hum = (buf[3] << 8) | buf[4];

        double temperature = kTempOffset + kTempScale * (static_cast<double>(raw_temp) / kRawMax);
        double humidity = kHumScale * (static_cast<double>(raw_hum) / kRawMax);

        auto now = this->now();

        auto temp_msg = sensor_msgs::msg::Temperature();
        temp_msg.header.stamp = now;
        temp_msg.header.frame_id = "sht3x_link";
        temp_msg.temperature = temperature;
        temp_msg.variance = 0.04;

        auto hum_msg = sensor_msgs::msg::RelativeHumidity();
        hum_msg.header.stamp = now;
        hum_msg.header.frame_id = "sht3x_link";
        hum_msg.relative_humidity = humidity / 100.0;
        hum_msg.variance = 0.0004;

        auto temp_raw_msg = std::make_unique<std_msgs::msg::Float64>();
        temp_raw_msg->data = static_cast<double>(raw_temp);
        temp_raw_pub_->publish(std::move(temp_raw_msg));

        auto hum_raw_msg = std::make_unique<std_msgs::msg::Float64>();
        hum_raw_msg->data = static_cast<double>(raw_hum);
        hum_raw_pub_->publish(std::move(hum_raw_msg));

        temp_pub_->publish(temp_msg);
        hum_pub_->publish(hum_msg);

        RCLCPP_INFO(this->get_logger(), "Published -> Temp: %.2f °C, Hum: %.2f %% (raw: %u, %u)", temperature, humidity, raw_temp, raw_hum);
    }

    // ── Member variables ───────────────────────────────────────────────
    // I2C
    int i2c_fd_{-1};
    int i2c_addr_{0};

    // ROS
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr hum_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hum_raw_pub_;

    // Timing
    static constexpr auto kMeasWait = 20ms;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<SHT3xNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("sht3x_main"), "Node creation failed: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

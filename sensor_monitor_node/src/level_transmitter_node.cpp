#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/serial.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

namespace
{
    // ── Hex conversion helper ───────────────────────────────────────────
    std::string to_hex_string(uint8_t val)
    {
        const char hex[] = "0123456789abcdef";
        return {hex[val >> 4], hex[val & 0x0F]};
    }

    // ── Modbus CRC16 (polynomial 0xA001) ─────────────────────────────────
    uint16_t modbus_crc16(const uint8_t *data, size_t len)
    {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; ++i)
        {
            crc ^= data[i];
            for (int j = 0; j < 8; ++j)
            {
                if (crc & 0x0001)
                {
                    crc = (crc >> 1) ^ 0xA001;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    // ── Unit code → human-readable string ────────────────────────────────
    const char *unit_code_to_string(int code)
    {
        switch (code)
        {
        case 0:
            return "MPa";
        case 1:
            return "KPa";
        case 2:
            return "Pa";
        case 3:
            return "bar";
        case 4:
            return "mbar";
        case 5:
            return "Kgcm2";
        case 6:
            return "PSI";
        case 7:
            return "mH2O";
        case 8:
            return "mmH2O";
        case 9:
            return "inH2O";
        case 10:
            return "H2O";
        case 11:
            return "mHg";
        case 12:
            return "mmHg";
        case 13:
            return "inHg";
        case 14:
            return "atm";
        case 15:
            return "Torr";
        case 16:
            return "m";
        case 17:
            return "cm";
        case 18:
            return "mm";
        case 19:
            return "Kg";
        case 20:
            return "°C";
        case 21:
            return "PH";
        case 22:
            return "°F";
        case 23:
            return "(none)";
        default:
            return "?";
        }
    }

    // ── Baud rate → termios constant ─────────────────────────────────────
    speed_t baud_to_speed(int baud)
    {
        switch (baud)
        {
        case 1200:
            return B1200;
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        default:
            return B9600;
        }
    }

    // ── Convert 4 big-endian bytes to native float (IEEE 754) ────────────
    float be_bytes_to_float(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
    {
        // ABCD big-endian: A is MSB, D is LSB.
        // Assemble into a uint32, then memcpy to float — the compiler
        // handles native byte order automatically; no explicit bswap needed.
        uint32_t be = (static_cast<uint32_t>(a) << 24) |
                      (static_cast<uint32_t>(b) << 16) |
                      (static_cast<uint32_t>(c) << 8) |
                      static_cast<uint32_t>(d);
        float value;
        std::memcpy(&value, &be, sizeof(value));
        return value;
    }
} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// LevelTransmitterNode — reads a submersible level/pressure transmitter
// via Modbus RTU over RS-485 and publishes the measured value.
// ═══════════════════════════════════════════════════════════════════════════
class LevelTransmitterNode : public rclcpp::Node
{
public:
    LevelTransmitterNode()
        : Node("level_transmitter_node"), serial_fd_(-1)
    {
        load_parameters();
        open_serial_port();
        create_publishers();
        read_sensor_metadata();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(read_interval_ms_),
            std::bind(&LevelTransmitterNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(),
                    "Level Transmitter node started.  port=%s  baud=%d  slave_id=%d  unit=%s  mode=%s",
                    serial_port_.c_str(), baud_rate_, slave_id_, unit_str_.c_str(),
                    use_float_mode_ ? "float" : "integer");
    }

    ~LevelTransmitterNode() override
    {
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
        }
    }

private:
    // ══════════════════════════════════════════════════════════════════════
    // Constants
    // ══════════════════════════════════════════════════════════════════════
    static constexpr uint8_t kFuncReadHolding = 0x03;
    static constexpr uint16_t kRegUnit = 0x0002;         // measurement unit
    static constexpr uint16_t kRegDecimalPoint = 0x0003; // decimal point position (0–4)
    static constexpr uint16_t kRegPvInteger = 0x0004;    // measured value (signed int16)
    static constexpr uint16_t kRegPvFloat = 0x0016;      // measured value (IEEE 754 float, 2 regs)

    static constexpr int kModbusReadTimeoutMs = 50;    // ≥3.5 char times at 9600 baud
    static constexpr int kModbusMaxAttempts = 20;      // 20 × 100 ms = 2 s total
    static constexpr double kNearZeroThreshold = 1e-9; // clamp smaller values to zero
    static constexpr double kMaxReasonableValue = 1e6; // reject implausibly large readings

    // ══════════════════════════════════════════════════════════════════════
    // Initialization (called once in constructor)
    // ══════════════════════════════════════════════════════════════════════

    void load_parameters()
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA3");
        this->declare_parameter<int>("baud_rate", 9600);
        this->declare_parameter<int>("slave_id", 1);
        this->declare_parameter<int>("read_interval_ms", 2000);
        this->declare_parameter<bool>("use_float_mode", true);
        this->declare_parameter<std::string>("topic_name", "~/level");

        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        slave_id_ = this->get_parameter("slave_id").as_int();
        read_interval_ms_ = this->get_parameter("read_interval_ms").as_int();
        use_float_mode_ = this->get_parameter("use_float_mode").as_bool();
        topic_name_ = this->get_parameter("topic_name").as_string();

        if (slave_id_ < 1 || slave_id_ > 255)
        {
            RCLCPP_WARN(this->get_logger(),
                        "slave_id %d out of range [1,255], clamping to 1", slave_id_);
            slave_id_ = 1;
        }
    }

    void open_serial_port()
    {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot open %s: %s (errno=%d)",
                         serial_port_.c_str(), strerror(errno), errno);
            throw std::runtime_error("Failed to open serial port: " + serial_port_);
        }

        struct termios tty;
        std::memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s (errno=%d)",
                         strerror(errno), errno);
            throw std::runtime_error("tcgetattr failed on " + serial_port_);
        }

        configure_baud_rate(tty);

        // 8 data bits, no parity, 1 stop bit (8N1)
        tty.c_cflag = CS8 | CREAD | CLOCAL;
        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        // Read timeout: VMIN=0, VTIME=10 → 1.0 s inter-byte timeout
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s (errno=%d)",
                         strerror(errno), errno);
            throw std::runtime_error("tcsetattr failed on " + serial_port_);
        }

        configure_rs485_mode();

        tcflush(serial_fd_, TCIOFLUSH);
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened (%d baud, 8N1)",
                    serial_port_.c_str(), baud_rate_);
    }

    /// Set baud rate on the given termios structure, with validation warning.
    void configure_baud_rate(struct termios &tty)
    {
        speed_t speed = baud_to_speed(baud_rate_);
        if (baud_rate_ != 1200 && baud_rate_ != 2400 && baud_rate_ != 4800 &&
            baud_rate_ != 9600 && baud_rate_ != 19200 && baud_rate_ != 38400)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Unsupported baud rate %d, falling back to 9600", baud_rate_);
            speed = B9600;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);
    }

    /// Enable RS-485 half-duplex mode: RTS controls transceiver direction.
    /// RTS high → driver enabled (TX), RTS low → receiver enabled (RX).
    void configure_rs485_mode()
    {
        struct serial_rs485 rs485_conf;
        std::memset(&rs485_conf, 0, sizeof(rs485_conf));
        rs485_conf.flags = SER_RS485_ENABLED;
        rs485_conf.flags |= SER_RS485_RTS_ON_SEND; // RTS=1 only while transmitting
        // deliberately omit SER_RS485_RTS_AFTER_SEND     RTS=0 after TX → RX mode
        rs485_conf.delay_rts_before_send = 0;
        rs485_conf.delay_rts_after_send = 0;

        if (ioctl(serial_fd_, TIOCSRS485, &rs485_conf) < 0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "TIOCSRS485 ioctl failed: %s (errno=%d) — RS-485 direction control may not work",
                        strerror(errno), errno);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "RS-485 mode enabled (RTS direction control)");
        }
    }

    void create_publishers()
    {
        level_pub_ = this->create_publisher<std_msgs::msg::Float64>(topic_name_, 10);
    }

    void read_sensor_metadata()
    {
        std::vector<uint16_t> regs;
        if (read_holding_registers(slave_id_, kRegUnit, 1, regs))
        {
            unit_code_ = static_cast<int>(regs[0]);
            unit_str_ = unit_code_to_string(unit_code_);
            RCLCPP_INFO(this->get_logger(), "Sensor unit code: %d (%s)",
                        unit_code_, unit_str_.c_str());
        }
        else
        {
            unit_code_ = -1;
            unit_str_ = "unknown";
            RCLCPP_WARN(this->get_logger(),
                        "Could not read unit register — will publish raw values");
        }
    }

    // ══════════════════════════════════════════════════════════════════════
    // Modbus RTU communication
    // ══════════════════════════════════════════════════════════════════════

    /// Build and send a Modbus RTU "Read Holding Registers" request,
    /// then read and validate the response.
    /// @return true on success, false on any error (logged with throttling).
    bool read_holding_registers(int slave_id, uint16_t start_addr,
                                uint16_t num_regs, std::vector<uint16_t> &values_out)
    {
        // 1. Build & send request
        std::vector<uint8_t> req = build_request_frame(slave_id, start_addr, num_regs);
        if (!write_request_frame(req))
        {
            return false;
        }

        // 2. Wait for response (Modbus RTU inter-frame gap)
        std::this_thread::sleep_for(std::chrono::milliseconds(kModbusReadTimeoutMs));

        // 3. Read response
        uint8_t buf[256];
        int total = 0;
        if (!read_response_frame(start_addr, buf, sizeof(buf), total))
        {
            return false;
        }

        // 4. Validate
        if (!validate_response_frame(buf, total, slave_id, num_regs))
        {
            return false;
        }

        // 5. Extract register values
        extract_register_values(buf, buf[2], values_out);
        return true;
    }

    /// Build Modbus RTU request frame: addr + func + start(2) + count(2) + CRC(2).
    std::vector<uint8_t> build_request_frame(int slave_id, uint16_t start_addr,
                                             uint16_t num_regs)
    {
        std::vector<uint8_t> req;
        req.reserve(8);
        req.push_back(static_cast<uint8_t>(slave_id));
        req.push_back(kFuncReadHolding);
        req.push_back(static_cast<uint8_t>(start_addr >> 8));
        req.push_back(static_cast<uint8_t>(start_addr & 0xFF));
        req.push_back(static_cast<uint8_t>(num_regs >> 8));
        req.push_back(static_cast<uint8_t>(num_regs & 0xFF));

        uint16_t crc = modbus_crc16(req.data(), req.size());
        req.push_back(static_cast<uint8_t>(crc & 0xFF)); // CRC low byte (little-endian on wire)
        req.push_back(static_cast<uint8_t>(crc >> 8));   // CRC high byte

        return req;
    }

    /// Flush RX buffer and send the request frame over the serial port.
    bool write_request_frame(const std::vector<uint8_t> &req)
    {
        tcflush(serial_fd_, TCIFLUSH);

        int written = write(serial_fd_, req.data(), req.size());
        if (written != static_cast<int>(req.size()))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Serial write failed: %d/%zu bytes written (errno=%d)",
                                 written, req.size(), errno);
            return false;
        }

        log_hex("TX", req.data(), req.size());
        return true;
    }

    /// Read response bytes with retry loop.  Sets total on success.
    bool read_response_frame(uint16_t start_addr, uint8_t *buf, int buf_size, int &total)
    {
        total = 0;
        for (int attempt = 0; attempt < kModbusMaxAttempts; ++attempt)
        {
            int n = read(serial_fd_, buf + total, buf_size - total - 1);
            if (n > 0)
            {
                total += n;
                // Minimum valid frame: addr(1) + func(1) + byte_count(1) + CRC(2) = 5
                if (total >= 5)
                {
                    uint8_t expected = static_cast<uint8_t>(3 + buf[2] + 2);
                    if (total >= expected)
                    {
                        log_hex("RX", buf, total);
                        return true; // frame complete
                    }
                }
            }
            else if (n < 0 && errno != EAGAIN)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Serial read error: %s (errno=%d)",
                                     strerror(errno), errno);
                return false;
            }
            // n == 0 (timeout) or EAGAIN → keep waiting
        }

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Modbus timeout: no response from slave %d (reg 0x%04X)",
                             slave_id_, start_addr);
        return false;
    }

    /// Validate the Modbus response: address, function, byte count, CRC.
    bool validate_response_frame(const uint8_t *resp, int total,
                                 int slave_id, uint16_t num_regs)
    {
        // Address check
        if (resp[0] != static_cast<uint8_t>(slave_id))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Response slave address mismatch: expected %d, got %d",
                                 slave_id, resp[0]);
            return false;
        }

        // Modbus exception?
        if (resp[1] & 0x80)
        {
            uint8_t ex_code = (total >= 3) ? resp[2] : 0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Modbus exception from slave %d: func=0x%02X, code=%d",
                                 slave_id, resp[1] & 0x7F, ex_code);
            return false;
        }

        // Function code
        if (resp[1] != kFuncReadHolding)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Unexpected function code 0x%02X (expected 0x%02X)",
                                 resp[1], kFuncReadHolding);
            return false;
        }

        uint8_t byte_count = resp[2];

        // Byte count consistency
        if (static_cast<int>(byte_count) != static_cast<int>(num_regs) * 2)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Byte count mismatch: expected %d, got %d",
                                 num_regs * 2, byte_count);
            return false;
        }

        // Frame length
        if (total < 3 + byte_count + 2)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Response truncated: got %d bytes, need %d",
                                 total, 3 + byte_count + 2);
            return false;
        }

        // CRC check
        uint16_t crc_received = static_cast<uint16_t>(resp[3 + byte_count]) |
                                (static_cast<uint16_t>(resp[4 + byte_count]) << 8);
        uint16_t crc_calc = modbus_crc16(resp, 3 + byte_count);
        if (crc_received != crc_calc)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "CRC error: received 0x%04X, calculated 0x%04X",
                                 crc_received, crc_calc);
            return false;
        }

        return true;
    }

    /// Extract big-endian register values from a validated response payload.
    void extract_register_values(const uint8_t *resp, int byte_count,
                                 std::vector<uint16_t> &values_out)
    {
        values_out.clear();
        values_out.reserve(byte_count / 2);
        for (int i = 0; i < byte_count; i += 2)
        {
            uint16_t val = (static_cast<uint16_t>(resp[3 + i]) << 8) |
                           static_cast<uint16_t>(resp[4 + i]);
            values_out.push_back(val);
        }
    }

    /// Debug helper: dump raw bytes as hex via RCLCPP_DEBUG.
    void log_hex(const char *prefix, const uint8_t *data, size_t len)
    {
        std::ostringstream oss;
        oss << prefix << " [" << len << "]:";
        for (size_t i = 0; i < len; ++i)
            oss << " " << to_hex_string(data[i]);
        RCLCPP_DEBUG(this->get_logger(), "%s", oss.str().c_str());
    }

    // ══════════════════════════════════════════════════════════════════════
    // Measurement
    // ══════════════════════════════════════════════════════════════════════

    void timer_callback()
    {
        double value = 0.0;
        bool ok = use_float_mode_ ? read_float_value(value)
                                  : read_integer_value(value);

        if (!ok || !sanitize_measurement(value))
        {
            return; // error already logged in sub-method
        }

        auto msg = std::make_unique<std_msgs::msg::Float64>();
        msg->data = value;
        level_pub_->publish(std::move(msg));

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Published: %.2f %s", value, unit_str_.c_str());
    }

    /// Read via float registers (0x0016, 2 regs → 4-byte IEEE 754 big-endian).
    bool read_float_value(double &out)
    {
        std::vector<uint16_t> regs;
        if (!read_holding_registers(slave_id_, kRegPvFloat, 2, regs) || regs.size() < 2)
        {
            return false;
        }

        // regs[0] = (A<<8)|B,  regs[1] = (C<<8)|D  (ABCD big-endian)
        uint8_t a = static_cast<uint8_t>(regs[0] >> 8);
        uint8_t b = static_cast<uint8_t>(regs[0] & 0xFF);
        uint8_t c = static_cast<uint8_t>(regs[1] >> 8);
        uint8_t d = static_cast<uint8_t>(regs[1] & 0xFF);

        float f = be_bytes_to_float(a, b, c, d);

        // Guard against NaN / Inf
        if (!std::isfinite(f))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Sensor returned non-finite float: 0x%02X%02X%02X%02X",
                                 a, b, c, d);
            return false;
        }

        out = static_cast<double>(f);
        return true;
    }

    /// Fallback: read integer PV (0x0004) + decimal point (0x0003), then
    /// compute value = raw / 10^decimals.
    bool read_integer_value(double &out)
    {
        // Read decimal point position (0x0003)
        int dp = 0;
        {
            std::vector<uint16_t> regs;
            if (read_holding_registers(slave_id_, kRegDecimalPoint, 1, regs) && !regs.empty())
            {
                dp = static_cast<int>(regs[0]);
                if (dp < 0)
                    dp = 0;
                if (dp > 4)
                    dp = 4;
            }
        }

        // Read raw PV (0x0004)
        std::vector<uint16_t> regs;
        if (!read_holding_registers(slave_id_, kRegPvInteger, 1, regs) || regs.empty())
        {
            return false;
        }

        // Signed 16-bit interpretation
        int16_t raw = static_cast<int16_t>(regs[0]);
        double scale = std::pow(10.0, dp);
        out = static_cast<double>(raw) / scale;
        return true;
    }

    /// Sanitize a raw sensor reading:
    /// - Clamp near-zero noise to true zero
    /// - Reject implausibly large values
    /// - Round to 2 decimal places
    /// @return true if the value is valid and can be published
    bool sanitize_measurement(double &value)
    {
        // Clamp near-zero floating-point noise to true zero.
        // -1.15e-19 and similar values are sensor idle noise, not real negatives.
        if (std::fabs(value) < kNearZeroThreshold)
        {
            value = 0.0;
        }

        // Reject impossibly large values (sensor max range is typically ≤ 200 mH2O / 2 MPa).
        // 9.6e+16 and similar are garbage from communication errors.
        if (std::fabs(value) > kMaxReasonableValue)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Sensor value out of range: %.6e — discarding", value);
            return false;
        }

        // Round to 2 decimal places
        value = std::round(value * 100.0) / 100.0;
        return true;
    }

    // ══════════════════════════════════════════════════════════════════════
    // Member variables
    // ══════════════════════════════════════════════════════════════════════

    // Serial port
    int serial_fd_{-1};
    std::string serial_port_;
    int baud_rate_{9600};

    // Modbus
    int slave_id_{1};

    // Sensor metadata
    int unit_code_{-1};
    std::string unit_str_{"unknown"};

    // Measurement mode
    bool use_float_mode_{true};

    // ROS
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr level_pub_;
    std::string topic_name_;
    int read_interval_ms_{2000};
};

// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<LevelTransmitterNode>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("level_transmitter_main"),
                     "Node creation failed: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

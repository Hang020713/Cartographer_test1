/* bms485_node.cpp - ROS 2 wrapper around the RS485 BMS poller.
 *
 * Publishes:
 *   ~/bms_status   (bms485_ros2/msg/BmsStatus)
 *   ~/battery      (sensor_msgs/msg/BatteryState)
 *
 * Parameters:
 *   serial_port     (string)  default "/dev/ttyAMA0"
 *   slave_id        (int)     default 1     (must be 1..16)
 *   poll_period     (double)  default 1.0   seconds
 *   error_period    (double)  default 2.0   seconds (gap after an error)
 *   resp_timeout_ms (int)     default 500
 *   frame_id        (string)  default "bms"
 */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/serial.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "bms485_ros2/msg/bms_status.hpp"

using namespace std::chrono_literals;

#define BAUDRATE   B9600
#define TOTAL_COMMAND 3

/* ----  Modbus CRC-16 lookup tables (verbatim from the BMS protocol PDF) ---- */
static const uint8_t aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};
static const uint8_t aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

static uint16_t bms_crc16(const uint8_t *frame, uint16_t len)
{
    uint16_t crc_hi = 0xFF;
    uint8_t  crc_lo = 0xFF;
    uint16_t idx;
    while (len--) {
        idx    = crc_lo ^ (*frame++ & 0x00FF);
        crc_lo = (uint8_t)(crc_hi ^ aucCRCHi[idx]);
        crc_hi = aucCRCLo[idx];
    }
    return (uint16_t)(crc_hi << 8 | crc_lo);
}

static int build_query_command(uint8_t *buf, uint8_t slave_id,
                               uint16_t start_addr, uint16_t data_length)
{
    buf[0] = slave_id;
    buf[1] = 0x03;
    buf[2] = (uint8_t)(start_addr >> 8);
    buf[3] = (uint8_t)(start_addr & 0xFF);
    buf[4] = (uint8_t)(data_length >> 8);
    buf[5] = (uint8_t)(data_length & 0xFF);
    uint16_t crc = bms_crc16(buf, 6);
    buf[6] = (uint8_t)(crc & 0xFF);   // CRC LSB first
    buf[7] = (uint8_t)(crc >> 8);     // CRC MSB
    return 8;
}

/* Aggregated decoded BMS state for one poll cycle. */
struct BmsData {
    std::vector<float> cell_voltages;     // V
    std::vector<float> cell_temps;        // K
    float    charge_current    = 0.0f;    // A
    float    discharge_current = 0.0f;    // A
    float    module_voltage    = 0.0f;    // V
    uint16_t soc               = 0;       // %
    uint32_t total_capacity    = 0;       // mAh
    bool     module_valid      = false;
};

class Bms485Node : public rclcpp::Node
{
public:
    Bms485Node() : Node("bms485_node")
    {
        // ----- Parameters -----
        serial_port_     = declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
        slave_id_        = (uint8_t)declare_parameter<int>("slave_id", 1);
        poll_period_     = declare_parameter<double>("poll_period", 1.0);
        error_period_    = declare_parameter<double>("error_period", 2.0);
        resp_timeout_ms_ = declare_parameter<int>("resp_timeout_ms", 500);
        frame_id_        = declare_parameter<std::string>("frame_id", "bms");

        // ----- Publishers -----
        status_pub_  = create_publisher<bms485_ros2::msg::BmsStatus>("~/bms_status", 10);
        battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("~/battery", 10);

        // ----- Open serial -----
        fd_ = serial_open(serial_port_.c_str());
        if (fd_ < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to open serial port '%s'. Shutting down.",
                         serial_port_.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "Polling BMS id %u on %s every %.1f s",
                    slave_id_, serial_port_.c_str(), poll_period_);

        // ----- Start poll thread -----
        running_ = true;
        worker_  = std::thread(&Bms485Node::poll_loop, this);
    }

    ~Bms485Node() override
    {
        running_ = false;
        if (worker_.joinable()) worker_.join();
        if (fd_ >= 0) close(fd_);
    }

private:
    // ---------------- serial helpers ----------------
    int serial_open(const char *device)
    {
        int fd = open(device, O_RDWR | O_NOCTTY);
        if (fd < 0) { RCLCPP_ERROR(get_logger(), "open: %s", strerror(errno)); return -1; }

        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "tcgetattr: %s", strerror(errno));
            close(fd); return -1;
        }

        cfsetispeed(&tty, BAUDRATE);
        cfsetospeed(&tty, BAUDRATE);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(INLCR | ICRNL);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "tcsetattr: %s", strerror(errno));
            close(fd); return -1;
        }

        // Hardware RS485 direction control via RTS/DIR
        struct serial_rs485 rs485;
        memset(&rs485, 0, sizeof(rs485));
        rs485.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
        rs485.delay_rts_before_send = 0;
        rs485.delay_rts_after_send  = 1;
        if (ioctl(fd, TIOCSRS485, &rs485) < 0) {
            RCLCPP_ERROR(get_logger(),
                "TIOCSRS485 failed (driver may not support hardware RS485): %s",
                strerror(errno));
            close(fd); return -1;
        }

        struct serial_rs485 chk;
        memset(&chk, 0, sizeof(chk));
        if (ioctl(fd, TIOCGRS485, &chk) == 0)
            RCLCPP_INFO(get_logger(), "RS485 readback flags = 0x%x %s",
                        chk.flags,
                        (chk.flags & SER_RS485_ENABLED) ? "(ENABLED)" : "(NOT ENABLED!)");

        return fd;
    }

    int rs485_send(const uint8_t *data, size_t len)
    {
        ssize_t w = write(fd_, data, len);
        if (w < 0) return -1;
        tcdrain(fd_);
        return (int)w;
    }

    int read_response(uint8_t *buf, size_t bufsz, int timeout_ms)
    {
        size_t total = 0;
        int first = 1;
        while (total < bufsz) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd_, &rfds);

            struct timeval tv;
            int wait_ms = first ? timeout_ms : 30;   // 30 ms idle gap = frame end
            tv.tv_sec  = wait_ms / 1000;
            tv.tv_usec = (wait_ms % 1000) * 1000;

            int r = select(fd_ + 1, &rfds, NULL, NULL, &tv);
            if (r < 0) { if (errno == EINTR) continue; return -1; }
            if (r == 0) break;

            ssize_t n = read(fd_, buf + total, bufsz - total);
            if (n < 0) { if (errno == EINTR) continue; return -1; }
            if (n == 0) break;

            total += (size_t)n;
            first = 0;
        }
        return (int)total;
    }

    // ---------------- decoding ----------------
    void decode_status_block(uint8_t type, const uint8_t *d, int data_bytes, BmsData &out)
    {
        switch (type) {
            case 0: // cells, 0.001 V per LSB
                for (int i = 0; i + 1 < data_bytes; i += 2) {
                    uint16_t v = (d[i] << 8) | d[i + 1];
                    out.cell_voltages.push_back(v / 1000.0f);
                }
                break;
            case 1: // temps, 0.1 K per LSB
                for (int i = 0; i + 1 < data_bytes; i += 2) {
                    uint16_t t = (d[i] << 8) | d[i + 1];
                    out.cell_temps.push_back(t / 10.0f);
                }
                break;
            case 2: // module
                if (data_bytes < 12) {
                    RCLCPP_WARN(get_logger(), "module block short (%d bytes)", data_bytes);
                    return;
                }
                out.charge_current    = ((d[0] << 8) | d[1]) / 10.0f;   // 0.1 A
                out.discharge_current = ((d[2] << 8) | d[3]) / 10.0f;   // 0.1 A
                out.module_voltage    = ((d[4] << 8) | d[5]) / 100.0f;  // 0.01 V
                out.soc               =  (d[6] << 8) | d[7];            // %
                out.total_capacity    = ((uint32_t)d[8]  << 24) |
                                         ((uint32_t)d[9]  << 16) |
                                         ((uint32_t)d[10] << 8)  |
                                          (uint32_t)d[11];              // mAh
                out.module_valid      = true;
                break;
        }
    }

    // ---------------- publishing ----------------
    void publish(const BmsData &data)
    {
        auto stamp = now();

        // Custom message
        bms485_ros2::msg::BmsStatus msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;
        msg.cell_voltages     = data.cell_voltages;
        msg.cell_temperatures = data.cell_temps;
        msg.charge_current    = data.charge_current;
        msg.discharge_current = data.discharge_current;
        msg.module_voltage    = data.module_voltage;
        msg.soc               = data.soc;
        msg.total_capacity    = data.total_capacity;
        status_pub_->publish(msg);

        // Standard BatteryState
        sensor_msgs::msg::BatteryState bat;
        bat.header.stamp = stamp;
        bat.header.frame_id = frame_id_;
        bat.voltage    = data.module_voltage;
        // current: positive = charging, negative = discharging (ROS convention)
        bat.current    = data.charge_current - data.discharge_current;  //mAh
        bat.charge     = data.total_capacity;
        bat.percentage = data.soc;
        bat.present    = data.module_valid;
        bat.power_supply_status =
            (data.charge_current > 0.0f)
                ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
            : (data.discharge_current > 0.0f)
                ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
                : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
        bat.power_supply_health     = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        bat.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        bat.cell_voltage    = data.cell_voltages;
        bat.cell_temperature = data.cell_temps;
        battery_pub_->publish(bat);
    }

    // Interruptible sleep so shutdown is responsive.
    void interruptible_sleep(double seconds)
    {
        auto end = std::chrono::steady_clock::now() +
                   std::chrono::duration<double>(seconds);
        while (running_ && rclcpp::ok() &&
               std::chrono::steady_clock::now() < end) {
            std::this_thread::sleep_for(50ms);
        }
    }

    // ---------------- main poll loop (runs in its own thread) ----------------
    void poll_loop()
    {
        uint8_t tx[TOTAL_COMMAND][8];
        int     txlen[TOTAL_COMMAND] = {
            build_query_command(tx[0], slave_id_, 0x0004, 8),
            build_query_command(tx[1], slave_id_, 0x0026, 8),
            build_query_command(tx[2], slave_id_, 0x0030, 6)
        };

        while (running_ && rclcpp::ok()) {
            BmsData data;
            bool cycle_ok = true;

            for (uint8_t i = 0; i < TOTAL_COMMAND && running_; i++) {
                tcflush(fd_, TCIFLUSH);

                if (rs485_send(tx[i], txlen[i]) < 0) {
                    RCLCPP_ERROR(get_logger(), "send query failed: %s", strerror(errno));
                    cycle_ok = false;
                    break;
                }

                uint8_t rx[256];
                int n = read_response(rx, sizeof(rx), resp_timeout_ms_);
                if (n <= 0) {
                    RCLCPP_WARN(get_logger(), "cmd %u: no response (timeout)", i);
                    cycle_ok = false;
                    interruptible_sleep(error_period_);
                    continue;
                }
                if (n < 5) {
                    RCLCPP_WARN(get_logger(), "cmd %u: frame too short (%d)", i, n);
                    cycle_ok = false;
                    interruptible_sleep(error_period_);
                    continue;
                }

                uint16_t calc = bms_crc16(rx, n - 2);
                uint16_t recv = (rx[n - 1] << 8) | rx[n - 2];  // LSB then MSB on wire
                if (calc != recv) {
                    RCLCPP_WARN(get_logger(), "cmd %u: CRC mismatch (calc=%04X recv=%04X)",
                                i, calc, recv);
                    cycle_ok = false;
                    interruptible_sleep(error_period_);
                    continue;
                }

                if (rx[1] == (0x03 | 0x80)) {           // abnormal response
                    const char *reason;
                    switch (rx[2]) {
                        case 0x01: reason = "Slave ID out of range"; break;
                        case 0x02: reason = "command type error";    break;
                        case 0x03: reason = "CRC error";             break;
                        default:   reason = "unknown";               break;
                    }
                    RCLCPP_WARN(get_logger(), "cmd %u: BMS error 0x%02X (%s)", i, rx[2], reason);
                    cycle_ok = false;
                } else if (rx[1] == 0x03) {             // normal response
                    uint16_t reg_count  = (rx[2] << 8) | rx[3];
                    int      data_bytes = reg_count * 2;
                    if (data_bytes > n - 5) data_bytes = n - 5; // guard
                    decode_status_block(i, &rx[4], data_bytes, data);
                } else {
                    RCLCPP_WARN(get_logger(), "cmd %u: unexpected command byte 0x%02X", i, rx[1]);
                    cycle_ok = false;
                }
            }

            if (cycle_ok && running_ && rclcpp::ok())
                publish(data);

            interruptible_sleep(poll_period_);
        }
    }

    // ---------------- members ----------------
    int         fd_ = -1;
    std::string serial_port_;
    uint8_t     slave_id_;
    double      poll_period_;
    double      error_period_;
    int         resp_timeout_ms_;
    std::string frame_id_;

    std::thread       worker_;
    std::atomic<bool> running_{false};

    rclcpp::Publisher<bms485_ros2::msg::BmsStatus>::SharedPtr      status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr   battery_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bms485Node>());
    rclcpp::shutdown();
    return 0;
}
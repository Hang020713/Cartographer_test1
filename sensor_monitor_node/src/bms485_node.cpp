#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/serial.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <chrono>
#include <vector>

// ============================================================================
// BMS Data Structure
// ============================================================================
struct BmsData
{
    bool  module_valid      = false;
    float module_voltage    = 0.0f;   // V
    float charge_current    = 0.0f;   // A
    float discharge_current = 0.0f;   // A
    float soc               = 0.0f;   // 0..1 (fraction)
    float total_capacity_ah = 0.0f;   // Ah
    std::vector<float> cell_voltages; // V
    std::vector<float> cell_temps;    // deg C
};

// ============================================================================
// Modbus CRC16 (identical result to the table version in the PDF / C program)
// ============================================================================
static uint16_t bms_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;   // transmit low byte first
}

// ============================================================================
// Build Modbus Query Command
// ============================================================================
static int build_query_command(uint8_t *out, uint8_t slave_id,
                               uint16_t start_reg, uint16_t reg_count)
{
    out[0] = slave_id;
    out[1] = 0x03;
    out[2] = (start_reg >> 8) & 0xFF;
    out[3] =  start_reg       & 0xFF;
    out[4] = (reg_count >> 8) & 0xFF;
    out[5] =  reg_count       & 0xFF;

    uint16_t crc = bms_crc16(out, 6);
    out[6] =  crc       & 0xFF;   // CRC LSB first
    out[7] = (crc >> 8) & 0xFF;
    return 8;
}

// ============================================================================
// Query table  --  MUST match bms485_serial.c
//   idx 0 : 0x0004, 8 regs -> cell voltages     (1 mV  / LSB)
//   idx 1 : 0x0026, 8 regs -> temperatures      (0.1 K / LSB)
//   idx 2 : 0x0030, 6 regs -> module summary
// ============================================================================
enum { CMD_CELLS = 0, CMD_TEMPS = 1, CMD_MODULE = 2, TOTAL_COMMAND = 3 };

struct Query { uint16_t start; uint16_t count; };
static const Query kQueries[TOTAL_COMMAND] = {
    { 0x0004, 8 },
    { 0x0026, 8 },
    { 0x0030, 6 },
};

// ============================================================================
// BMS485 Node
// ============================================================================
class Bms485Node : public rclcpp::Node
{
public:
    Bms485Node() : Node("bms485_node")
    {
        serial_port_        = declare_parameter<std::string>("serial_port", "/dev/ttyAMA3");
        slave_id_           = static_cast<uint8_t>(declare_parameter<int>("slave_id", 1));
        resp_timeout_ms_    = declare_parameter<int>("resp_timeout_ms", 500);
        frame_id_           = declare_parameter<std::string>("frame_id", "bms");
        autonomous_polling_ = declare_parameter<bool>("autonomous_polling", true);
        poll_period_        = declare_parameter<double>("poll_period", 1.0);
        inter_cmd_ms_       = declare_parameter<int>("inter_cmd_delay_ms", 20);
        require_rs485_      = declare_parameter<bool>("require_rs485_ioctl", true);
        debug_frames_       = declare_parameter<bool>("debug_frames", false);

        battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("~/battery", 10);

        trigger_service_ = create_service<std_srvs::srv::Trigger>(
            "~/trigger_read",
            std::bind(&Bms485Node::handle_trigger_read, this,
                      std::placeholders::_1, std::placeholders::_2));

        fd_ = serial_open(serial_port_.c_str());
        if (fd_ < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to open serial port '%s'. Shutting down.",
                         serial_port_.c_str());
            rclcpp::shutdown();
            return;
        }

        if (autonomous_polling_) {
            RCLCPP_INFO(get_logger(),
                "BMS polling on %s (slave_id=%u, period=%.1f s)",
                serial_port_.c_str(), slave_id_, poll_period_);
            timer_ = create_wall_timer(
                std::chrono::duration<double>(poll_period_),
                std::bind(&Bms485Node::timer_callback, this));
        } else {
            RCLCPP_INFO(get_logger(),
                "BMS node ready on %s (slave_id=%u, trigger-based polling only)",
                serial_port_.c_str(), slave_id_);
        }
    }

    ~Bms485Node() override { if (fd_ >= 0) close(fd_); }

private:
    // ---------------- Serial port open ----------------
    int serial_open(const char* device)
    {
        // NOTE: no O_NDELAY -- we control blocking with select() + VMIN/VTIME=0,
        // exactly like the working C program.
        int fd = open(device, O_RDWR | O_NOCTTY);
        if (fd < 0) {
            RCLCPP_ERROR(get_logger(), "Cannot open %s: %s", device, strerror(errno));
            return -1;
        }

        struct termios tio;
        if (tcgetattr(fd, &tio) != 0) {
            RCLCPP_ERROR(get_logger(), "tcgetattr: %s", strerror(errno));
            close(fd); return -1;
        }

        cfsetispeed(&tio, B9600);
        cfsetospeed(&tio, B9600);

        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |=  CS8;
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cflag |= (CLOCAL | CREAD);

        tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
        tio.c_oflag &= ~OPOST;

        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 0;

        if (tcsetattr(fd, TCSANOW, &tio) != 0) {
            RCLCPP_ERROR(get_logger(), "tcsetattr: %s", strerror(errno));
            close(fd); return -1;
        }
        tcflush(fd, TCIOFLUSH);

        // Hardware RS485 direction control (same values as the working C code)
        struct serial_rs485 rs485;
        memset(&rs485, 0, sizeof(rs485));
        rs485.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
        rs485.delay_rts_before_send = 0;
        rs485.delay_rts_after_send  = 1;      // <-- was 0; 1 ms turnaround like C version

        if (ioctl(fd, TIOCSRS485, &rs485) < 0) {
            RCLCPP_ERROR(get_logger(), "ioctl(TIOCSRS485) failed: %s", strerror(errno));
            if (require_rs485_) {             // without DE control you read your own TX
                close(fd);
                return -1;
            }
        } else {
            struct serial_rs485 chk;
            memset(&chk, 0, sizeof(chk));
            if (ioctl(fd, TIOCGRS485, &chk) == 0)
                RCLCPP_INFO(get_logger(), "RS485 readback flags = 0x%x %s", chk.flags,
                            (chk.flags & SER_RS485_ENABLED) ? "(ENABLED)" : "(NOT ENABLED!)");
        }
        return fd;
    }

    // ---------------- RS485 send ----------------
    int rs485_send(const uint8_t* data, int len)
    {
        int written = static_cast<int>(write(fd_, data, len));
        if (written < 0) return -1;
        tcdrain(fd_);
        return written;
    }

    // ---------------- Read one frame: first-byte timeout + idle-gap ----------------
    int read_response(uint8_t* buf, size_t bufsize, int timeout_ms, int gap_ms = 30)
    {
        size_t total = 0;
        bool   first = true;

        while (total < bufsize) {
            fd_set rfds; FD_ZERO(&rfds); FD_SET(fd_, &rfds);

            int wait_ms = first ? timeout_ms : gap_ms;   // 30 ms silence == end of frame
            struct timeval tv;
            tv.tv_sec  = wait_ms / 1000;
            tv.tv_usec = (wait_ms % 1000) * 1000;

            int r = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
            if (r < 0) { if (errno == EINTR) continue; return -1; }
            if (r == 0) break;                            // timeout -> done

            ssize_t n = read(fd_, buf + total, bufsize - total);
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
                return -1;
            }
            if (n == 0) break;

            total += static_cast<size_t>(n);
            first  = false;
        }
        return static_cast<int>(total);
    }

    // ---------------- Decode (mirrors decode_status_block() in the C program) -------
    void decode_status_block(int cmd_index, const uint8_t* d, int len, BmsData& out)
    {
        auto u16 = [&](int off) -> uint16_t {
            return static_cast<uint16_t>((d[off] << 8) | d[off + 1]);
        };

        switch (cmd_index) {

        case CMD_CELLS:                       // 0x0004..: 1 mV per LSB
            out.cell_voltages.clear();
            for (int i = 0; i + 1 < len; i += 2)
                out.cell_voltages.push_back(u16(i) * 0.001f);
            break;

        case CMD_TEMPS:                       // 0x0026..: 0.1 C per LSB
            out.cell_temps.clear();
            for (int i = 0; i + 1 < len; i += 2)
                out.cell_temps.push_back(u16(i) * 0.1f);
            break;

        case CMD_MODULE:                      // 0x0030.. : 6 registers = 12 bytes
            if (len < 12) {
                RCLCPP_WARN(get_logger(), "module block short (%d bytes)", len);
                return;
            }
            out.charge_current    = u16(0) * 0.1f;    // 0.1 A
            out.discharge_current = u16(2) * 0.1f;    // 0.1 A
            out.module_voltage    = u16(4) * 0.01f;   // 0.01 V
            out.soc               = u16(6);
            {   // 32-bit capacity in mAh
                uint32_t cap_mah = (static_cast<uint32_t>(d[8])  << 24) |
                                   (static_cast<uint32_t>(d[9])  << 16) |
                                   (static_cast<uint32_t>(d[10]) <<  8) |
                                   (static_cast<uint32_t>(d[11]));
                out.total_capacity_ah = cap_mah / 1000.0f;
            }
            out.module_valid = true;
            break;

        default: break;
        }
    }

    // ---------------- Timer ----------------
    void timer_callback()
    {
        BmsData data;
        if (perform_single_poll(data)) publish(data);
    }

    // ---------------- Publish ----------------
    void publish(const BmsData &data)
    {
        sensor_msgs::msg::BatteryState bat;
        bat.header.stamp    = now();
        bat.header.frame_id = frame_id_;

        bat.voltage    = data.module_voltage;
        // ROS convention: negative when discharging, positive when charging
        bat.current    = data.charge_current - data.discharge_current;
        bat.percentage = data.soc;                                   // 0.0 .. 1.0
        bat.capacity        = data.total_capacity_ah;                // Ah
        bat.design_capacity = data.total_capacity_ah;                // Ah
        bat.charge          = data.soc * data.total_capacity_ah;     // Ah remaining
        bat.present    = data.module_valid;

        if (data.charge_current > 0.05f)
            bat.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        else if (data.discharge_current > 0.05f)
            bat.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        else
            bat.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;

        bat.power_supply_health     = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        bat.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

        bat.cell_voltage     = data.cell_voltages;
        bat.cell_temperature = data.cell_temps;

        battery_pub_->publish(bat);
    }

    // ---------------- Trigger service ----------------
    void handle_trigger_read(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        BmsData data;
        if (perform_single_poll(data)) {
            publish(data);
            response->success = true;
            response->message = "BMS poll complete";
        } else {
            response->success = false;
            response->message = "BMS poll failed";
        }
    }

    // ---------------- One full poll (3 commands) ----------------
    bool perform_single_poll(BmsData &data)
    {
        uint8_t tx[TOTAL_COMMAND][8];
        int     txlen[TOTAL_COMMAND];
        for (int i = 0; i < TOTAL_COMMAND; ++i)
            txlen[i] = build_query_command(tx[i], slave_id_,
                                           kQueries[i].start, kQueries[i].count);

        bool all_ok = true;

        for (int i = 0; i < TOTAL_COMMAND; i++) {
            tcflush(fd_, TCIFLUSH);

            if (rs485_send(tx[i], txlen[i]) < 0) {
                RCLCPP_ERROR(get_logger(), "send query failed: %s", strerror(errno));
                return false;
            }
            if (debug_frames_) hexdump("TX", tx[i], txlen[i]);

            uint8_t rx[256];
            int n = read_response(rx, sizeof(rx), resp_timeout_ms_);
            if (debug_frames_ && n > 0) hexdump("RX", rx, n);

            if (n <= 0)  { RCLCPP_WARN(get_logger(), "cmd %d: no response", i);            all_ok = false; continue; }
            if (n < 7)   { RCLCPP_WARN(get_logger(), "cmd %d: frame too short (%d)", i, n); all_ok = false; continue; }

            uint16_t calc = bms_crc16(rx, n - 2);
            uint16_t recv = static_cast<uint16_t>((rx[n - 1] << 8) | rx[n - 2]);
            if (calc != recv) {
                RCLCPP_WARN(get_logger(), "cmd %d: CRC mismatch (calc=%04X recv=%04X)", i, calc, recv);
                all_ok = false; continue;
            }
            if (rx[0] != slave_id_) {
                RCLCPP_WARN(get_logger(), "cmd %d: wrong slave id 0x%02X", i, rx[0]);
                all_ok = false; continue;
            }

            if (rx[1] == (0x03 | 0x80)) {                    // exception response
                const char* txt = "unknown";
                switch (rx[2]) {
                    case 0x01: txt = "slave ID out of range"; break;
                    case 0x02: txt = "command type error";    break;
                    case 0x03: txt = "CRC error";             break;
                }
                RCLCPP_WARN(get_logger(), "cmd %d: BMS error 0x%02X (%s)", i, rx[2], txt);
                all_ok = false; continue;
            }
            if (rx[1] != 0x03) {
                RCLCPP_WARN(get_logger(), "cmd %d: unexpected function 0x%02X", i, rx[1]);
                all_ok = false; continue;
            }

            // Frame: [0]=id [1]=0x03 [2..3]=register count [4..]=data [n-2..n-1]=CRC
            uint16_t reg_count  = static_cast<uint16_t>((rx[2] << 8) | rx[3]);
            int      data_bytes = reg_count * 2;
            const int avail     = n - 6;                     // <-- was n - 5 (off by one)
            if (data_bytes > avail) data_bytes = avail;
            if (data_bytes <= 0) { all_ok = false; continue; }

            decode_status_block(i, &rx[4], data_bytes, data);

            if (inter_cmd_ms_ > 0 && i + 1 < TOTAL_COMMAND)
                usleep(inter_cmd_ms_ * 1000);
        }

        // Publish only if at least the module block came back
        return all_ok && data.module_valid;
    }

    void hexdump(const char* label, const uint8_t* b, int n)
    {
        char line[3 * 256 + 1]; int p = 0;
        for (int i = 0; i < n && p < (int)sizeof(line) - 3; i++)
            p += snprintf(line + p, sizeof(line) - p, "%02X ", b[i]);
        RCLCPP_INFO(get_logger(), "%s (%d): %s", label, n, line);
    }

    // ---------------- Members ----------------
    int         fd_ = -1;
    std::string serial_port_;
    uint8_t     slave_id_;
    int         resp_timeout_ms_;
    std::string frame_id_;
    bool        autonomous_polling_;
    double      poll_period_;
    int         inter_cmd_ms_;
    bool        require_rs485_;
    bool        debug_frames_;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bms485Node>());
    rclcpp::shutdown();
    return 0;
}
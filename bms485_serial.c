/* bms485.c  -  Pi 5 / RP1 hardware-RTS RS485 BMS poller (Modbus-style protocol)
 *
 * Auth: B
 * Build:  gcc -Wall -O2 -o bms485 bms485.c
 * Run:    sudo ./bms485
 *
 * Wiring (RS485 transceiver with combined DIR pin):
 *   DI  -> GPIO14 / pin 8  (TXD0)
 *   RO  -> GPIO15 / pin 10 (RXD0)
 *   DIR -> GPIO17 / pin 11 (RTS0, auto-toggled by the kernel)
 *   GND -> pin 6 ;  VCC -> 3.3V (verify your module is 3.3V-safe!)
 *
 * Protocol: 9600 8N1. Query = ID, 0x03, AddrMSB, AddrLSB, CntMSB, CntLSB, CRClo, CRChi
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/serial.h>

#define SERIAL_PORT       "/dev/ttyAMA0"
#define BAUDRATE          B9600
#define SLAVE_ID          0x01     /* BMS address, must be 1..16 */
#define ERROR_PERIOD_SEC 2
#define POLL_PERIOD_SEC   1
#define RESP_TIMEOUT_MS   500
#define QUERY_START_ADDRESS 0x30        // BMS general status parameters inquiry: Start from 0x0003
#define QUERY_DATA_LENGTH 6             // Query how many data(int for one kind of data)

/* ----  Modbus CRC-16 Lookup tables, copied verbatim from the BMS protocol PDF ---- */
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


// Functions FPF
int  serial_open(const char *device);
static uint16_t bms_crc16(const uint8_t *frame, uint16_t len);
int  rs485_send(int fd, const uint8_t *data, size_t len);
int  read_response(int fd, uint8_t *buf, size_t bufsz, int timeout_ms);
void hexdump(const char *label, const uint8_t *buf, int n);     // Printout the hex with label
int build_query_command(uint8_t *buf, uint8_t slave_id, uint16_t start_addr, uint16_t data_length);

/* Decode the 0x0030..0x0035 block (6 registers, 12 data bytes). */
void decode_status_block(const uint8_t *d, int data_bytes)
{
    if (data_bytes < 12) { printf("  (short data, %d bytes)\n", data_bytes); return; }
    uint16_t chg_i  = (d[0]  << 8) | d[1];   /* 0.1A  */
    uint16_t dis_i  = (d[2]  << 8) | d[3];   /* 0.1A  */
    uint16_t volt   = (d[4]  << 8) | d[5];   /* 0.1V  */
    uint16_t soc    = (d[6]  << 8) | d[7];   /* %     */
    uint32_t cap    = ((uint32_t)d[8] << 24) | ((uint32_t)d[9] << 16) |
                      ((uint32_t)d[10] << 8) |  (uint32_t)d[11];  /* mAh, 4 bytes */
    printf("  Charge current   : %.1f A\n",  chg_i / 10.0);
    printf("  Discharge current: %.1f A\n",  dis_i / 10.0);
    printf("  Module voltage   : %.1f V\n",  volt  / 10.0);
    printf("  SOC              : %u %%\n",   soc);
    printf("  Total capacity   : %u mAh\n",  cap);
}

int main(void)
{
    // Open serial
    int fd = serial_open(SERIAL_PORT);
    if (fd < 0) return 1;

    // Function Variables
    uint8_t tx[8];
    uint8_t rx[256];

    // Build read command
    int txlen = build_query_command(tx, SLAVE_ID, QUERY_START_ADDRESS, QUERY_DATA_LENGTH);  /* live-data block */
    printf("Polling BMS id %d on %s every %d s ... (Ctrl-C to quit)\n", SLAVE_ID, SERIAL_PORT, POLL_PERIOD_SEC);

    // Start reading
    while (1) {
        tcflush(fd, TCIFLUSH);  // Clear buffer

        // Send query command
        hexdump("TX", tx, txlen);
        if (rs485_send(fd, tx, txlen) < 0) 
        { 
            perror("send query command"); 
            break; 
        }

        // Read response
        int n = read_response(fd, rx, sizeof(rx), RESP_TIMEOUT_MS);
        if (n <= 0) {
            printf("RX: (no response - timeout)\n----\n");
            sleep(ERROR_PERIOD_SEC);    // Take a gap
            continue;
        }
        hexdump("RX", rx, n);

        // Validation response format, at least 5 bytes
        if (n < 5) 
        { 
            printf("  frame too short\n----\n"); 
            sleep(ERROR_PERIOD_SEC);     // Take a gap
            continue; 
        }

        // CRC check (received CRC is LSB then MSB)
        uint16_t calc = bms_crc16(rx, n - 2);
        uint16_t recv = rx[n - 2] | (rx[n - 1] << 8);   // Convert to MSB, LSB
        if (calc != recv) {
            printf("  CRC mismatch (calc=%04X recv=%04X)\n----\n", calc, recv);
            sleep(ERROR_PERIOD_SEC);     // Take a gap
            continue;
        }

        // Check response
        if (rx[1] == (0x03 | 0x80)) {            /* abnormal response, command type + 128 */
            printf("  BMS ERROR, code 0x%02X ", rx[2]);
            switch (rx[2]) {
                case 0x01: printf("(Slave ID out of range)\n"); break;
                case 0x02: printf("(command type error)\n");    break;
                case 0x03: printf("(CRC error)\n");             break;
                default:   printf("(unknown)\n");               break;
            }
        } else if (rx[1] == 0x03) {              /* normal response */
            uint16_t reg_count  = (rx[2] << 8) | rx[3];
            int      data_bytes = reg_count * 2;
            printf("  OK: %u registers (%d data bytes)\n", reg_count, data_bytes);
            decode_status_block(&rx[4], data_bytes);
        } else {
            printf("  unexpected command byte 0x%02X\n", rx[1]);
        }

        printf("----\n");
        sleep(POLL_PERIOD_SEC);
    }

    close(fd);
    return 0;
}

int serial_open(const char *device)
{   // https://stackoverflow.com/questions/17254923/raspberry-pi-uart-program-in-c-using-termios-receives-garbage-rx-and-tx-are-con
    // Open Serial Port (UART), // O_RDWR: Read/Write | O_NOCTTY: Prevents terminal control | O_NDELAY: Non-blocking open
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open serial"); return -1; }

    // Configure termious tty
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) { perror("tcgetattr"); close(fd); return -1; }

    // Set Baud Rate to 9600
    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    // Set tty settings
    tty.c_cflag &= ~PARENB;         // no parity
    tty.c_cflag &= ~CSTOPB;         // 1 stop bit
    tty.c_cflag &= ~CSIZE;          // Clear the size bits, ~CSIZE = fffffcff
    tty.c_cflag |= CS8;             // 8 data bits
    tty.c_cflag &= ~CRTSCTS;        // no hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;  // enable receiver

    /* Raw mode: no canonical processing, no echo, no signals */
    // Disable software flow control and parity/mapping options
    // ICANON, canonical mode, handle line by line
    // ECHO, echoing?
    // ISIG, formatting
    // INLCR, Newline
    // ICRNL, Newline
    // OPOST, output processing
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    // Non-Blocking read
    tty.c_cc[VMIN]  = 0;    // Minimum byte to read
    tty.c_cc[VTIME] = 0;    // Timeout

    // Apply settings
    if (tcsetattr(fd, TCSANOW, &tty) != 0) 
    { 
        perror("tcsetattr"); 
        close(fd); 
        return -1; 
    }

    // Hardware RS485 direction control via RTS/DIR
    struct serial_rs485 rs485;  //Linux-kernel structure, <linux/serial.h>
    memset(&rs485, 0, sizeof(rs485));
    rs485.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
    rs485.delay_rts_before_send = 0;
    rs485.delay_rts_after_send  = 1;
    if (ioctl(fd, TIOCSRS485, &rs485) < 0)
    {
        perror("TIOCSRS485 (driver may not support hardware RS485)");
        close(fd);
        return -1;
    }

    // TIOCGRS485 = Terminal IO Control, Get RS485
    struct serial_rs485 chk;
    memset(&chk, 0, sizeof(chk));
    if (ioctl(fd, TIOCGRS485, &chk) == 0)
    {
        fprintf(stderr, "RS485 readback flags = 0x%x %s\n", chk.flags, (chk.flags & SER_RS485_ENABLED) ? "(ENABLED)" : "(NOT ENABLED!)");
        close(fd);
        return -1;
    }

    // Serial open success
    return fd;
}

// CRC function from the PDF
static uint16_t bms_crc16(const uint8_t *frame, uint16_t len)
{
    uint16_t crc_hi = 0xFF;     // CRC high
    uint8_t  crc_lo = 0xFF;     // CRC low
    uint16_t idx;               // Index

    while (len--) {
        idx    = crc_lo ^ (*frame++ & 0x00FF);
        crc_lo = (uint8_t)(crc_hi ^ aucCRCHi[idx]);
        crc_hi = aucCRCLo[idx];
    }

    // LSB & MSB
    return (uint16_t)(crc_lo << 8 | crc_hi);
}

int rs485_send(int fd, const uint8_t *data, size_t len)
{   // https://linux.die.net/man/2/write
    // Write data
    ssize_t w = write(fd, data, len);

    if (w < 0) return -1;   // If number of bytes return < 0 means error
    tcdrain(fd);            // Clear buffer

    // Return written byte length
    return (int)w;  
}

int read_response(int fd, uint8_t *buf, size_t bufsz, int timeout_ms)
{
    size_t total = 0;   // Total read length
    int first = 1;      // Its first response?

    // Start Loop
    while (total < bufsz) {
        // Setup the filedescriptor
        fd_set rfds; 
        FD_ZERO(&rfds); 
        FD_SET(fd, &rfds);

        // Set timeout structure
        struct timeval tv;
        int wait_ms = first ? timeout_ms : 30;   /* 30 ms idle gap = frame end */
        tv.tv_sec  = wait_ms / 1000;
        tv.tv_usec = (wait_ms % 1000) * 1000;

        // Wait data(select)
        int r = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) 
        {   // Error handling
            if (errno == EINTR) continue; // Interrupted system call
            return -1; 
        }
        if (r == 0) break;    // Timeout

        // Get data, https://linux.die.net/man/2/read
        ssize_t n = read(fd, buf + total, bufsz - total);
        if (n < 0) 
        {   // Error handling
            if (errno == EINTR) continue; 
            return -1; 
        }
        if (n == 0) break;  // EOF

        // Update read length, first time & continue
        total += (size_t)n;
        first = 0;
    }

    // Return read length
    return (int)total;
}

void hexdump(const char *label, const uint8_t *buf, int n)
{
    printf("%s (%d bytes):", label, n);
    for (int i = 0; i < n; i++) printf(" %02X", buf[i]);
    printf("\n");
}

int build_query_command(uint8_t *buf, uint8_t slave_id, uint16_t start_addr, uint16_t data_length)
{
    buf[0] = slave_id;                                  // Slave_ID
    buf[1] = 0x03;                                      // Command Type
    buf[2] = (uint8_t)(start_addr >> 8);                // Start_address MSB
    buf[3] = (uint8_t)(start_addr & 0xFF);              // Start_address LSB
    buf[4] = (uint8_t)(data_length >> 8);               // Data_length MSB
    buf[5] = (uint8_t)(data_length & 0xFF);             // Data_length LSB

    // CRC
    uint16_t crc = bms_crc16(buf, 6);
    buf[6] = (uint8_t)(crc & 0xFF);                     // CRC LSB first
    buf[7] = (uint8_t)(crc >> 8);                       // CRC MSB
    
    // Return command package length
    return 8;
}
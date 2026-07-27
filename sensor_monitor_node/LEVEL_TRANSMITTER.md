# Level Transmitter Node

`level_transmitter_node` is a ROS 2 node in the `sensor_monitor` package that reads measurements from a submersible level/pressure transmitter via the **Modbus RTU over RS-485** protocol and publishes the readings to a ROS 2 topic.

---

## Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Raspberry Pi CM5 Port Configuration](#raspberry-pi-cm5-port-configuration)
- [RS-485 Wiring](#rs-485-wiring)
- [Build & Run](#build--run)
- [Parameters](#parameters)
- [Topic](#topic)
- [Modbus Protocol](#modbus-protocol)
- [Data Flow & Processing](#data-flow--processing)
- [Log Output Reference](#log-output-reference)
- [Troubleshooting](#troubleshooting)

---

## Hardware Requirements

| Component | Model / Specification |
|-----------|----------------------|
| Controller | Raspberry Pi CM5 + SCH_CM5 carrier board |
| Sensor | Submersible level/pressure transmitter (Modbus RTU compatible) |
| Physical Interface | RS-485 half-duplex (A/B differential signaling) |
| UART | CM5 UART3 (`/dev/ttyAMA3`), **CTS/RTS** hardware flow control required for RS-485 direction switching |

---

## Raspberry Pi CM5 Port Configuration

### 1. Edit `/boot/firmware/config.txt`

```bash
sudo nano /boot/firmware/config.txt
```

Add the following:

```ini
[all]
# Enable UART3 with CTS/RTS pins (required for RS-485 direction control)
dtoverlay=uart3-pi5,ctsrts
```

> **Note:**
> - `uart3-pi5` — enables UART3 on Pi 5 / CM5 (mapped to `/dev/ttyAMA3`)
> - `ctsrts` — enables CTS/RTS hardware flow control pins. **The code uses the RTS pin to automatically control the RS-485 transceiver direction** (RTS high enables the driver for TX mode; RTS low enables the receiver for RX mode). The `ctsrts` parameter is therefore **required**.

### 2. Disable Bluetooth (optional, frees UART0)

If your application does not use Bluetooth, it is recommended to disable it to free up `ttyAMA0`:

```ini
[all]
dtoverlay=disable-bt
```

### 3. Reboot to apply

```bash
sudo reboot
```

### 4. Verify the serial port

```bash
# Check that the UART3 device exists
ls -la /dev/ttyAMA3

# Expected output:
# crw-rw---- 1 root dialout ... /dev/ttyAMA3
```

### 5. Grant user access to the serial port

```bash
sudo usermod -aG dialout $USER
# Re-login for the change to take effect
```

---

## RS-485 Wiring

RS-485 half-duplex communication requires only **2 differential signal lines + GND**. The TX/RX direction is automatically controlled via the RTS pin.

```
CM5 Pins (UART3)              RS-485 Transceiver          Transmitter
─────────────────           ──────────────           ──────────
TXD  (GPIO 8)    ───────→   DI                         
RXD  (GPIO 9)    ←───────   RO                         
RTS  (GPIO 11)   ───────→   DE + ~RE  (direction)      
                            A         ───────────────  A (485+)
                            B         ───────────────  B (485-)
GND              ───────→   GND       ───────────────  GND
```

> **Key point**: With `SER_RS485_RTS_ON_SEND` configured, the kernel driver automatically asserts RTS → DE=1 (TX mode) during `write()` and de-asserts RTS → DE=0 (RX mode) immediately after transmission completes. No manual RTS control is needed at the application level.

---

## Build & Run

### Build

```bash
cd ~/boat_control_ws
colcon build --symlink-install --packages-select sensor_monitor
source install/setup.bash
```

### Run (default parameters)

```bash
ros2 run sensor_monitor level_transmitter_node
```

### Run (Debug mode — view raw Modbus bytes)

```bash
ros2 run sensor_monitor level_transmitter_node --ros-args --log-level debug
```

### Run (custom parameters)

```bash
ros2 run sensor_monitor level_transmitter_node --ros-args \
  -p serial_port:="/dev/ttyAMA3" \
  -p baud_rate:=9600 \
  -p slave_id:=1 \
  -p read_interval_ms:=2000 \
  -p use_float_mode:=true \
  -p topic_name:="~/level"
```

---

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | `/dev/ttyAMA3` | RS-485 serial device path |
| `baud_rate` | int | `9600` | Baud rate (supports 1200/2400/4800/9600/19200/38400) |
| `slave_id` | int | `1` | Modbus slave address (1–255) |
| `read_interval_ms` | int | `2000` | Data read interval (milliseconds) |
| `use_float_mode` | bool | `true` | `true` = read from floating-point registers; `false` = read from integer registers |
| `topic_name` | string | `~/level` | ROS 2 topic name for publishing (`~` expands to the node name) |

---

## Topic

| Topic | Type | Description |
|-------|------|-------------|
| `~/level` (default) | `std_msgs/msg/Float64` | Level/pressure measurement (sanitized, rounded to 2 decimal places) |

### Monitor data

```bash
ros2 topic echo /level_transmitter_node/level
```

---

## Modbus Protocol

### Communication Parameters

| Item | Value |
|------|-------|
| Physical layer | RS-485 half-duplex |
| Protocol | Modbus RTU |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Frame format | 8N1 |
| CRC | CRC16 (polynomial 0xA001) |
| Inter-frame gap | ≥ 3.5 character times (code uses a 50 ms safety wait) |

### Registers Read

| Address | Name | Length | Data Type | Description |
|---------|------|--------|-----------|-------------|
| `0x0002` | Measurement unit | 1 register (2 B) | uint16 | Read once at startup to determine display unit |
| `0x0003` | Decimal places | 1 register (2 B) | uint16 (0–4) | Integer mode: `value = raw / 10^dp` |
| `0x0004` | Measurement (integer) | 1 register (2 B) | int16 | Raw value for integer mode |
| `0x0016` | Measurement (float) | 2 registers (4 B) | IEEE 754 float | Float mode: big-endian byte order |

### Unit Codes

| Code | Unit | Code | Unit | Code | Unit |
|------|------|------|------|------|------|
| 0 | MPa | 8 | mmH₂O | 16 | m |
| 1 | KPa | 9 | inH₂O | 17 | cm |
| 2 | Pa | 10 | H₂O | 18 | mm |
| 3 | bar | 11 | mHg | 19 | Kg |
| 4 | mbar | 12 | mmHg | 20 | °C |
| 5 | Kg/cm² | 13 | inHg | 21 | PH |
| 6 | PSI | 14 | atm | 22 | °F |
| 7 | mH₂O | 15 | Torr | 23 | (none) |

---

## Data Flow & Processing

```
┌──────────┐   RS-485     ┌──────────────┐   Modbus RTU   ┌───────────────────────┐
│Transmitter│ ◄──────────► │  CM5 UART3   │ ◄────────────► │ level_transmitter_node │
│ (slave 1) │   A/B diff.  │ /dev/ttyAMA3 │                │                       │
└──────────┘              └──────────────┘                └───────────┬───────────┘
                                                                      │
                                                       ┌──────────────┘
                                                       ▼
                                              ┌─────────────────┐
                                              │ timer_callback() │  fires every 2 s
                                              └────────┬────────┘
                                                       │
                                          ┌────────────┴────────────┐
                                          ▼                         ▼
                                  read_float_value()        read_integer_value()
                                  reads 0x0016 (4 B)        reads 0x0003 + 0x0004
                                          │                         │
                                          └────────────┬────────────┘
                                                       ▼
                                             sanitize_measurement()
                                             ┌─ Near-zero clamp (< 1e-9) → 0.0
                                             ├─ Outlier rejection (> 1e6) → discard
                                             └─ Round to 2 decimal places
                                                       │
                                                       ▼
                                              publish(topic, value)
```

### Value Sanitization Rules

| Scenario | Raw Value Example | Action | Result |
|----------|------------------|--------|--------|
| Sensor idle (in air) | `-1.15×10⁻¹⁹` | Clamp to 0 | `0.00` |
| Communication noise / dirty data | `9.62×10¹⁶` | Reject + WARN log | *(not published)* |
| Normal reading | `0.00056349` → `6.40` | Rounded | `6.40` |

---

## Log Output Reference

### Debug Mode (`--log-level debug`)

Each read cycle outputs a complete hex dump of the TX request frame and RX response frame:

```
[DEBUG] [level_transmitter_node]: TX [8]: 01 03 00 16 00 02 25 cf
[DEBUG] [level_transmitter_node]: RX [9]: 01 03 04 40 cc b0 29 9b d2
```

**Frame format breakdown (TX example):**

| Byte | Meaning |
|------|---------|
| `01` | Slave address |
| `03` | Function code (Read Holding Registers) |
| `00 16` | Starting address 0x0016 |
| `00 02` | Register count 2 |
| `25 cf` | CRC16 checksum |

**RX data decoding example:**

```
RX: 01 03 04 40 CC B0 29 9B D2
    │  │  │  └───────┘ └───┘
    │  │  │     │        └── CRC16
    │  │  │     └── IEEE 754 float big-endian → decodes to ~6.40
    │  │  └── Byte count 4
    │  └── Function code 03
    └── Slave address 01
```

### Normal Mode

Outputs only startup information, anomaly warnings, and periodic measurement publications:

```
[INFO] [level_transmitter_node]: Level Transmitter node started. port=/dev/ttyAMA3 baud=9600 slave_id=1 unit=cm mode=float
[INFO] [level_transmitter_node]: RS-485 mode enabled (RTS direction control)
[INFO] [level_transmitter_node]: Sensor unit code: 17 (cm)
[DEBUG] [level_transmitter_node]: Published: 6.40 cm
```

---

## Troubleshooting

### 1. `Cannot open /dev/ttyAMA3`

```bash
# Check if the device exists
ls -la /dev/ttyAMA*

# Check if the uart3-pi5 overlay is configured in config.txt
grep uart3 /boot/firmware/config.txt

# Check if the user is in the dialout group
groups $USER | grep dialout
```

### 2. `Modbus timeout: no response from slave`

- Check if the RS-485 A/B lines are swapped (A↔A, B↔B)
- Verify that the transmitter is powered correctly
- Use an oscilloscope to confirm differential signals are present on the bus
- Confirm the `slave_id` parameter matches the transmitter's address
- Verify that `ctsrts` is enabled in config.txt (without the RTS signal, the driver stays in receive mode and never transmits data onto the bus)

### 3. `TIOCSRS485 ioctl failed`

- Kernel version is too old and does not support `TIOCSRS485`
- The UART hardware does not support RS-485 mode
- Basic serial communication is unaffected, but **RS-485 direction control will not function**. Check whether an external hardware auto-direction control circuit is present.

### 4. Published value is always `0.00`

Common causes:

- **Incorrect byte order in `be_bytes_to_float`** (already fixed in the latest code)
- The sensor is genuinely measuring zero (in air, or the liquid level is actually zero)
- Communication frames are silently dropped due to CRC errors — inspect with `--log-level debug`

### 5. `CRC error`

- Missing RS-485 bus termination resistors (120 Ω should be placed at both ends of the bus)
- Excessively long cabling or strong electromagnetic interference in the environment
- Baud rate is too high — try lowering to 9600 or 4800

### 6. `Serial read error`

- Serial port is occupied by another process: `sudo lsof /dev/ttyAMA3`
- Kernel driver anomaly: `dmesg | grep ttyAMA`

---

## Code Architecture

```
level_transmitter_node.cpp
│
├─ Anonymous namespace ─────────────────────────
│  ├─ to_hex_string()          hex conversion utility
│  ├─ modbus_crc16()           CRC16 calculation
│  ├─ unit_code_to_string()    unit code lookup table
│  ├─ baud_to_speed()          baud rate → termios constant
│  └─ be_bytes_to_float()      IEEE 754 big-endian → native float
│
└─ class LevelTransmitterNode ─────────────────
   │
   ├─ Initialization ──────────────────────────
   │  ├─ load_parameters()       load ROS parameters
   │  ├─ open_serial_port()      open serial port + 8N1 + timeout
   │  │  ├─ configure_baud_rate()
   │  │  └─ configure_rs485_mode()   kernel-level RTS direction control
   │  ├─ create_publishers()
   │  └─ read_sensor_metadata()  read unit code
   │
   ├─ Modbus Communication ────────────────────
   │  ├─ read_holding_registers() orchestration layer (5 steps)
   │  │  ├─ build_request_frame()
   │  │  ├─ write_request_frame()
   │  │  ├─ read_response_frame()
   │  │  ├─ validate_response_frame()
   │  │  └─ extract_register_values()
   │  └─ log_hex()             debug hex dump
   │
   └─ Measurement & Publishing ────────────────
      ├─ timer_callback()
      ├─ read_float_value()      float mode
      ├─ read_integer_value()    integer mode
      └─ sanitize_measurement()  value sanitization
```

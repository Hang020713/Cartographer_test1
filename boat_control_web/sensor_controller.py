"""
sensor_controller.py — I2C sensor controller with level transmitter support.

Reads SHT3x temperature/humidity and INA4230 motor currents via I2C,
and level transmitter values via Modbus RTU.
Follows the same design pattern as BMSController: background thread + threading.Lock + get_status().
"""

import logging
import os
import struct
import threading
import time
from collections import deque
from typing import Optional

# ── Low-level I2C constants ─────────────────────────────────────
I2C_SLAVE = 0x0703  # ioctl command to set I2C slave address

LOG_MAX_LEN = 100
LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"


class ListLogHandler(logging.Handler):
    """Appends formatted log records to a deque for UI display."""

    def __init__(self, log_history):
        super().__init__()
        self.log_history = log_history

    def emit(self, record):
        message = self.format(record)
        self.log_history.append(message)


class SensorController:
    """
    Unified I2C sensor and level transmitter controller.

    Periodically reads via background thread:
    - SHT3x: temperature (°C) + humidity (%)
    - INA4230: per-channel current (A) across multiple chips
    - Level transmitter: liquid level via Modbus RTU
    """

    # ── SHT3x constants ───────────────────────────────────────────
    SHT3X_MEAS_CMD = bytes([0x24, 0x00])
    SHT3X_MEAS_WAIT = 0.020  # 20 ms
    SHT3X_TEMP_OFFSET = -45.0
    SHT3X_TEMP_SCALE = 175.0
    SHT3X_HUM_SCALE = 100.0
    SHT3X_RAW_MAX = 65535.0

    # ── INA4230 constants ─────────────────────────────────────────
    INA4230_REG_CONFIG = 0x20
    INA4230_CONFIG_INIT = 0xF427
    INA4230_REG_CURRENT = [0x02, 0x0A, 0x12, 0x1A]  # 4 channels
    INA4230_REG_CAL = [0x05, 0x0D, 0x15, 0x1D]
    INA4230_CURRENT_LSB = 0.001   # 1 mA per LSB
    INA4230_SHUNT_RESISTANCE = 0.01  # 10 mΩ

    # ── Modbus RTU constants ──────────────────────────────────────
    MODBUS_FUNC_READ = 0x03
    MODBUS_RESP_TIMEOUT = 0.05  # 50 ms

    def __init__(
        self,
        i2c_device: str = "/dev/i2c-3",
        sht3x_addr: int = 0x45,
        ina4230_addrs: Optional[list] = None,
        current_channels: Optional[list] = None,
        level_port: str = "/dev/ttyAMA2",
        level_baud: int = 9600,
        level_slave_id: int = 1,
        poll_interval: float = 2.0,
    ):
        if getattr(self, "initialized", False):
            return

        # Store parameters
        self.i2c_device = i2c_device
        self.sht3x_addr = sht3x_addr
        self.ina4230_addrs = ina4230_addrs or [0x40, 0x41, 0x44]
        self.current_channels = current_channels or []
        self.level_port = level_port
        self.level_baud = level_baud
        self.level_slave_id = level_slave_id
        self.poll_interval = poll_interval

        # Logging
        self.log_history = deque(maxlen=LOG_MAX_LEN)
        self.logger = self._setup_logger()

        # ── Sensor data (updated by _read_loop, read by get_status()) ──
        self.lock = threading.Lock()

        # SHT3x
        self.temperature: Optional[float] = None
        self.humidity: Optional[float] = None
        self.sht3x_ok = False

        # INA4230
        self.currents: dict = {}  # key: "0x41_ch3", value: float (A)
        self.ina4230_ok = False

        # Level transmitter
        self.level_value: Optional[float] = None
        self.level_unit = ""
        self.level_ok = False

        # Runtime state
        self.running = True
        self.i2c_fd: int = -1

        # ── Initialize hardware ──
        self._init_i2c()
        self._init_sht3x()
        self._init_ina4230()
        self._init_level()

        # ── Start background read thread ──
        threading.Thread(target=self._read_loop, daemon=True).start()
        self.initialized = True

    # ══════════════════════════════════════════════════════════════
    # Initialization
    # ══════════════════════════════════════════════════════════════

    def _setup_logger(self) -> logging.Logger:
        logger = logging.getLogger("sensor_controller")
        logger.setLevel(logging.INFO)
        logger.propagate = False
        if not logger.handlers:
            formatter = logging.Formatter(LOG_FORMAT)
            handler = ListLogHandler(self.log_history)
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        return logger

    def _init_i2c(self) -> None:
        """Open the I2C bus device."""
        try:
            self.i2c_fd = os.open(self.i2c_device, os.O_RDWR)
            self.logger.info(f"I2C bus opened: {self.i2c_device}")
        except Exception as e:
            self.logger.error(f"Failed to open I2C bus {self.i2c_device}: {e}")
            self.i2c_fd = -1

    def _i2c_select(self, addr: int) -> bool:
        """Set the I2C slave device address."""
        if self.i2c_fd < 0:
            return False
        try:
            import fcntl
            fcntl.ioctl(self.i2c_fd, I2C_SLAVE, addr)
            return True
        except Exception as e:
            self.logger.error(f"ioctl I2C_SLAVE 0x{addr:02X} failed: {e}")
            return False

    def _init_sht3x(self) -> None:
        """Probe the SHT3x temperature/humidity sensor."""
        if self.i2c_fd < 0:
            return
        if not self._i2c_select(self.sht3x_addr):
            return
        # Probe via zero-length write
        try:
            os.write(self.i2c_fd, b"")
            self.logger.info(f"SHT3x detected at 0x{self.sht3x_addr:02X}")
        except Exception as e:
            self.logger.warning(f"SHT3x not found at 0x{self.sht3x_addr:02X}: {e}")

    def _init_ina4230(self) -> None:
        """Probe and initialize all INA4230 chips."""
        if self.i2c_fd < 0:
            return

        detected = []
        for addr in self.ina4230_addrs:
            if self._probe_ina4230(addr):
                detected.append(addr)
                self._config_ina4230(addr)

        self.ina4230_addrs = detected
        if detected:
            self.logger.info(f"INA4230 devices detected: {[f'0x{a:02X}' for a in detected]}")
        else:
            self.logger.warning("No INA4230 devices found")

    def _probe_ina4230(self, addr: int) -> bool:
        """Probe INA4230 by reading the CONFIG register."""
        if not self._i2c_select(addr):
            return False
        try:
            os.write(self.i2c_fd, bytes([self.INA4230_REG_CONFIG]))
            time.sleep(0.002)
            os.read(self.i2c_fd, 2)
            self.logger.info(f"INA4230 detected at 0x{addr:02X}")
            return True
        except Exception as e:
            self.logger.warning(f"No INA4230 at 0x{addr:02X}: {e}")
            return False

    def _config_ina4230(self, addr: int) -> None:
        """Configure one INA4230 chip: write CONFIG + CAL registers."""
        # Write CONFIG
        if not self._write_reg16(addr, self.INA4230_REG_CONFIG, self.INA4230_CONFIG_INIT):
            self.logger.error(f"Failed to configure INA4230 at 0x{addr:02X}")
            return

        # Calculate CAL value
        cal = int(0.00512 / (self.INA4230_CURRENT_LSB * self.INA4230_SHUNT_RESISTANCE) + 0.5)
        self.logger.info(
            f"INA4230 0x{addr:02X}: R_shunt={self.INA4230_SHUNT_RESISTANCE}Ω, "
            f"LSB={self.INA4230_CURRENT_LSB}A, CAL={cal}"
        )

        for reg in self.INA4230_REG_CAL:
            if not self._write_reg16(addr, reg, cal):
                self.logger.error(f"Failed to write CAL for INA4230 0x{addr:02X}")
                return

    def _init_level(self) -> None:
        """Initialize the level transmitter serial port (Modbus RTU / RS-485)."""
        try:
            import serial

            self.level_ser = serial.Serial(
                port=self.level_port,
                baudrate=self.level_baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
            )
            # Attempt to enable RS-485 mode
            try:
                self.level_ser.rs485_mode = serial.rs485.RS485Settings(
                    rts_level_for_tx=True,
                    rts_level_for_rx=False,
                )
            except Exception:
                pass  # Not supported on all drivers; ignore

            self.logger.info(f"Level transmitter serial opened: {self.level_port} @ {self.level_baud}")
            # Read sensor metadata (unit code)
            self._read_level_metadata()
        except ImportError:
            self.logger.warning("pyserial not available, level transmitter disabled")
            self.level_ser = None
        except Exception as e:
            self.logger.error(f"Failed to open level transmitter port {self.level_port}: {e}")
            self.level_ser = None

    def _read_level_metadata(self) -> None:
        """Read level transmitter metadata (unit code, etc.) via Modbus."""
        if self.level_ser is None:
            return

        try:
            regs = self._modbus_read_registers(self.level_slave_id, 0x0002, 1)
            if regs:
                unit_code = regs[0]
                self.level_unit = self._unit_code_to_string(unit_code)
                self.logger.info(f"Level sensor unit: {unit_code} ({self.level_unit})")
        except Exception as e:
            self.logger.warning(f"Failed to read level sensor metadata: {e}")
            self.level_unit = "unknown"

    # ══════════════════════════════════════════════════════════════
    # Background read loop
    # ══════════════════════════════════════════════════════════════

    def _read_loop(self) -> None:
        """Background thread: periodically reads all sensor data."""
        while self.running:
            if self.i2c_fd >= 0:
                self._read_sht3x()
                self._read_ina4230()

            if self.level_ser is not None:
                self._read_level()

            time.sleep(self.poll_interval)

    # ══════════════════════════════════════════════════════════════
    # SHT3x read
    # ══════════════════════════════════════════════════════════════

    @staticmethod
    def _sht3x_crc8(data: bytes) -> int:
        """CRC-8/MAXIM: polynomial x^8 + x^5 + x^4 + 1 = 0x31."""
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x31) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def _read_sht3x(self) -> None:
        """Read SHT3x temperature and humidity."""
        if not self._i2c_select(self.sht3x_addr):
            with self.lock:
                self.sht3x_ok = False
            return

        try:
            # Send measurement command
            if os.write(self.i2c_fd, self.SHT3X_MEAS_CMD) != 2:
                self.logger.warning("SHT3x: write command failed")
                with self.lock:
                    self.sht3x_ok = False
                return

            time.sleep(self.SHT3X_MEAS_WAIT)

            # Read 6 bytes
            buf = os.read(self.i2c_fd, 6)
            if len(buf) < 6:
                self.logger.warning(f"SHT3x: short read ({len(buf)} bytes)")
                with self.lock:
                    self.sht3x_ok = False
                return

            # CRC check
            temp_crc_ok = self._sht3x_crc8(buf[0:2]) == buf[2]
            hum_crc_ok = self._sht3x_crc8(buf[3:5]) == buf[5]

            if not temp_crc_ok or not hum_crc_ok:
                self.logger.error(
                    f"SHT3x CRC failed (Temp: {'OK' if temp_crc_ok else 'BAD'}, "
                    f"Hum: {'OK' if hum_crc_ok else 'BAD'})"
                )
                with self.lock:
                    self.sht3x_ok = False
                return

            raw_temp = (buf[0] << 8) | buf[1]
            raw_hum = (buf[3] << 8) | buf[4]

            temperature = (
                self.SHT3X_TEMP_OFFSET
                + self.SHT3X_TEMP_SCALE * (raw_temp / self.SHT3X_RAW_MAX)
            )
            humidity = self.SHT3X_HUM_SCALE * (raw_hum / self.SHT3X_RAW_MAX)

            with self.lock:
                self.temperature = round(temperature, 2)
                self.humidity = round(humidity, 2)
                self.sht3x_ok = True

        except Exception as e:
            self.logger.error(f"SHT3x read error: {e}")
            with self.lock:
                self.sht3x_ok = False

    # ══════════════════════════════════════════════════════════════
    # INA4230 read
    # ══════════════════════════════════════════════════════════════

    def _read_ina4230(self) -> None:
        """Read current data from all INA4230 chips."""
        all_ok = True
        new_currents = {}

        for addr in self.ina4230_addrs:
            for ch_idx in range(4):  # 4 channels
                reg = self.INA4230_REG_CURRENT[ch_idx]
                raw = self._read_reg16_signed(addr, reg)
                if raw is not None:
                    current_a = raw * self.INA4230_CURRENT_LSB
                    key = f"0x{addr:02X}_ch{ch_idx + 1}"
                    new_currents[key] = round(current_a, 3)
                else:
                    all_ok = False

        with self.lock:
            self.currents = new_currents
            self.ina4230_ok = all_ok

    # ══════════════════════════════════════════════════════════════
    # Level transmitter read (Modbus RTU)
    # ══════════════════════════════════════════════════════════════

    def _read_level(self) -> None:
        """Read level transmitter float value via Modbus RTU."""
        if self.level_ser is None:
            return

        try:
            # Read 2 registers (0x0016, IEEE 754 float big-endian)
            regs = self._modbus_read_registers(self.level_slave_id, 0x0016, 2)
            if regs and len(regs) >= 2:
                # 4 bytes big-endian → float
                a = (regs[0] >> 8) & 0xFF
                b = regs[0] & 0xFF
                c = (regs[1] >> 8) & 0xFF
                d = regs[1] & 0xFF
                be_bytes = bytes([a, b, c, d])
                value = struct.unpack(">f", be_bytes)[0]

                # Reject non-finite or implausible values
                import math
                if not math.isfinite(value) or abs(value) > 1e6:
                    self.logger.warning(f"Level sensor returned invalid value: {value}")
                    with self.lock:
                        self.level_ok = False
                    return

                # Clamp near-zero noise to true zero
                if abs(value) < 1e-9:
                    value = 0.0

                value = round(value, 2)
                with self.lock:
                    self.level_value = value
                    self.level_ok = True
            else:
                with self.lock:
                    self.level_ok = False
        except Exception as e:
            self.logger.error(f"Level read error: {e}")
            with self.lock:
                self.level_ok = False

    # ══════════════════════════════════════════════════════════════
    # Modbus RTU utilities
    # ══════════════════════════════════════════════════════════════

    @staticmethod
    def _modbus_crc16(data: bytes) -> int:
        """Modbus CRC-16 (polynomial 0xA001)."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _modbus_read_registers(
        self, slave_id: int, start_addr: int, num_regs: int
    ) -> list:
        """Send a Modbus RTU Read Holding Registers request; return register values."""
        if self.level_ser is None:
            return []

        # Build request frame
        req = bytearray()
        req.append(slave_id)
        req.append(self.MODBUS_FUNC_READ)
        req.append((start_addr >> 8) & 0xFF)
        req.append(start_addr & 0xFF)
        req.append((num_regs >> 8) & 0xFF)
        req.append(num_regs & 0xFF)
        crc = self._modbus_crc16(bytes(req))
        req.append(crc & 0xFF)
        req.append((crc >> 8) & 0xFF)

        try:
            self.level_ser.reset_input_buffer()
            self.level_ser.write(bytes(req))

            # Wait for response
            time.sleep(self.MODBUS_RESP_TIMEOUT)

            # Response: header (3 bytes) + data (num_regs*2) + CRC (2)
            expected_len = 3 + num_regs * 2 + 2
            response = self.level_ser.read(expected_len)

            if len(response) < 5:
                self.logger.warning(
                    f"Modbus short response: {len(response)} bytes (expected {expected_len})"
                )
                return []

            # Validate
            if response[0] != slave_id:
                return []

            if response[1] & 0x80:
                self.logger.warning(
                    f"Modbus exception: func=0x{response[1] & 0x7F:02X}, code={response[2] if len(response) > 2 else '?'}"
                )
                return []

            if response[1] != self.MODBUS_FUNC_READ:
                return []

            byte_count = response[2]
            if byte_count != num_regs * 2:
                return []

            # CRC check
            crc_received = response[3 + byte_count] | (response[4 + byte_count] << 8)
            crc_calc = self._modbus_crc16(response[:3 + byte_count])
            if crc_received != crc_calc:
                self.logger.warning(
                    f"Modbus CRC error: received 0x{crc_received:04X}, calculated 0x{crc_calc:04X}"
                )
                return []

            # Extract register values
            regs = []
            for i in range(num_regs):
                offset = 3 + i * 2
                val = (response[offset] << 8) | response[offset + 1]
                regs.append(val)
            return regs

        except Exception as e:
            self.logger.error(f"Modbus read error: {e}")
            return []

    @staticmethod
    def _unit_code_to_string(code: int) -> str:
        """Level transmitter unit code → human-readable string."""
        mapping = {
            0: "MPa", 1: "KPa", 2: "Pa", 3: "bar", 4: "mbar",
            5: "Kgcm2", 6: "PSI", 7: "mH2O", 8: "mmH2O",
            9: "inH2O", 10: "H2O", 11: "mHg", 12: "mmHg",
            13: "inHg", 14: "atm", 15: "Torr", 16: "m",
            17: "cm", 18: "mm", 19: "Kg", 20: "°C",
            21: "PH", 22: "°F", 23: "(none)",
        }
        return mapping.get(code, "?")

    # ══════════════════════════════════════════════════════════════
    # I2C register read/write
    # ══════════════════════════════════════════════════════════════

    def _write_reg16(self, dev_addr: int, reg_addr: int, value: int) -> bool:
        """Write a 16-bit register to an I2C device."""
        if not self._i2c_select(dev_addr):
            return False
        try:
            buf = bytes([reg_addr, (value >> 8) & 0xFF, value & 0xFF])
            if os.write(self.i2c_fd, buf) != 3:
                self.logger.error(
                    f"write_reg16 failed: 0x{dev_addr:02X} reg=0x{reg_addr:02X}"
                )
                return False
            return True
        except Exception as e:
            self.logger.error(
                f"write_reg16 error 0x{dev_addr:02X} reg=0x{reg_addr:02X}: {e}"
            )
            return False

    def _read_reg16(self, dev_addr: int, reg_addr: int) -> Optional[int]:
        """Read a 16-bit register from an I2C device (unsigned)."""
        if not self._i2c_select(dev_addr):
            return None
        try:
            os.write(self.i2c_fd, bytes([reg_addr]))
            time.sleep(0.002)
            buf = os.read(self.i2c_fd, 2)
            if len(buf) < 2:
                return None
            return (buf[0] << 8) | buf[1]
        except Exception as e:
            self.logger.warning(
                f"read_reg16 error 0x{dev_addr:02X} reg=0x{reg_addr:02X}: {e}"
            )
            return None

    def _read_reg16_signed(self, dev_addr: int, reg_addr: int) -> Optional[int]:
        """Read a 16-bit register from an I2C device (signed)."""
        raw = self._read_reg16(dev_addr, reg_addr)
        if raw is None:
            return None
        if raw & 0x8000:
            return raw - 0x10000
        return raw

    # ══════════════════════════════════════════════════════════════
    # Public interface (consistent with BMSController / MavController)
    # ══════════════════════════════════════════════════════════════

    def get_status(self) -> dict:
        """Thread-safe return all sensor data."""
        with self.lock:
            return {
                # SHT3x
                "sht3x_ok": self.sht3x_ok,
                "temperature": self.temperature,
                "humidity": self.humidity,
                # INA4230
                "ina4230_ok": self.ina4230_ok,
                "currents": dict(self.currents),
                # Level transmitter
                "level_ok": self.level_ok,
                "level_value": self.level_value,
                "level_unit": self.level_unit,
            }

    def get_logs(self) -> list:
        return list(self.log_history)

    def clear_logs(self) -> None:
        self.log_history.clear()
        self.logger.info("Sensor log history cleared")

    def shutdown(self) -> None:
        self.running = False
        if self.i2c_fd >= 0:
            try:
                os.close(self.i2c_fd)
                self.logger.info("I2C bus closed")
            except Exception as e:
                self.logger.error(f"Error closing I2C bus: {e}")
        if self.level_ser and self.level_ser.is_open:
            try:
                self.level_ser.close()
                self.logger.info("Level transmitter serial closed")
            except Exception as e:
                self.logger.error(f"Error closing level serial: {e}")

import struct
import logging
import threading
import time
from collections import deque

import serial

from config import BMS_PORT, BMS_BAUD, BMS_POLL_INTERVAL

LOG_MAX_LEN = 50
LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"


class ListLogHandler(logging.Handler):
    def __init__(self, log_history):
        super().__init__()
        self.log_history = log_history

    def emit(self, record):
        message = self.format(record)
        self.log_history.append(message)


class BMSController:
    """Battery Management System reader via Modbus RTU over serial."""

    def __init__(self, port: str = BMS_PORT, baudrate: int = BMS_BAUD):
        if getattr(self, "initialized", False):
            return

        self.port = port
        self.baudrate = baudrate
        self.slave_id = 0x01
        self.func_code = 0x03

        self.log_history = deque(maxlen=LOG_MAX_LEN)
        self.logger = self._setup_logger()

        self.voltage = 0.0
        self.soc = 0.0
        self.charging_current = 0.0
        self.discharging_current = 0.0
        self.capacity = 0.0
        self.is_connected = False

        self.temp_sensor_count = 0
        self.temp_sensors = []

        self.lock = threading.Lock()
        self.running = True

        self._connect()
        threading.Thread(target=self._read_loop, daemon=True).start()
        self.initialized = True

    def _setup_logger(self) -> logging.Logger:
        logger = logging.getLogger("bms_controller")
        logger.setLevel(logging.INFO)
        logger.propagate = False

        if not logger.handlers:
            formatter = logging.Formatter(LOG_FORMAT)
            handler = ListLogHandler(self.log_history)
            handler.setFormatter(formatter)
            logger.addHandler(handler)

        return logger

    def _connect(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.logger.info(f"BMS connected: {self.port} @ {self.baudrate}")
        except Exception as e:
            self.logger.error(f"Failed to connect BMS on {self.port}: {e}")
            self.ser = None

    def calculate_crc(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack("<H", crc)

    def read(self) -> bool:
        if not self.ser or not self.ser.is_open:
            return False

        try:
            start_addr = 0x0030
            reg_count = 0x0006
            request = bytearray([self.slave_id, self.func_code])
            request.extend(struct.pack(">H", start_addr))
            request.extend(struct.pack(">H", reg_count))
            request.extend(self.calculate_crc(request))

            self.ser.reset_input_buffer()
            self.ser.write(request)
            response = self.ser.read(17)

            if len(response) < 17:
                self.logger.warning(
                    f"Incomplete BMS data: received {len(response)} bytes, expected 17"
                )
                return False

            with self.lock:
                self.charging_current = struct.unpack(">H", response[4:6])[0] * 0.1
                self.discharging_current = struct.unpack(">H", response[6:8])[0] * 0.1
                self.voltage = struct.unpack(">H", response[8:10])[0] * 0.01
                self.soc = struct.unpack(">H", response[10:12])[0]
                self.capacity = struct.unpack(">I", response[12:16])[0] / 1000.0
                self.is_connected = True

            return True
        except Exception as e:
            self.logger.error(f"BMS read error: {e}")
            with self.lock:
                self.is_connected = False
            return False

    def read_temperature(self) -> bool:
        """Read temperature sensor count (0x0025) and values (0x0026~0x002E)."""
        if not self.ser or not self.ser.is_open:
            return False

        try:
            # Step 1: Read sensor count from 0x0025 (1 register)
            start_addr = 0x0025
            reg_count = 0x0001
            request = bytearray([self.slave_id, self.func_code])
            request.extend(struct.pack(">H", start_addr))
            request.extend(struct.pack(">H", reg_count))
            request.extend(self.calculate_crc(request))

            self.ser.reset_input_buffer()
            self.ser.write(request)
            response = self.ser.read(7)

            if len(response) < 7:
                self.logger.warning(
                    f"Incomplete temperature count response: {len(response)} bytes"
                )
                return False

            sensor_count = struct.unpack(">H", response[4:6])[0]
            if sensor_count > 9:
                sensor_count = 9

            if sensor_count == 0:
                with self.lock:
                    self.temp_sensor_count = 0
                    self.temp_sensors = []
                return True

            # Step 2: Read temperature values from 0x0026 (each 2 bytes, unit 0.1K)
            start_addr = 0x0026
            reg_count = sensor_count
            request = bytearray([self.slave_id, self.func_code])
            request.extend(struct.pack(">H", start_addr))
            request.extend(struct.pack(">H", reg_count))
            request.extend(self.calculate_crc(request))

            self.ser.reset_input_buffer()
            self.ser.write(request)
            expected_len = 6 + reg_count * 2
            response = self.ser.read(expected_len)

            if len(response) < expected_len:
                self.logger.warning(
                    f"Incomplete temperature data: {len(response)} bytes, expected {expected_len}"
                )
                return False

            temp_sensors = []
            for i in range(sensor_count):
                offset = 4 + i * 2
                raw = struct.unpack(">H", response[offset:offset + 2])[0]
                kelvin = raw * 0.1
                celsius = kelvin - 273.15
                temp_sensors.append({
                    "index": i + 1,
                    "kelvin": round(kelvin, 1),
                    "celsius": round(celsius, 1),
                })

            with self.lock:
                self.temp_sensor_count = sensor_count
                self.temp_sensors = temp_sensors

            return True
        except Exception as e:
            self.logger.error(f"BMS temperature read error: {e}")
            return False

    def _read_loop(self) -> None:
        while self.running:
            self.read()
            self.read_temperature()
            time.sleep(BMS_POLL_INTERVAL)

    def get_status(self) -> dict:
        with self.lock:
            return {
                "bms_connected": self.is_connected,
                "voltage": self.voltage,
                "soc": self.soc,
                "charging_current": self.charging_current,
                "discharging_current": self.discharging_current,
                "capacity": self.capacity,
                "temp_sensor_count": self.temp_sensor_count,
                "temp_sensors": list(self.temp_sensors),
            }

    def get_logs(self) -> list:
        return list(self.log_history)

    def clear_logs(self) -> None:
        self.log_history.clear()

    def shutdown(self) -> None:
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.logger.info("BMS serial port closed")
            except Exception as e:
                self.logger.error(f"Error closing BMS serial: {e}")

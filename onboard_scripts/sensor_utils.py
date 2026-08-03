import rclpy
from rclpy.node import Node
from functools import partial
import threading
import time

from std_msgs.msg import Float64
from sensor_msgs.msg import BatteryState

# INA4230 current/power monitor channels (publish Float64)
# INA4230_TOPIC = [
#     "/ina4230_0x40/channel_1/raw",
#     "/ina4230_0x40/channel_2/raw",
#     "/ina4230_0x40/channel_3/raw",
#     "/ina4230_0x40/channel_4/raw",
#     "/ina4230_0x41/channel_1/raw",
#     "/ina4230_0x41/channel_2/raw",
#     "/ina4230_0x41/channel_3/raw",
#     "/ina4230_0x41/channel_4/raw",
#     "/ina4230_0x44/channel_1/raw",
#     "/ina4230_0x44/channel_2/raw",
#     "/ina4230_0x44/channel_3/raw",
#     "/ina4230_0x44/channel_4/raw",
# ]
INA4230_TOPIC = [
    "/ina4230_0x40/channel_4/raw",
    "/ina4230_0x41/channel_3/raw",
    "/ina4230_0x40/channel_2/raw",
    "/ina4230_0x41/channel_1/raw",
]

# SHT3X temperature/humidity sensor
SHT3X_HUMIDITY_TOPIC = "/sht3x_node/humidity/raw"
SHT3X_TEMPERATURE_TOPIC = "/sht3x_node/temperature/raw"

# BMS485
BMS485_TOPIC = "/bms485_node/battery"

# Level Transmitter
LEVEL_TRANSMITTER_TOPIC = "/level_transmitter_node/level"

# Timeout settings (in seconds)
SENSOR_TIMEOUT_S = 2.0  # Consider data stale after 5 seconds

class SensorSubscriber(Node):

    def __init__(self, timeout_seconds=SENSOR_TIMEOUT_S):
        super().__init__('sensor_subscriber')

        # Timeout setting
        self.timeout_seconds = timeout_seconds

        # Thread lock for thread-safe access
        self._lock = threading.Lock()

        # Keep references so subscriptions aren't garbage collected
        self.subscriptions_list = []

        # INA4230 current/power monitor channels
        self.latest_ina4230_values = {}
        self.latest_ina4230_timestamps = {}

        # Initialize with default values
        for topic in INA4230_TOPIC:
            self.latest_ina4230_values[topic] = 0.0
            self.latest_ina4230_timestamps[topic] = None

        # INA4230 channels
        for topic in INA4230_TOPIC:
            sub = self.create_subscription(
                Float64,
                topic,
                partial(self.ina4230_callback, topic),
                10)
            self.subscriptions_list.append(sub)

        # SHT3X temperature/humidity sensor
        self.latest_humidity = 0.0
        self.latest_humidity_timestamp = None
        self.latest_temperature = 0.0
        self.latest_temperature_timestamp = None

        # SHT3X humidity
        self.subscriptions_list.append(
            self.create_subscription(
                Float64,
                SHT3X_HUMIDITY_TOPIC,
                self.humidity_callback,
                10))

        # SHT3X temperature
        self.subscriptions_list.append(
            self.create_subscription(
                Float64,
                SHT3X_TEMPERATURE_TOPIC,
                self.temperature_callback,
                10))

        # Battery
        self.latest_discharge_current = 0.0
        self.latest_discharge_current_timestamp = None
        self.latest_module_voltage = 0.0
        self.latest_module_voltage_timestamp = None
        self.latest_percentage = 0.0
        self.latest_percentage_timestamp = None

        # BMS485
        self.subscriptions_list.append(
            self.create_subscription(
                BatteryState,
                BMS485_TOPIC,
                self.bms485_callback,
                10))

        # Level Transmitter
        self.latest_water_level = 0.0
        self.latest_water_level_timestamp = None

        # Level Transmitter
        self.subscriptions_list.append(
            self.create_subscription(
                Float64,
                LEVEL_TRANSMITTER_TOPIC,
                self.level_callback,
                10))

    def _get_timestamp(self):
        """Get current timestamp in seconds (float)"""
        return time.time()

    def _is_data_stale(self, timestamp):
        """Check if data is stale based on timeout"""
        if timestamp is None:
            return True
        return (time.time() - timestamp) > self.timeout_seconds

    def ina4230_callback(self, topic, msg):
        with self._lock:
            self.latest_ina4230_values[topic] = msg.data
            self.latest_ina4230_timestamps[topic] = self._get_timestamp()
        # self.get_logger().info('[%s] value: %f' % (topic, msg.data))

    def humidity_callback(self, msg):
        with self._lock:
            self.latest_humidity = msg.data
            self.latest_humidity_timestamp = self._get_timestamp()
        # self.get_logger().info('Humidity: %f %%' % msg.data)

    def temperature_callback(self, msg):
        with self._lock:
            self.latest_temperature = msg.data
            self.latest_temperature_timestamp = self._get_timestamp()
        # self.get_logger().info('Temperature: %f °C' % msg.data)

    def bms485_callback(self, msg):
        with self._lock:
            self.latest_discharge_current = -msg.current
            self.latest_discharge_current_timestamp = self._get_timestamp()
            self.latest_module_voltage = msg.voltage
            self.latest_module_voltage_timestamp = self._get_timestamp()
            self.latest_percentage = msg.percentage
            self.latest_percentage_timestamp = self._get_timestamp()

    def level_callback(self, msg):
        with self._lock:
            self.latest_water_level = msg.data * 10.0
            self.latest_water_level_timestamp = self._get_timestamp()
        # self.get_logger().info('Water Level: %f' % msg.data)

    def get_snapshot(self):
        """Thread-safe method to get a snapshot of all sensor values with timeout checking"""
        with self._lock:
            # Check and reset stale INA4230 values
            ina4230_values = {}
            for topic in INA4230_TOPIC:
                if self._is_data_stale(self.latest_ina4230_timestamps.get(topic)):
                    ina4230_values[topic] = 0.0
                else:
                    ina4230_values[topic] = self.latest_ina4230_values.get(topic, 0.0)

            # Check and reset other sensor values
            humidity = 0.0 if self._is_data_stale(self.latest_humidity_timestamp) else self.latest_humidity
            temperature = 0.0 if self._is_data_stale(self.latest_temperature_timestamp) else self.latest_temperature
            discharge_current = 0.0 if self._is_data_stale(self.latest_discharge_current_timestamp) else self.latest_discharge_current
            module_voltage = 0.0 if self._is_data_stale(self.latest_module_voltage_timestamp) else self.latest_module_voltage
            percentage = 0.0 if self._is_data_stale(self.latest_percentage_timestamp) else self.latest_percentage
            water_level = 0.0 if self._is_data_stale(self.latest_water_level_timestamp) else self.latest_water_level

            return {
                "ina4230": ina4230_values,
                "humidity": humidity,
                "temperature": temperature,
                "discharge_current": discharge_current,
                "module_voltage": module_voltage,
                "percentage": percentage,
                "water_level": water_level,
            }

    def get_snapshot_with_status(self):
        """Get snapshot with additional status information about data freshness"""
        with self._lock:
            ina4230_values = {}
            ina4230_stale = {}
            for topic in INA4230_TOPIC:
                is_stale = self._is_data_stale(self.latest_ina4230_timestamps.get(topic))
                ina4230_stale[topic] = is_stale
                ina4230_values[topic] = 0.0 if is_stale else self.latest_ina4230_values.get(topic, 0.0)

            humidity_stale = self._is_data_stale(self.latest_humidity_timestamp)
            temperature_stale = self._is_data_stale(self.latest_temperature_timestamp)
            discharge_current_stale = self._is_data_stale(self.latest_discharge_current_timestamp)
            module_voltage_stale = self._is_data_stale(self.latest_module_voltage_timestamp)
            percentage_stale = self._is_data_stale(self.latest_percentage_timestamp)
            water_level_stale = self._is_data_stale(self.latest_water_level_timestamp)

            return {
                "values": {
                    "ina4230": ina4230_values,
                    "humidity": 0.0 if humidity_stale else self.latest_humidity,
                    "temperature": 0.0 if temperature_stale else self.latest_temperature,
                    "discharge_current": 0.0 if discharge_current_stale else self.latest_discharge_current,
                    "module_voltage": 0.0 if module_voltage_stale else self.latest_module_voltage,
                    "percentage": 0.0 if percentage_stale else self.latest_percentage,
                    "water_level": 0.0 if water_level_stale else self.latest_water_level,
                },
                "stale": {
                    "ina4230": ina4230_stale,
                    "humidity": humidity_stale,
                    "temperature": temperature_stale,
                    "discharge_current": discharge_current_stale,
                    "module_voltage": module_voltage_stale,
                    "percentage": percentage_stale,
                    "water_level": water_level_stale,
                }
            }

def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()
    rclpy.spin(sensor_subscriber)
    sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
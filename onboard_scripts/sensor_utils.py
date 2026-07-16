import rclpy
from rclpy.node import Node
from functools import partial

from std_msgs.msg import Float64
from sensor_msgs.msg import BatteryState

# INA4230 current/power monitor channels (publish Float64)
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

class SensorSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_subscriber')

        # Keep references so subscriptions aren't garbage collected
        self.subscriptions_list = []

        # Store the latest values for access from outside the callbacks
        self.latest_ina4230_values = {}
        self.latest_humidity = None
        self.latest_temperature = None

        self.latest_discharge_current = None
        self.latest_module_voltage = None
        self.latest_percentage = None

        # INA4230 channels
        for topic in INA4230_TOPIC:
            sub = self.create_subscription(
                Float64,
                topic,
                partial(self.ina4230_callback, topic),
                10)
            self.subscriptions_list.append(sub)

        # SHT3X humidity
        self.subscriptions_list.append(
            self.create_subscription(
                # RelativeHumidity,
                Float64,
                SHT3X_HUMIDITY_TOPIC,
                self.humidity_callback,
                10))

        # SHT3X temperature
        self.subscriptions_list.append(
            self.create_subscription(
                # Temperature,
                Float64,
                SHT3X_TEMPERATURE_TOPIC,
                self.temperature_callback,
                10))
        
        # BMS485
        self.subscriptions_list.append(
            self.create_subscription(
                BatteryState,
                BMS485_TOPIC,
                self.bms485_callback,
                10))

    def ina4230_callback(self, topic, msg):
        self.latest_ina4230_values[topic] = msg.data
        # self.get_logger().info('[%s] value: %f' % (topic, msg.data))

    def humidity_callback(self, msg):
        self.latest_humidity = msg.data
        # self.get_logger().info('Humidity: %f %%' % msg.data)

    def temperature_callback(self, msg):
        self.latest_temperature = msg.data
        # self.get_logger().info('Temperature: %f °C' % msg.data)

    def bms485_callback(self, msg):
        self.latest_discharge_current = -msg.current
        self.latest_module_voltage = msg.voltage
        self.latest_percentage = msg.percentage

def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()
    rclpy.spin(sensor_subscriber)
    sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
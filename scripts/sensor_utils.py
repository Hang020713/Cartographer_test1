import rclpy
from rclpy.node import Node
from functools import partial

from std_msgs.msg import Float64
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import Temperature

# INA4230 current/power monitor channels (publish Float64)
INA4230_TOPIC = [
    "/ina4230_0x44/channel_1",
    "/ina4230_0x44/channel_2",
    "/ina4230_0x44/channel_3",
    "/ina4230_0x44/channel_4",
]

# SHT3X temperature/humidity sensor
SHT3X_HUMIDITY_TOPIC = "/sht3x_node/humidity"
SHT3X_TEMPERATURE_TOPIC = "/sht3x_node/temperature"

class SensorSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_subscriber')

        # Keep references so subscriptions aren't garbage collected
        self.subscriptions_list = []

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
                RelativeHumidity,
                SHT3X_HUMIDITY_TOPIC,
                self.humidity_callback,
                10))

        # SHT3X temperature
        self.subscriptions_list.append(
            self.create_subscription(
                Temperature,
                SHT3X_TEMPERATURE_TOPIC,
                self.temperature_callback,
                10))

    def ina4230_callback(self, topic, msg):
        self.get_logger().info('[%s] value: %f' % (topic, msg.data))

    def humidity_callback(self, msg):
        self.get_logger().info('Humidity: %f %%' % msg.relative_humidity)

    def temperature_callback(self, msg):
        self.get_logger().info('Temperature: %f °C' % msg.temperature)


def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()
    rclpy.spin(sensor_subscriber)
    sensor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
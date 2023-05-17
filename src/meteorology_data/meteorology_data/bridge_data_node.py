#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Bridge_data(Node):

    def __init__(self):
        super().__init__("bridge_data_node")

        self.power_subscriber = self.create_subscription(String,"power",self.power_callback,10)
        self.telemetry_subscriber = self.create_subscription(String,"telemetry",self.telemetry_callback,10)

        self.get_logger().info("bridge node started")

    def power_callback(self, msg: String):
        pass
    #self.get_logger().info(msg.data)
    def telemetry_callback(self, msg: String):
        pass
    #self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Bridge_data()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
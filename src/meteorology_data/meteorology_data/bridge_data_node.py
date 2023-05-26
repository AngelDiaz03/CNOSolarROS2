#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class Bridge_data(Node):

    def __init__(self):
        super().__init__("bridge_data_node")

        self.power_dc_subscriber = self.create_subscription(Float64,"power_ac",self.power_ac_callback,10)
        self.power_ac_subscriber = self.create_subscription(Float64,"power_dc",self.power_dc_callback,10)
        self.power_dc_data = []
        self.power_ac_data = []
        self.get_logger().info("bridge node started")
        self.fig, self.ax = plt.subplots()

    def power_ac_callback(self, msg: Float64):
        self.power_ac_data.append(msg.data)
        self.get_logger().info('ac ' + str(msg.data))
        self.plot_values()

    def power_dc_callback(self, msg: Float64):
        self.power_dc_data.append(msg.data)
        self.get_logger().info('dc ' + str(msg.data))
        self.plot_values()
    def plot_values(self):
            self.ax.clear()
            self.ax.plot(self.power_ac_data, label = 'power AC')
            self.ax.plot(self.power_dc_data, label = 'power DC')
            self.ax.legend()
            self.ax.set_xlabel('data')
            self.ax.set_ylabel('power')
            #plt.ylabel('Value 2')
            #plt.title('ROS 2 Topic Values')
            #plt.show()
            self.fig.canvas.draw()
            plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = Bridge_data()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
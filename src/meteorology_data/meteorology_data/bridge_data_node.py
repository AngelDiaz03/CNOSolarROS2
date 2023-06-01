#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import pandas as pd
import matplotlib.pyplot as plt
from interfaces.msg import Timestamp

class Bridge_data(Node):

    def __init__(self):
        super().__init__("bridge_data_node")

        self.power_dc_subscriber = self.create_subscription(Float64,"power_ac",self.power_ac_callback,10)
        self.power_ac_subscriber = self.create_subscription(Float64,"power_dc",self.power_dc_callback,10)
        self.time_subscriber = self.create_subscription(Timestamp,"time_msg",self.time_callback,10)

        self.power_dc_data = []
        self.power_ac_data = []
        self.time_data = []
        self.get_logger().info("bridge node started")
        self.fig, self.ax = plt.subplots()

    def power_ac_callback(self, msg: Float64):
        self.power_ac_data.append(msg.data)
        self.plot_values()

    def power_dc_callback(self, msg: Float64):
        self.power_dc_data.append(msg.data)
        self.plot_values()
    def time_callback(self, msg: Timestamp):
        self.timestamp = pd.to_datetime(msg.unix_time_ns, unit='ns')
        self.timestamp = self.timestamp.tz_localize(msg.time_zone)
        self.time_data.append(self.timestamp)

    def plot_values(self):
        self.ax.clear()
        self.ax.plot(self.time_data[0:len(self.power_ac_data)], self.power_ac_data, label = 'power AC')
        self.ax.plot(self.time_data[0:len(self.power_dc_data)], self.power_dc_data, label = 'power DC')
        self.ax.legend()
        self.ax.set_xlabel('Date time')
        self.ax.set_ylabel('Power [W]')
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
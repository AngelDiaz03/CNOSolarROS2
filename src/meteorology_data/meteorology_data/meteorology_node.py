#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float64
from interfaces.msg import Timestamp
import cnsolar as cno 
import datetime
class Meteorology(Node):

    def __init__(self):
        super().__init__("meteorology_node")
        self.GHI_msg = self.create_publisher(Float64,"GHI_msg",10)
        self.POA_msg = self.create_publisher(Float64,"POA_msg",10)
        self.tamb_msg = self.create_publisher(Float64,"tamb_msg",10)
        self.tmod_msg = self.create_publisher(Float64,"tmod_msg",10)
        self.time_msg = self.create_publisher(Timestamp,"time_msg",10)

        self.data_file = cno.data.load_csv("src/meteorology_data/resource/data_effective.csv",None)

        self.get_logger().info("Meteorology node started")

        self.create_timer(0.09, self.get_data)
        self.count = 0

    def get_data(self): # take meteorology data from the repository
        self.actual_info = self.data_file.iloc[[self.count]]
        self.send_GHI_data(Float64(data = self.actual_info["GHI"].values[0]))
        self.send_POA_data(Float64(data = self.actual_info["Effective_Irradiance"].values[0]))
        self.send_tamb_data(Float64(data = self.actual_info["Tamb"].values[0]))
        self.send_tmod_data(Float64(data = self.actual_info["Tmod"].values[0]))
        
        data_time = Timestamp()
        data_time.unix_time_ns = int(self.actual_info.index.astype('int64')[0])
        data_time.time_zone = str(self.actual_info.index.tz)

        self.send_time_stamp_data(data_time)
        
        self.count += 1

        if self.count >= self.data_file.size:
            self.count = 0

    def send_GHI_data(self,data:Float64 = 0.0): 
        self.GHI_msg.publish(data)

    def send_POA_data(self,data:Float64 = 0.0): 
        self.POA_msg.publish(data)

    def send_tamb_data(self,data:Float64 = 0.0):  
        self.tamb_msg.publish(data)

    def send_tmod_data(self,data:Float64 = 0.0): 
        self.tmod_msg.publish(data)

    def send_time_stamp_data(self,data:Timestamp): 
        self.time_msg.publish(data)
        

def main(args=None):
    rclpy.init(args=args)
    node = Meteorology()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
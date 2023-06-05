#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from interfaces.msg import Timestamp
import cnsolar as cno
import pandas as pd
import json

class Solar_park(Node):

    def __init__(self):
        super().__init__("solar_park_node")
        # Subscribers
        self.GHI_subscriber = self.create_subscription(Float64,
                                                       "GHI_msg",
                                                       self.GHI_callback,
                                                       10)
        self.POA_subscriber = self.create_subscription(Float64,
                                                       "POA_msg",
                                                       self.POA_callback,
                                                       10)
        self.tamb_subscriber = self.create_subscription(Float64,
                                                        "tamb_msg",
                                                        self.tamb_callback,
                                                        10)
        self.tmod_subscriber = self.create_subscription(Float64,
                                                        "tmod_msg",
                                                        self.tmod_callback,
                                                        10)
        self.time_subscriber = self.create_subscription(Timestamp,
                                                        "time_msg",
                                                        self.time_callback,
                                                        10)
        # Publishers
        self.power_ac_msg = self.create_publisher(Float64,
                                                  "power_ac",
                                                  10)
        self.power_dc_msg = self.create_publisher(Float64,
                                                  "power_dc",
                                                  10)
        # Initialization
        self.get_logger().info("solar park node started")
        self.create_timer(0.1, self.test_callback)
        with open('src/cnsolar/resource/system_config_sd_51.json') as f:
            self.json_str = json.load(f)
        self.system_configuration = {}
        self.GHI_flag = False 
        self.POA_flag = False 
        self.tamb_flag = False 
        self.tmod_flag = False 
        self.time_flag = False 

    def test_callback(self):
        if self.GHI_flag and self.POA_flag and \
                             self.tamb_flag and \
                             self.tmod_flag and \
                             self.time_flag:
            
            self.system_configuration[0] = self.json_str
            data = {'GHI': [self.GHI],
                    'Tamb': [self.tamb],
                    'Effective_Irradiance': [self.POA],
                    'Tmod': [self.tmod]}
            index = pd.DatetimeIndex([self.timestamp], tz='UTC-05:00')
            df = pd.DataFrame(data, index=index)
            availability = None
            energy_units = 'Wh'
            self.output_data = cno.pipeline.run(self.system_configuration,
                                                df,
                                                availability,
                                                energy_units)
            self.get_logger().info(str(self.output_data['plant']))
            self.send_power_dc_data()
            self.send_power_ac_data()
            self.GHI_flag = False 
            self.POA_flag = False 
            self.tamb_flag = False 
            self.tmod_flag = False 
            self.time_flag = False 

    def GHI_callback(self, msg: Float64):
        self.GHI = msg.data
        self.GHI_flag = True

    def POA_callback(self, msg: Float64):
        self.POA = msg.data
        self.POA_flag = True
    
    def tamb_callback(self, msg: Float64):
       self.tamb = msg.data
       self.tamb_flag = True

    def tmod_callback(self, msg: Float64):
        self.tmod = msg.data
        self.tmod_flag = True

    def time_callback(self, msg: Timestamp):
        self.timestamp = pd.to_datetime(msg.unix_time_ns, unit='ns')
        self.timestamp = self.timestamp.tz_localize(msg.time_zone)
        self.time_flag = True

    def send_power_dc_data(self):
        # Publish power_dc data
        msg = Float64()
        msg.data = self.output_data['plant']['p_dc'].values[0]
        self.power_dc_msg.publish(msg)

    def send_power_ac_data(self):
        # Publish power_ac data
        msg = Float64()
        msg.data = self.output_data['plant']['ac'].values[0]
        self.power_ac_msg.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Solar_park()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
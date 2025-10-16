#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
class serial_transmitter(Node):
    def __init__(self):
        super().__init__("serial_transmitter")
        self.ardin=serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout=0.1)
        self.sub=self.create_subscription(String,"serial_transmitter",self.callBack, 10)

    def callBack(self,msg):
        self.ardin.write(msg.data.encode("utf-8"))

def main():
    rclpy.init()


    sim=serial_transmitter()
    rclpy.spin(sim)


    sim.destroy_node()
    rclpy.shutdown()
if __name__=="__main__":
    main()

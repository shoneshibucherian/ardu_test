#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialReciver(Node):
    def __init__(self):
        super().__init__("SimpleSerialReciver")
        self.pub_=self.create_publisher(String,"serial_reviver",10)
        self.frequecy_=0.001
        self.get_logger().info("publishing at %d Hz"%self.frequecy_)


        self.arduino_=serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout=0.1)
        
        self.timer_=self.create_timer(self.frequecy_,self.timerCallback)
    
    def timerCallback(self):
        if rclpy.ok() and self.arduino_.is_open:
            data=self.arduino_.readline()
            try:
                data.decode("utf-8")
            except:
                return
            msg= String()
            msg.data= str(data)
            self.pub_.publish(msg)
    

def main():
    rclpy.init()
    simple_publisher=SimpleSerialReciver()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown


if __name__=='__main__':
    main()
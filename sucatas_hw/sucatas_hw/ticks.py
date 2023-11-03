#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import serial
import sys
import time
#import math

#este código é exprimental
serialPort = serial.Serial('/dev/controlo', baudrate=115200)

class Controller:
    def __init__(self):
       self.linearVelocity = 0.0
       self.angularVelocity = 0.0
       self.gradient = 30.0
       rclpy.init(args=sys.argv)
       self.node = rclpy.create_node('drive_controller')
       self.cmd_vel_sub = self.node.create_subscription(Twist, '/diff_drive/cmd_vel', self.cmd_vel_callback, 1)
       self.timer_period = 0.05  # seconds
       self.rate = 50
       self.wheelradius = 0.045
       self.timer            = self.node.create_timer(self.timer_period, self.timer_callback)
       self.wheelSeparation  = 0.180 #self.node.declare_parameter('wheel_separation', 0.150).value
       assert isinstance(self.wheelSeparation, float), 'wheel_separation must be float'
       self.left_speed   = 0.0
       self.right_speed  = 0.0
       self.last_left_ticks   = 0
       self.last_right_ticks  = 0
       self.reset_encoders()
       self.pub_lmotor = self.node.create_publisher(Float32, 'lwheel', 10)
       self.pub_rmotor = self.node.create_publisher(Float32, 'rwheel', 10)
       
    def reset_encoders(self):
       a = bytes('@,'+str(196)+','+str(0)+','+str(0)+';', 'utf-8')
       serialPort.write(bytes(a))
       while serialPort.inWaiting():
             a = serialPort.read()
       serialPort.flush()
    
    def encoders(self, serialPort):
       a=bytes('@,'+str(197)+','+str(0)+','+str(0)+';', 'unicode_escape')
       serialPort.write(a)
       string = ""
       time.sleep(.0045)
       while serialPort.inWaiting():
            string += serialPort.read().decode('unicode_escape') #por fazer
       
       string = string.replace(';','')
       string = string.split(',')
       
       return string

    def velocity(self):
        string = ""
        a=bytes('@,'+str(198)+','+str(round(14*self.left_speed))+','+str(round(14*self.right_speed))+';', 'unicode_escape')
        
        serialPort.write(a)
        time.sleep(.0035)    
        while serialPort.inWaiting():
              string += serialPort.read().decode('unicode_escape')
        serialPort.flush()
        
        return string
    
    def timer_callback(self):
        out = self.velocity()
        left_ticks  = Float32()
        right_ticks = Float32()
        string_list = self.encoders(serialPort)
        try:
           left_ticks.data  = -float(string_list[-2])
           right_ticks.data = -float(string_list[-1])
           
        except:
           return
       
        
        self.pub_lmotor.publish(left_ticks)
        self.pub_rmotor.publish(right_ticks)
        self.node.get_logger().info(str(left_ticks.data)+"  "+str(right_ticks.data))
        
              
    def cmd_vel_callback(self, msg):
        self.linearVelocity  = msg.linear.x
        self.angularVelocity = msg.angular.z
        
        self.left_speed  =   -1* self.linearVelocity + self.gradient*self.angularVelocity * self.wheelSeparation/2.0
        self.right_speed =   -1* self.linearVelocity - self.gradient*self.angularVelocity * self.wheelSeparation/2.0
	    

    def spin(self):
        self.node.get_logger().info("Start differential_drive_controller")
        self.lastTwistTime = self.node.get_clock().now().nanoseconds
        rate = self.node.create_rate(self.rate)
        
        rclpy.spin(self.node)

if __name__ == '__main__':
    controller = Controller()
    controller.spin()


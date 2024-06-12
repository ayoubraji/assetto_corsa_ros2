#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from deep_orange_msgs.msg import CtReport
from ac_actuation_msgs.msg import BrakeCmd, AcceleratorPedalCmd, SteeringCmd
from std_msgs.msg import UInt8
from receiver.vjoy import vj, setJoy
from receiver.sim_info import info

import traceback
import math
import time

SCALE = 16384
class Listener(Node):
    def __init__(self):
        super().__init__('listener')   
        self.get_logger().info('init')
        self.create_subscription(BrakeCmd, "/ac_actuation/brake_cmd", self.callback_brake, 1) 
        self.create_subscription(AcceleratorPedalCmd, "/ac_actuation/accelerator_pedal_cmd", self.callback_acc, 1)
        self.create_subscription(SteeringCmd, "/ac_actuation/steering_cmd", self.callback_steer, 1)
        self.create_subscription(UInt8, "/ac_actuation/gear_cmd", self.callback_gear, 1)
        self.create_subscription(CtReport, "/ac_actuation/ct_report", self.callback_ct_report, 1)
        #global variables
        self.ct_12_stop = False
        #global steer 
        self.steer= 1.0
        #global acc 
        self.acc = 0.0
        #global brake
        self.brake = 0.0
        global onButtons
        onButtons = 0

    def callback_ct_report(self, data):
        if data.ct_state == 12:
            self.ct_12_stop = True
            #self.steer= 1.0
            self.acc = 0.0
            self.brake = 0.5
            print("CT12 triggered")
            setJoy(self.steer, self.acc, self.brake, onButtons, SCALE)
        elif (data.ct_state == 7) and (self.ct_12_stop is True):
            self.ct_12_stop = False

    def callback_brake(self, data):
        if self.ct_12_stop is False:
            self.brake = (data.pedal_cmd)/2700
        #self.get_logger().info('brake')
        print(self.brake)
        #self.brake = data.pedal_cmd
        setJoy(self.steer, self.acc, self.brake, onButtons, SCALE)
        self.get_logger().info("Setting vJoy to [" + str(self.acc) + ',' + str(self.brake) + ',' + str(self.steer) + ']')
        #rolling counter can't be set on vJoy

    def callback_steer(self, data):
        self.get_logger().info('steer')
        #self.steer = (data.angle_cmd / 600) + 1 
        if self.ct_12_stop is False:
            self.steer = (-data.angle_cmd/220) + 1 #TO-DO: change angle max to parameter
            #self.steer = ((-data.angle_cmd)/180 ) + 1
        if(self.steer < 0):
            self.steer = 0
        elif(self.steer > 2):
            self.steer = 2

        #self.steer = data.angle_cmd / (math.pi) + 1 
        setJoy(self.steer, self.acc, self.brake, onButtons, SCALE)
        self.get_logger().info("Setting vJoy to [" + str(self.acc) + ',' + str(self.brake) + ',' + str(self.steer) + ']')
        #rolling counter can't be set on vJoy

    def callback_acc(self, data):        
        self.get_logger().info('acc')
        if self.ct_12_stop is False:
            self.acc = data.pedal_cmd/100 
        #self.acc = data.pedal_cmd     
        setJoy(self.steer, self.acc, self.brake, onButtons, SCALE)
        self.get_logger().info("Setting vJoy to [" + str(self.acc) + ',' + str(self.brake) + ',' + str(self.steer) + ']')
        #rolling counter can't be set on vJoy
    
    def callback_gear (self, data):
        #not working
        gear = info.physics.gear -1
        print("current gear: %d | required gear is %d " % (gear, data.data))
        if gear > data.data:     
            setJoy(self.steer, self.acc, self.brake, 0x00000002, SCALE)
        elif gear < data.data:
            setJoy(self.steer, self.acc, self.brake, 0x00000001, SCALE)
 
        
    def callback(self, data):

        self.steer = data.steer_angle / (math.pi) + 1 
        if data.accel > 0:
            self.brake = 0
            throttle = data.accel
        else:
            self.brake = -1 * data.accel
            throttle = 0

        setJoy(self.steer, throttle, self.brake, SCALE)
        time_now = self.get_clock().now().to_msg()
        delay = time_now.sec + time_now.nanosec*1e-9 - data.header.stamp.sec -data.header.stamp.nanosec*1e-9
        self.get_logger().info("Setting vJoy to [" + str(throttle) + ',' + str(self.brake) + ',' + str(self.steer) + ']' + ", Delay: " + str(delay))
        # rospy.loginfo("My time: " + str(rospy.Time.now()))
        # rospy.loginfo("Other time: " + str(data.header.stamp))



def main(args=None):
    rclpy.init(args=args)

    vj.open()
    listener = Listener()
    rclpy.spin(listener)
    vj.close()

    
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
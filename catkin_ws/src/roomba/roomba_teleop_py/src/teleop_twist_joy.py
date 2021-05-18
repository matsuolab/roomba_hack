#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopTwistJoy:
    def __init__(self):
        rospy.Subscriber('/joy', Joy, self.joy_callback)
       # rospy.Subscriber('/planner/cmd_vel', Twist, self.cmd_callback)
        
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.hz = 20
        self.max_speed = 0.5
        self.max_yawrate = 0.5
        self.vel_ratio = 0.5

    def process(self):
        self.joy_vel = Twist()
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.vel_pub.publish(self.joy_vel) 
            rate.sleep()

    def joy_callback(self, joy):
        self.joy_vel = Twist()
        
        self.joy_vel.linear.x = joy.axes[1]*self.max_speed
        self.joy_vel.angular.z = joy.axes[0]*self.max_yawrate

        if joy.axes[7] == 1.0:
            self.joy_vel.linear.x = self.vel_ratio*self.max_speed
            self.joy_vel.angular.z = 0.0
        elif joy.axes[7] == -1.0:
            self.joy_vel.linear.x = -self.vel_ratio*self.max_speed
            self.joy_vel.angular.z = 0.0
        elif joy.axes[6] == 1.0:
            self.joy_vel.linear.x = 0.0
            self.joy_vel.angular.z = self.vel_ratio*self.max_yawrate
        elif joy.axes[6] == -1.0:
            self.joy_vel.linear.x = 0.0
            self.joy_vel.angular.z = -self.vel_ratio*self.max_yawrate

   # def cmd_callback(self, cmd):
      # self.cmd_vel = cmd

if __name__ == '__main__':
    rospy.init_node('teleop_twist_joy')

    teleop_twist_joy = TeleopTwistJoy()
    teleop_twist_joy.process()

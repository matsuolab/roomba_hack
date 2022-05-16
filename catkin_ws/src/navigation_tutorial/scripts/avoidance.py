#!/usr/bin/env python3
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Avoidance:
    def __init__(self):
        rospy.init_node('avoidance', anonymous=True)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/planner/cmd_vel', Twist, queue_size=10)

        # Subscriber
        scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)

        self.min_range = None

    def callback_scan(self, data):
        fov = np.deg2rad(60)
        min_range = data.range_max
        min_idx = -1
        angle = data.angle_min
        for idx, r in enumerate(data.ranges):
            angle += data.angle_increment
            if -fov<angle<fov:
                if r<min_range:
                    min_range = r
                    min_idx = idx
        if min_idx < len(data.ranges)/2.0:
            self.direction = "RIGHT"
        else:
            self.direction = "LEFT"
        self.min_range = min_range

    def process(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            vel = Twist()
            if self.min_range is not None:
                if self.min_range >= 0.4:
                    vel.linear.x = 0.2
                    vel.angular.z = 0.0
                else:
                    vel.linear.x = 0.0
                    if self.direction == "RIGHT":
                        vel.angular.z = 0.5
                    elif self.direction == "LEFT":
                        vel.angular.z = -0.5
            self.cmd_vel_pub.publish(vel)
            r.sleep()

if __name__=='__main__':
    avoidance = Avoidance()
    try:
        avoidance.process()
    except rospy.ROSInitException:
        pass

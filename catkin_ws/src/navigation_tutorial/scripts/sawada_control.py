#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def time_control(pub, velocity, yawrate, time):
    vel = Twist()
    start_time = rospy.get_rostime().secs
    while(rospy.get_rostime().secs-start_time<time):
        vel.linear.x = velocity
        vel.angular.z = yawrate
        pub.publish(vel)
        rospy.sleep(0.1)

def simple_controller():
    rospy.init_node('simple_controller', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    for i in range(4):
        time_control(pub, 0.0, 0.0, 0.5)
        time_control(pub, 0.6, 0.0, 1.0)

        time_control(pub, 0.0, 0.0, 0.5)
        time_control(pub, 0.0, 0.5, 2.0)
    #time_control(pub,  0.0,  0.0, 0.5)
    #time_control(pub,  0.3,  0.0, 2.0)

    #time_control(pub,  0.0,  0.0, 0.5)
    #time_control(pub, -0.3,  0.0, 2.0)

    #time_control(pub,  0.0,  0.0, 0.5)
    #time_control(pub,  0.0,  0.5, 2.0)

    #time_control(pub, 0.0, 0.0, 0.5)
    #time_control(pub, 0.3, 0.0, 2.0)

    #time_control(pub,  0.0,  0.0, 0.5)
    #time_control(pub,  0.0, -0.58, 2.0)

if __name__=='__main__':
    try:
        simple_controller()
    except rospy.ROSInitException:
        pass

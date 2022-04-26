#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

class Goal:
    def __init__(self):
        rospy.init_node('set_goal', anonymous=True)
        self.ps_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
 
    def setGoal(self, px, py, pz, ow):
        rospy.sleep(1.0)
 
        now = rospy.Time.now()
        goal_point = PoseStamped()
 
        goal_point.pose.position.x = px
        goal_point.pose.position.y = py
        goal_point.pose.position.z = pz
        goal_point.pose.orientation.w = ow
        goal_point.header.stamp = now 
        goal_point.header.frame_id = 'map'
 
        self.ps_pub.publish(goal_point)
 
 
if __name__ == '__main__':
   
    try:
        goal_ob = Goal()
        goal_ob.setGoal(2.0, 4.0, 0.0, 1.0)
        rospy.spin()
 
    except rospy.ROSInterruptException: pass
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf
from geometry_msgs.msg import Quaternion

class TopicGoal:
    def __init__(self):
        rospy.init_node('topic_goal', anonymous=True)
        self.ps_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.sleep(1.0)
 
    def set_goal(self, x, y, yaw):
        self.goal = PoseStamped() # goalのメッセージの定義
        self.goal.header.stamp = rospy.Time.now() # 現在時刻
        self.goal.header.frame_id = 'map' # マップ座標系でのゴールとして設定

        # ゴールの姿勢を指定
        self.goal.pose.position.x = x
        self.goal.pose.position.y = y

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)  # 回転はquartanionで記述するので変換
        self.goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
 
    def send_topic(self):
        self.ps_pub.publish(self.goal)
 
 
if __name__ == '__main__':
    tg = TopicGoal()
    tg.set_goal(1.0, 2.0, 0.0)
    tg.send_topic()

#!/usr/bin/env python3

import actionlib
import tf
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion

rospy.init_node('avoid')
action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
action_client.wait_for_server()  # action serverの準備ができるまで待つ

goal = MoveBaseGoal()  # goalのメッセージの定義
goal.target_pose.header.frame_id = 'map'  # マップ座標系でのゴールとして設定
goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻
# ゴールの姿勢を指定
goal.target_pose.pose.position.x = 1.0
goal.target_pose.pose.position.y = 2.0
q = tf.transformations.quaternion_from_euler(0, 0, 0)  # 回転はquartanionで記述するので変換
goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

action_client.send_goal(goal)  # ゴールを命令

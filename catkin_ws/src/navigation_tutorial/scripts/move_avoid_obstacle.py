#!/usr/bin/env python3
import actionlib
import tf
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion

class ActionGoal():
    def __init__(self):
        rospy.init_node('action_goal')
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action_client.wait_for_server()  # action serverの準備ができるまで待つ

    def set_goal(self, x, y, yaw):
        self.goal = MoveBaseGoal()  # goalのメッセージの定義
        self.goal.target_pose.header.frame_id = 'map'  # マップ座標系でのゴールとして設定
        self.goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻
        
        # ゴールの姿勢を指定
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)  # 回転はquartanionで記述するので変換
        self.goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    def send_action(self, duration=30.0):
        self.action_client.send_goal(self.goal)  # ゴールを命令
        while not rospy.is_shutdown():
            result = self.action_client.get_result()
            if result is not None:
                return result
            #result = self.action_client.wait_for_result(rospy.Duration(duration))
        print("out while")
        self.action_client.cancel_goal()
        return False

if __name__ == '__main__':
    ag = ActionGoal()
    ag.set_goal(2.0, 4.0, 0.0)
    res = ag.send_action()
    print(res)
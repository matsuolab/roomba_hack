---
title: ROSのパッケージ・ワークスペース
date: '2022-04-05'
type: book
weight: 21
---

ROSのパッケージ管理について理解しよう
<!--more-->

## Learn

### ROSのパッケージ

ROSでは、特定の目的のためのプログラム群をまとめてパッケージとして管理する。

例として、navigation_tutorailパッケージのファイル構成を示す。

```
navigation_tutorial
   ├── CMakeLists.txt
   ├── launch
   │   ├── amcl.launch
   │   ├── avoidance.launch
   │   ├── gmapping.launch
   │   ├── go_straight.launch
   │   ├── localization.launch
   │   ├── map_server.launch
   │   ├── move_base.launch
   │   └── navigation.launch
   ├── package.xml
   ├── params
   │   ├── base_global_planner_params.yaml
   │   ├── base_local_planner_params.yaml
   │   ├── costmap_common_params.yaml
   │   ├── dwa_local_planner_params.yaml
   │   ├── global_costmap_params.yaml
   │   ├── local_costmap_params.yaml
   │   └── move_base_params.yaml
   ├── scripts
   │   ├── avoidance.py
   │   ├── simple_control2.py
   │   └── simple_control.py
   └── src
       ├── avoidance.cpp
       └── go_straight.cpp

```

一般的に、`scripts`ディレクトリや`src`ディレクトリにそれぞれPython, C++のプログラムが配置される。

作成したプログラムは`rosrun`コマンドで実行することができる。

```shell
(Python) rosrun navigation_tutorail simple_control2.py
(C++) rosrun navigation_tutorail go_straight
```

`launch`ディレクトリに入っているlaunchファイルは複数のプログラムを同時に実行できるための仕組みである。

launchファイルについてでも同様に`roslaunch`コマンドで実行することができる。

```shell
(Python) roslaunch navigation_tutorial move_base.launch
```

実行時にパッケージ名(今回だとnavigation_tutorial)を指定するので、現在どこのディレクトリにいるかに関係なく実行が可能である。

### ROSのワークスペース

ROSのパッケージはワークスペースと呼ばれる作業スペースに配置される。

一般的に`catkin_ws`という名前が使われることが多い。

catkin_wsのファイル構成を示す。

```
catkin_ws
   ├── build
   ├── devel
   └── src
       ├── CMakeLists.txt
       ├── navigation_tutorial
       │   ├── CMakeLists.txt
       │   ├── launch
       │   ├── package.xml
       │   ├── params
       │   ├── scripts
       │   └── src
       └── roomba
           ├── roomba_bringup
           │   ├── CMakeLists.txt
           │   ├── config
           │   ├── launch
           │   └── package.xml
           ├── roomba_description
           │   ├── CMakeLists.txt
           │   ├── config
           │   ├── launch
           │   ├── meshes
           │   ├── package.xml
           │   └── urdf
           ├── roomba_gazebo
           │   ├── CMakeLists.txt
           │   ├── launch
           │   └── package.xml
           └── roomba_teleop
               ├── CMakeLists.txt
               ├── include
               ├── launch
               ├── package.xml
               └── src
```

catkin_wsのsrc内でパッケージ作成を行い、catkin_ws直下で`catkin_make`コマンドを実行すると、Cプログラムのビルドが行われ、buildディレクトリとdevelディレクトリが作成される。

作成されたdevelディレクトリの中のsetup.bashをソース`source devel/setup.bash`することで、ワークスペース内のパッケージのパスを通すことができる。　

パッケージのパスを通すことで、ROSのパッケージに関するコマンドや、プログラムの実行(`rosrun`や`roslaunch`)が行えるようになる。
 
### ROSのコマンド

ROSのコマンドのうち、よく用いるものを紹介する。

- Topic関連

```
rostopic list                 　　　　　　   topicの一覧を表示する
rostopic echo <topic name>   　 　　　　　   指定されたtopicの中身を表示する
rostopic hz <topic name>      　　　　　　　  topicの配信周波数を取得する
rostopic info <topic name>    　　　　　　　  topicの情報を表示する
rostopic pub <topic name> <topic> 　 　topicを配信する
rostopic type <topic name>          topicの型を確認する  
```

- Node関連

```
rosnode list                nodeの一覧を表示する
rosnode ping <node name>    nodeの接続テストを行う
rosnode info <node name>    nodeの情報を表示する
rosnode kill <node name>    nodeをシャットダウンする
```

- Package関連
```
rospack list             packageの一覧を表示する
roscd <package name>     指定したpackage内に移動する
```

### ROSのプログラムの書き方

それでは実際にプログラム例を見てみましょう。

```python:simple_control.py
#!/usr/bin/env python3

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

    time_control(pub,  0.0,  0.0, 0.5)
    time_control(pub,  0.3,  0.0, 2.0)

    time_control(pub,  0.0,  0.0, 0.5)
    time_control(pub, -0.3,  0.0, 2.0)

    time_control(pub,  0.0,  0.0, 0.5)
    time_control(pub,  0.0,  0.5, 2.0)

    time_control(pub,  0.0,  0.0, 0.5)
    time_control(pub,  0.0, -0.5, 2.0)

if __name__=='__main__':
    try:
        simple_controller()
    except rospy.ROSInitException:
        pass
```

まずsimple_controller関数内をみていきましょう。

以下の部分で`simple_controller`という名前でノードを定義しています。

```python
rospy.init_node('simple_controller', anonymous=True)
```

以下の部分でPublisher(トピックのpublish)を宣言しています。

今回の場合は、`/cmd_vel`トピックを`Twist`型で送信するPublisherを宣言しています。

```python
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
```

続いて、time_control関数です。

この関数はpublisher、速度、角速度、時間を受け取り、速度指令をpublishします。

```python
def time_control(pub, velocity, yawrate, time):
    vel = Twist()
    start_time = rospy.get_rostime().secs
    while(rospy.get_rostime().secs-start_time<time):
        vel.linear.x = velocity
        vel.angular.z = yawrate
        pub.publish(vel)
        rospy.sleep(0.1)
```

ここでTwist型のインスタンスを作成しています。

```python
  vel = Twist()
```

while文で受け取った時間が過ぎるまでの間、受け取った速度と各速度をvelに格納し、`pub.publish(vel)`でpublishを行なっています。

```python
    while(rospy.get_rostime().secs-start_time<time):
        vel.linear.x = velocity
        vel.angular.z = yawrate
        pub.publish(vel)
        rospy.sleep(0.1)
```

## 演習

{{< spoiler text="【jetson・開発マシン】それぞれdockerコンテナを起動" >}}

jetsonでdockerコンテナを起動

```shell
(開発PC):~$ ssh roomba_dev1
(jetson):~$ cd ~/group_a/roomba_hack
(jetson):~/group_a/roomba_hack ./RUN-DOCKER-CONTAINER.sh
(jetson)(docker):~/roomba_hack#  
```
開発PCでdockerコンテナを起動

```shell
(開発PC):~$ cd ~/group_a/roomba_hack
(開発PC):~/group_a/roomba_hack ./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
(開発PC)(docker):~/roomba_hack#  
```
{{< /spoiler >}}

{{< spoiler text="【jetson・開発マシン】ビルドをしてパスを通す" >}}

catkin_make後に`devel`と`build`ディレクトリが作成されることを確認しましょう。

```shell
(開発PC)(docker):~/roomba_hack# cd catkin_ws
(開発PC)(docker):~/roomba_hack/catkin_ws# rm -rf devel build
(開発PC)(docker):~/roomba_hack/catkin_ws# ls
(開発PC)(docker):~/roomba_hack/catkin_ws# catkin_make
(開発PC)(docker):~/roomba_hack/catkin_ws# ls
(開発PC)(docker):~/roomba_hack/catkin_ws# source ./devel/setup.bash
```
{{< /spoiler >}}

{{< spoiler text="【jetson】ROSマスタ、各種ノードを起動" >}}

```shell
(jetson)(docker):~/roomba_hack# roslaunch roomba_bringup bringup.launch
```
{{< /spoiler >}}

### ROSメッセージの可視化
{{< spoiler text="【開発PC】topicの確認" >}}

Topic関連のコマンドのところの`rostopic list`コマンドを使用してtopic一覧を表示してみましょう

```shell
(開発PC)(docker):~/roomba_hack# rostopic list
```

特定のtopicの型を確認

```shell
(開発PC)(docker)# rostopic type /camera/color/image_raw
(開発PC)(docker)# rostopic type /scan
```

その型が実際にどのような構成をしているのかは`rosmsg info <topic type>`で調べられます。

参考

sensor_msgs/LaserScan型 http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

sensor_msgs/Image型 http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html

特定のtopicの中身を確認
```shell
(開発PC)(docker)# rostopic echo /camera/color/image_raw
(開発PC)(docker)# rostopic echo /scan
```

rvizを用いて可視化
```shell
(開発PC)(docker)# rviz
```
{{< /spoiler >}}

{{< spoiler text="【開発PC】topicのpublish(配信)" >}}

topic`/cmd_vel`の情報を確認

```shell
(開発PC)(docker)# rostopic info /cmd_vel
```

topic`/cmd_vel`の型を確認

```shell
(開発PC)(docker)# rostopic type /cmd_vel
```

geometry_msgs/Twist型 http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html

topic`/cmd_vel`をpublish

```shell
(開発PC)(docker)# rostopic pub /cmd_vel geometry_msgs/Twist "linear:
    x: 1.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"
```

```shell
(開発PC)(docker)# rosrun navigation_tutorial simple_control.py
```

{{< /spoiler >}}


{{< spoiler text="Try it! 時間が余った人向け" >}}

try it! `roomba_bringup`パッケージの`bringup.launch`の中身を読んでみよう

hint roscdコマンドを使うとパッケージへ簡単に移動ができます。ファイルの中身を表示するには`cat`コマンドを使用します。

try it! 開発PCで`rosnode`関連のコマンドを使ってみよう

try it! 開発PCで`rosrun rqt_graph rqt_graph`を実行してnodeとtopicの関連を可視化してみよう

try it! 開発PCで`simple_control.py`の中身を読んでコードを変更してみよう

hint コードを編集するときはエディタを使うことがおすすめです。新しくターミナルを開いて

```shell
(開発PC):~$ cd group_a/roomba_hack
(開発PC):~group_a/roomba_hack$ code .
```

でVScodeを起動することができます。

{{< /spoiler >}}


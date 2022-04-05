---
title: ロボットシステムにおけるセンシング・アクチュエーション・通信①
date: '2022-04-05'
type: book
weight: 21
---

センサの値を読み取りロボットを動かしてみよう
<!--more-->

## Learn

### ロボットセンサの基礎知識

ロボットが動作するために必要なセンサは大きく2種類に分けられる。

1つ目が外界センサで、これはロボットが行動する環境の情報を取得するためのセンサーである。
具体的なセンサとして、
- LiDAR
- デプスカメラ
- ホイールエンコーダ 
- IMU

などがあげられる。

センサのノイズの影響を軽減するため、複数のセンサを組み合わせて利用されることもある。

2つ目は内界センサで、これは(ロボットアームのような変形可能な)ロボットが自身の内部状態を把握し、位置や姿勢を制御するために使われるセンサーである。
- 関節位置・角度センサ
- 関節姿勢センサ

などが内界センサである。

参考
- https://www.jsme.or.jp/jsme-medwiki/14:1013897#:~:text=robot%20sensor


### ROSのパッケージ

ROSのプログラムはパッケージとして管理される。

navigation_tutorailパッケージのファイル構成を示す。

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

作成したプログラムは`rosrun`コマンドで実行することができる。

```shell
(Python) rosrun navigation_tutorail simple_control2.py
(C++) rosrun navigation_tutorail go_straight
```

launchファイルについてでも同様に`roslaunch`コマンドで実行することができる。

```shell
(Python) roslaunch navigation_tutorial move_base.launch
```

実行時にパッケージを指定するので、(パスが通ってさえれば)ディレクトリに関係なく実行が可能である。

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

catkin_wsのsrc内でパッケージ作成を行い、catkin_ws直下で`catkin_make`コマンドでビルドをすると、buildディレクトリとdevelディレクトリが作成される。

develディレクトリの中のsetup.bashをソース`source devel/setup.bash`することで、ワークスペース内のパッケージのパスを通すことができる。　
 
### ROSのコマンド

ROSのコマンドのうち、よく用いるものを紹介する。

- Topic関連

```
rostopic list            topicの一覧を表示する
rostopic echo            指定されたtopicの中身を表示する
rostopic hz              topicの配信周波数を取得する
rostopic info            topicの情報を表示する
rostopic pub             topicを配信する
rostopic type            topicの型を確認する  
```

- Node関連

```
rosnode list             nodeの一覧を表示する
rosnode ping             nodeの接続テストを行う
rosnode info             nodeの情報を表示する
rosnode kill             nodeをシャットダウンする
```

- Package関連
```
rospack list             packageの一覧を表示する
roscd                    指定したpackage内に移動する
```


## 演習

{{< spoiler text="【jetson・開発マシン】ブランチ切り替え" >}}
```shell
cd roomba_hack
git fetch
git checkout lec_0405 
```
{{< /spoiler >}}

{{< spoiler text="【jetson・開発マシン】それぞれdockerコンテナを起動" >}}

try it! roomba_modeの前後で`echo $ROS_MASTER_URI`をしてみよう

```shell
cd roomba_hack
./RUN-DOCKER-CONTAINER.sh
(docker) roomba_mode
```
{{< /spoiler >}}

{{< spoiler text="【jetson・開発マシン】ビルドをしてパスを通す" >}}

try it! パスを通した後にcatkin_wsの中にあるパッケージが一覧`rospack list`に追加されているかを確認してみよう

```shell
(docker) cd catkin_ws
(docker) catkin_make
(docker) source ./devel/setup.bash
```
{{< /spoiler >}}

{{< spoiler text="【jetson】ROSマスタ、各種ノードを起動" >}}

try it! `bringup.launch`の中身を読んでみよう

hint `roscd <パッケージ名>`とするとパッケージへ簡単に移動ができる

```shell
(docker) roslaunch roomba_bringup bringup.launch
```
{{< /spoiler >}}

{{< spoiler text="【jetson】RealSenseを起動" >}}
```shell
 cd realsense_docker
 ./launch_realsense.sh
```
{{< /spoiler >}}

### ROSメッセージの可視化
{{< spoiler text="【開発PC】topicの確認" >}}

topic一覧を表示

```shell
(docker) rostopic list
```

特定のtopicの型を確認

```shell
(docker) rostopic type /camera/color/image_raw
(docker) rostopic type /scan
```

sensor_msgs/LaserScan型 http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
sensor_msgs/Image型 http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html

特定のtopicの中身を確認
```shell
(docker) rostopic echo /camera/color/image_raw
(docker) rostopic echo /scan
```

rvizを用いて可視化
```shell
(docker) rviz
```
{{< /spoiler >}}

{{< spoiler text="【開発PC】topicのpublish(配信)" >}}

topic`/cmd_vel`の情報を確認

```shell
(docker) rostopic info /cmd_vel
```

topic`/cmd_vel`の型を確認

```shell
(docker) rostopic type /cmd_vel
```

geometry_msgs/Twist型 http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html

topic`/cmd_vel`をpublish

```shell
(docker) rostopic pub /cmd_vel geometry_msgs/Twist "linear:
    x: 1.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"
```

topicをスクリプトからpublish

```shell
(docker) rosrun navigation_tutorial simple_control.py
```

try it! `simple_control.py`の中身を読んでコードを変更してみよう

{{< /spoiler >}}

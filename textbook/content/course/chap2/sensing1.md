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

参考: https://www.jsme.or.jp/jsme-medwiki/14:1013897#:~:text=robot%20sensor


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

topic`/cmd_vel`の型を確認

```shell
(docker) rostopic type /cmd_vel
```

geometry_msgs/Twist型 http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html

topic`/cmd_vel`をpublish
```shell
(docker) 
```

{{< /spoiler >}}





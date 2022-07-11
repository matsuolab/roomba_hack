---
title: ロボットシステムにおけるセンシング・アクチュエーション・通信②
date: '2022-04-05'
type: book
weight: 20
---

複数のセンサを組み合わせてよりかしこくロボットを動かしてみよう
<!--more-->

## Learn



[前回の演習](../../chap2/sensing1/#演習)では，速度と時間の指令を使ってロボットを制御しました．


周囲に障害物が何もない状況や，ロボットの滑りがない環境では，速度と時間のコマンドを使って思った通りにロボットを動かすことができるかもしれませんが，実環境では，ロボットの周囲には障害物が存在しますし，移動距離で制御する方が直感的です．

前回の演習のようにロボットに速度と時間を一回与えて，その通りに動かすようなフィードフォワード制御ではなく，今回は，ロボットが逐次的にセンサの情報を反映して振る舞いを変える{{< hl >}}フィードバック制御{{< /hl >}}を行なってみましょう．


### オドメトリのセンサ情報を使ってロボットを動かしてみよう

まずは，ロボットのタイヤの回転量から計算される移動距離である{{< hl >}}オドメトリ（odometry）{{< /hl >}}を使った制御をしてみましょう．


#### オドメトリのメッセージ（`/odom`）の中身を見てみよう

roombaのオドメトリの情報は，`/odom`トピックにpublishされています．


`rostopic echo /odom`をしてみるとメッセージとしてどんな情報が流れているかわかります．
{{< spoiler text="`rostopic echo -n 1 /odom`">}}
```bash
root@dynamics:~/roomba_hack# rostopic echo -n 1 /odom
header:
  seq: 2115
  stamp:
    secs: 1649692132
    nsecs: 791056254
  frame_id: "odom"
child_frame_id: "base_footprint"
pose:
  pose:
    position:
      x: -0.014664691872894764
      y: -0.0010878229513764381
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0056752621080531414
      w: 0.9999838955703261
  covariance: [0.08313143998384476, 0.00019857974257320166, 0.0, 0.0, 0.0, 0.004368376452475786, 0.00019857988809235394, 0.015032557770609856, 0.0, 0.0, 0.0, -0.26573312282562256, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0043683769181370735, -0.26573312282562256, 0.0, 0.0, 0.0, 6.021446704864502]
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```
{{< /spoiler >}}


`rostopic type /odom`をしてみると，メッセージとして，`nav_msgs/Odometry`型が使われていることがわかります．
{{< spoiler text="`rostopic type /odom`">}}
```bash
root@dynamics:~/roomba_hack# rostopic type /odom
nav_msgs/Odometry
```
{{< /spoiler >}}

`nav_msgs/Odometry`型の[ドキュメント](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)を確認してみると，このメッセージは`pose`と`twist`で構成されていることがわかります．

`pose`は．（`child_frame`から見た）ロボットの推定姿勢（位置と回転角）を表していて，`covariance`にはその不確かさを表す共分散が記録されています．

一方，`twist`は，（`child_frame`から見た）ロボットの速度を表していて，`pose`と同様に`covariance`にはその不確かさを表す共分散が記録されています．

なお，メッセージ型の定義は，`rosmsg info nav_msgs/Odometry`することでもコマンドから確認できます．
{{< spoiler text="`rosmsg info nav_msgs/Odometry`">}}
```bash
root@dynamics:~/roomba_hack# rosmsg info nav_msgs/Odometry
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```
{{< /spoiler >}}


#### クォータニオン(quaternion)

さて，`/odom`のトピックでは，ロボットの回転角は{{< hl >}}クォータニオン（quaternion）{{< /hl >}}で記述されています．

クォータニオンは，日本語では四元数と呼ばれ，3次元空間上での回転角を表現する方法の一つで，4つの要素を持つベクトルで表現されます．

クォータニオンによる3次元回転の表現は，角度を連続的にかつ簡潔に表現できるためROSではよく用いられます（その他には，オイラー角による表現や回転行列による表現があります）．

それぞれの回転角に関する表現のメリット・デメリットを調べてみましょう（「ジンバルロック」などのキーワードで調べるとよりよく理解できると思います）．

クォータニオンからオイラー角へは，`tf`パッケージの`tf.transformations.euler_from_quaternion`を使うことで変換できます（[ドキュメント](http://docs.ros.org/en/jade/api/tf/html/python/transformations.html#tf.transformations.euler_from_quaternion)）．


#### サブスクライバ（subscriber)の仕組みを知ろう

それでは，オドメトリ`/odom`の情報を使った制御の実装の例として`navigation_tutorial`パッケージの中の`simple_control2.py`を見てみましょう（[github](https://github.com/matsuolab/roomba_hack/blob/master/catkin_ws/src/navigation_tutorial/scripts/simple_control2.py)）．


前回までに強調されてきた通り，ROSは非同期分散のシステムを簡単に作ることができるのが特徴です．
そのため，ロボットから非同期に送られてくる`/odom`の情報をうまく扱うことが重要です．

実装例にあるように，Pythonによるノードの実装では，クラスとして定義するのがわかりやすい方法でしょう．

実装例では，`SimpleControlller`クラスとして，`simple_controller`というノードを定義しています．
以下のように，ノードを初期化する際に，コマンドを`/cmd_vel`トピックに送信するパブリッシャ（publisher)と，`/odom`を受信するサブスクライバ(subscriber)を作成しています．
```python
class SimpleController:
    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)

        self.x = None
        self.y = None
        self.yaw = None
        while self.x is None:
            rospy.sleep(0.1)
```

パブリッシャの使い方は前回の`simple_control.py`の[実装](https://github.com/matsuolab/roomba_hack/blob/master/catkin_ws/src/navigation_tutorial/scripts/simple_control.py)を確認してください．

パブリッシャと同様に，サブスクライバは`rospy`の`Subscriber`を用いて作成できます．
サブスクライバの特徴として，メッセージを受信した時の処理である{{< hl >}}コールバック（callback）{{< /hl >}}を定義できます．

この実装例では，`self.callback_odom`として定義されており，インスタンスの属性（`self.x`, `self.y`, `self.yaw`）を，受信したメッセージで変更するようなプログラムになっています．
```python
    def callback_odom(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.yaw = self.get_yaw_from_quaternion(data.pose.pose.orientation)
```
つまり，`self.x`には`/odom`から受信した位置のx座標，`self.y`には位置のy座標，`self.yaw`には，回転角のyawを格納しています．

クォータニオンとして受信した姿勢の回転角のyaw成分を取り出すための`self.get_yaw_from_quaternion`は以下のようになっています（オイラー角はroll, pitch, yawの順で返ってくるので`e[2]`でyawを取得しています）．
```python
    def get_yaw_from_quaternion(self, quaternion):
        e = tf.transformations.euler_from_quaternion(
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return e[2]
```


これらのセンサの値を使うことで，以下のように，指定した距離ロボットが移動するまで直進させ続けたり，指定した角度までロボットが回転するまで回転させ続けることができるようになります．

直進
 ```python
    def go_straight(self, dis, velocity=0.3):
        vel = Twist()
        x0 = self.x
        y0 = self.y
        while(np.sqrt((self.x-x0)**2+(self.y-y0)**2)<dis):
            vel.linear.x = velocity
            vel.angular.z = 0.0
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()
```

右回転
```python
    def turn_right(self, yaw, yawrate=-0.5):
        vel = Twist()
        yaw0 = self.yaw
        while(abs(self.yaw-yaw0)<np.deg2rad(yaw)):
            vel.linear.x = 0.0
            vel.angular.z = yawrate
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()
```

左回転
```python
    def turn_left(self, yaw, yawrate=0.5):
        vel = Twist()
        yaw0 = self.yaw
        while(abs(self.yaw-yaw0)<np.deg2rad(yaw)):
            vel.linear.x = 0.0
            vel.angular.z = yawrate
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()
```

それでは，オドメトリを使って実際にロボットを制御してみましょう．

## 演習

{{< spoiler text="【jetson・開発マシン】それぞれdockerコンテナを起動" >}}．
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

{{< spoiler text="【jetson】ROSマスタ、各種ノードを起動" >}}

```shell
(jetson)(docker):~/roomba_hack# roslaunch roomba_bringup bringup.launch
```
{{< /spoiler >}}

### ROSメッセージの可視化
{{< spoiler text="【開発PC】topicの確認" >}}

`/odom`の型を確認

```shell
(開発PC)(docker):~/roomba_hack# rostopic type /odom
```

`/odom`の中身を確認
```shell
(開発PC)(docker):~/roomba_hack# rostopic echo /odom
```
{{< /spoiler >}}

{{< spoiler text="オドメトリを使ったフィードバック制御" >}}


`simple_control2.py`を実行してみよう．

開発PCでteleopのコードを実行しましょう
```shell
(開発PC)(docker):~/roomba_hack# roslaunch roomba_teleop teleop.launch
```

このプログラムを動かすときには，コントローラの`Y`ボタンを押してから`B`ボタンを押して`auto`モードにしておきましょう．

1メートルほど前に進んだあと，左に90度程度旋回し，右に90度程度旋回したら成功です．



```shell
(開発PC)(docker):~/roomba_hack# rosrun navigation_tutorial simple_control2.py
```

try it! `simple_control2.py`の中身を読んでコードを変更してみよう

{{< /spoiler >}}

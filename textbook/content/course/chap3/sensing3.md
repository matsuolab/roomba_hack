---
title: ロボットシステムにおけるセンシング・アクチュエーション・通信③
date: '2022-04-05'
type: book
weight: 21
---

複数のセンサを組み合わせてよりかしこくロボットを動かしてみよう
<!--more-->

## Learn

### LiDARのスキャンデータを使って，障害物を回避してみよう

次に，LiDARでスキャンしたデータを使って，障害物を回避するようなプログラムを作ってみましょう．


#### LiDARスキャンのメッセージ（`/scan`）の中身を見てみよう

LiDARは，Light Detection And Rangingの略で，レーザ光を使って離れた場所にある物体形状や距離を測定するためのセンサです．
近年では，自動車の自動運転にも用いられることの多いセンサの一つです．

roombaに搭載されたLiDARセンサ（rplidar）の値は，`/scan`のトピックに流れていて，`rostopic echo /scan`をしてみるとメッセージとしてどんな情報が流れているかわかります．

大きなデータなので今回はテキストに掲載するのは省略しますが，`rostopic type /scan`をしてみると，メッセージとして，`sensor_msgs/LaserScan`型が使われていることがわかります．
{{< spoiler text="`rostopic type /scan`">}}
```bash
root@dynamics:~/roomba_hack# rostopic type /scan
sensor_msgs/LaserScan
```
{{< /spoiler >}}

`sensor_msgs/LaserScan`型の定義を確認してみましょう．
メッセージ型の定義は，[ドキュメント](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)のほか，`rosmsg info sensor_msgs/LaserScan`することでもコマンドから確認できます．
{{< spoiler text="`rosmsg info sensor_msgs/LaserScan`">}}
```bash
root@dynamics:~/roomba_hack# rosmsg info sensor_msgs/LaserScan
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```
{{< /spoiler >}}

`angle_min`にはスキャンの開始角度，`angle_max`にはスキャンの終了角度がラジアンで記録されています．
`angle_increment`は，計測した間隔がラジアンで記録されています．
`range_max`にはスキャンの間で検出された最大の距離，`range_min`には最小の距離がメートルで記録されています．


#### rvizでLiDARスキャンの値を可視化してみよう

rvizでLiDARのスキャン結果を可視化してみましょう．

`LaserScan`をAddして，`topic`に`/scan`を設定すると，以下のように，ロボットを中心にLiDARによって計測された障害物が赤く表示されます．

{{< figure src="../lidar_scan.png" caption="LiDARスキャンをrvizで可視化" >}}


#### LiDARを使って障害物を回避しよう

それでは，LiDARスキャン`/scen`の情報を使った制御の実装の例として`navigation_tutorial`パッケージの中の`avoidance.py`を見てみましょう（[github](https://github.com/matsuolab/roomba_hack/blob/master/catkin_ws/src/navigation_tutorial/scripts/avoidance.py)）．

このプログラムでは，LiDARを使って進行方向に存在する障害物を見つけ，それを回避しながら進むようにロボットを制御しています．具体的には，

- ロボットの進行方向に物体がなかったら直進
- ロボットの右側に障害物があったら左回転
- ロボットの左側に障害物があったら右回転

することで障害物を回避（ぶつかる前に方向転換）しています．


では，プログラムの中身を見ていきます．

[`/odom`を使った制御の場合](../sensing2/)と同様に，ノードを定義する際に，コマンドを送るパブリッシャと，LiDARスキャンのデータを読み取るサブスクライバを作成します．
```python
class Avoidance:
    def __init__(self):
        rospy.init_node('avoidance', anonymous=True)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/planner/cmd_vel', Twist, queue_size=10)

        # Subscriber
        scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)

        self.min_range = None
```

`/scan`のコールバックは，
```python
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
```
となっており，正面から左右60度の範囲内で最も短い距離を`self.min_range`に格納し，それが右側にあるのか左側にあるのかを`self.direction`に格納しています．．


このプログラムを実行すると`process`メソッドが（0.1秒おきに）常に実行されます．
```python
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
```

`process`メソッド内部では，格納された`self.min_range`が0.4（メートル）より大きい場合は，ロボットの前に何もないと判断して直進，小さい場合は，`self.direction`の値を見て，`RIGHT`であれば右に障害物があると判断して左回転，`LEFT`であれば左に障害物があると判断して右回転するようなプログラムになっています．

それでは，実際にLiDARを使って障害物を回避するプログラムを実行してみましょう．

## 演習

### ROSメッセージの可視化
{{< spoiler text="【開発PC】topicの確認" >}}

`/scan`の型を確認

```shell
(開発PC)(docker):~/roomba_hack# rostopic type /scan
```

`/scan`の中身を確認
```shell
(開発PC)(docker):~/roomba_hack# rostopic echo /scan
```
{{< /spoiler >}}

{{< spoiler text="LiDARスキャンを使ったフィードバック制御" >}}


`avoidance.py`を実行してみよう．

このプログラムを動かすときには，コントローラの`Y`ボタンを押してから`B`ボタンを押して`auto`モードにしておきましょう．

今回はせっかくなので，launchfileから起動してみましょう．
このlaunchfileは，`navigation_tutorial`パッケージの中の`launch`フォルダの中にある`avoidance.launch`に記述されています（[github](https://github.com/matsuolab/roomba_hack/blob/master/catkin_ws/src/navigation_tutorial/launch/avoidance.launch)）．


```shell
(開発PC)(docker):~/roomba_hack# roslaunch navigation_tutorial avoidance.launch
```

ロボットの進行方向に障害物があるときに，それを避けるように方向転換したら成功です．

try it! `avoidance.py`の中身を読んでコードを変更してみよう

{{< /spoiler >}}

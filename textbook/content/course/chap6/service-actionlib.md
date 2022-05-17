---
title: serviceとactionlib
date: '2022-01-22'
type: book
weight: 20
---

<!--more-->

## Learn

ここまでトピックを使った通信を使ってロボットシステムを構築してきました．
トピック通信は，メッセージを出版(publish，配信とも訳される)・購読（subscribe）することで通信する，相手を仮定しない非同期な通信方法でした．

しかし，もっと複雑なシステムを組む場合には，「相手の処理の結果を呼び出し側で受け取って知りたい」など様々な場合が考えられます．

このようなより複雑な通信を実現するための通信方式として，ROSにはサービス（service）とアクション（actionlib）が用意されています．

### service
これまで利用してきたトピック通信は，通信の相手を仮定しない（相手がいようといまいと関係ない）ため，ロボットシステムに特有な非同期通信・処理を実現するために簡単な方法でした．

一方で，他のノードに対して「特定の処理の依頼をして，その結果を待ちたい」場合など，同期的・双方向な通信が必要になることがあります．
例えば，あるノードの設定を変更をして，それが成功したかどうかを知りたい場合などに使えます．
サービスを使った通信は，「クライアント・サーバ」型の通信（クライアントサーバモデル, client-server model）となり，クライアントがサーバにリクエストを送ると，サーバがレスポンスを返すような仕組みになっています．

pythonでは`rospy.service`を使ってサーバを，`rospy.service_proxy`を使ってクライアントを簡単に実装できます（[参考URL](http://wiki.ros.org/ja/ROS/Tutorials/WritingServiceClient%28python%29)）．

また，コマンドラインからは
```bash
rosservice call [service] [args]
```
として，簡単にクライアントを作成できますし，システム上に存在するサービスの一覧は
```bash
rosservice list
```
とすることで表示できます．あるサービスのメッセージがどのように定義されているかは，
```bash
rosservice type [service]
```
で調べられます．

### actionlib
ここまで，トピック通信を使うことで相手を仮定しない非同期通信を，サービスを使った通信を行うことで相手のレスポンスを待つ同期的な通信を実現できることを見てきました．

サービスによる通信では，クライアントはサーバからのレスポンスを待つため，サーバで長い時間がかかるような処理を行う（計算量が大きい，または，移動に時間がかかるなど）場合には，クライアントの処理が長い間停止してしまうという問題があります．

そのため，処理の呼び出し側のプログラムをブロックせずに，かつ，処理の結果（や途中経過）を知れるような非同期通信が欲しくなります．
この要求を満たすのが，ROSのアクション(actionlib)です．

actionlibは，実はトピック通信の組み合わせとして構成されており，`goal`（命令），`result`（処理の結果），`feedback`（途中経過），`status`（サーバの状態），`cancel`（命令の取り消し）の5つのトピックからなります．
このあたりの仕様は，[qiitaのROS講座](https://qiita.com/srs/items/a39dcd24aaeb03216026#%E6%A6%82%E8%A6%81)が詳しいので参照してください．

pythonでは，actionlibのサーバやクライアントも，
```python
import actionlib
```
したのちに，他の通信方式と同様に`actionlib.SimpleActionServer`として，簡単に作成できます（[ドキュメント](http://wiki.ros.org/ja/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29)）．

今回の演習では，簡単のためaction serverの作成は行いません．
変わりに，移動のためのactionとして，`move_base`パッケージの中で定義されている`move_base`というactionを使うことにしましょう．

実はこのパッケージは
```bash
roslaunch navigation_tutorial navigation.launch
```
して`move_base`ノードを起動した際に既に利用されていました（これまでは，そのパッケージの中でサブスクライバとして定義された`move_base_simple/goal`というトピックにpublishすることで移動をしていました）．

`move_base`のパッケージの詳細は[ドキュメント](http://wiki.ros.org/move_base)を見て確認してみてください．

同様に，action clientも`actionlib.SimpleActionClient`を利用することで簡単に作成できます．

例えば，`move_base`のaction clientの実装する際には，
```python
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion

action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
action_client.wait_for_server()  # action serverの準備ができるまで待つ

goal = MoveBaseGoal()  # goalのメッセージの定義
goal.target_pose.header.frame_id = 'map'  # マップ座標系でのゴールとして設定
goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻
# ゴールの姿勢を指定
goal.target_pose.pose.position.x = X
goal.target_pose.pose.position.y = Y
q = uaternion_from_euler(0, 0, YAW)  # 回転はquartanionで記述するので変換
goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

action_client.send_goal(goal)  # ゴールを命令
```
のようにクライアントの`send_goal`メソッドでゴールを指定できます．

その後，
```python
action_client.wait_for_result(rospy.Duration(30))
```
とすると，結果が返ってくるまで（この場合30秒間），クライアントの処理をブロックできますし，
```python
result = action_client.wait_for_result(rospy.Duration(30))
```
とすることで，`result`変数に処理の結果が格納され，確認できます．


## 演習

{{< spoiler text="【jetson・開発マシン】起動準備" >}}
```shell
cd roomba_hack
git fetch
git checkout feature/integrate
(jetson) ./RUN-DOCKER-CONTAINER.sh
(開発マシン) ./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
```
{{< /spoiler >}}


{{< spoiler text="【開発マシン】scriptベースのnavigationを実行してみる" >}}
```shell
(開発マシン)(docker) roslaunch navigation_turtorial navigation.launch

(開発マシン)(docker) rosrun navigation_turtorial topic_goal.py
(開発マシン)(docker) rosrun navigation_turtorial action_goal.py
```
{{< /spoiler >}}

{{< spoiler text="【開発マシン】RealSenseで検出した障害物をコストマップに追加してみよう" >}}
```shell
(開発マシン)(docker) roslaunch three-dimensions_tutorial detection_pc.launch
```
{{< /spoiler >}}

{{< spoiler text="（総合課題）障害物を避けながらnavigationする" >}}

Lidarに映らない物体も画像ベースで検出しコストマップに追加することでナビゲーション時にぶつからないようにしましょう。

ヒント
- 物体検出結果に基づいて物体部分以外をマスクしたデプス画像をpublishする
- depth2pc.launchでそれをsubscribeし、point(cloud)に変換する
- 変換されたpointからmap座標系での位置を取得する
- costmapに反映する
- `move_base`アクションを使ってナビゲーションを実装しよう．
  - するとactionがタイムアウトした場合や，`KeyboardInterrupt`された場合に`cancel_goal`メソッドを使うことでactionをキャンセルできるように拡張できるはずです．

さらに，PyTorchを使用した自作の分類器やネット上の分類器をシステムに組み込んで（例えばセグメンテーションモデルなど），よりよく動作するように改良してみましょう．


{{< /spoiler >}}

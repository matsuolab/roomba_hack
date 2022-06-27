---
title: ROSとは
date: '2022-01-22'
type: book
weight: 21
---

ロボット開発によく用いられるROSの概要を理解する
<!--more-->

## Learn

### ROSの概要
ROS(Robot Operating System)は、ロボット・アプリケーション作成を支援するライブラリとツールを提供するミドルウェアです。
具体的には以下にあげるものをROSは提供しています。

- メッセージ通信

    プロセス間、コンピュータ間の通信ライブラリが提供されています。用途に応じて、多対多や一対多、非同期、同期などの通信形態を選択することができます。

- デバイスドライバ
    
    ロボットに搭載される多くのセンサやアクチュエータがROSのAPIで標準化された形で提供されています。

    https://github.com/ros-drivers
    http://wiki.ros.org/Sensors

- ライブラリ
    
    ロボットを動作させるソフトウェア(ナビゲーション、マニピュレーション)の基本機能の大半が提供されています。

- 視覚化ツール

    ロボットの内部状態やセンサ出力を2次元、3次元で視覚化するRvizや3次元動力学シミュレータのGazeboなどが提供されています。

- パッケージ管理

    多種多様なプログラミング言語(python, C++, ...)、依存関係で記述されたプログラム(パッケージ)同士を統合的にセットアップ、ビルド、テスト、リリースすることが可能です。

    たとえば、経路計画など処理が重いプロセスはC++で、画像認識など機械学習系のプロセスはpythonで実装し、それらプロセス間の通信を容易に実装できる。

### ROSのメッセージ通信
ロボットシステムでは、多数のプログラムを並列に実行し、それぞれがデータをやりとりします。
それらのプログラム間の通信ライブラリをROSは提供します。

- ノード(node)

    ROSでは、一つのプログラム単位を「ノード(node)」と呼びます。
    ノードは、ROSクライアントライブラリを用いて、他のノードとデータをやりとりします。
    ROSクライアントライブラリは異なるプログラミング言語で記述されたノードがやりとりできるようにしています。
    ノードは、次に述べるトピックの配信・購読、またはサービスの提供・使用が可能です。

- トピック(topic)

    ROSでの、標準的なデータ通信の経路を「トピック(topic)」と呼びます。
    ノードはメッセージをトピックへ向けて配信(Publish)し、同様に購読する(Subscribe)ことでトピックからメッセージを受け取ることができます。

    トピックには名前が付けられ、同じトピックに複数のノードがデータを送り、複数のノードがデータを受け取ることができます。

- メッセージ(message)

    トピックへ配信したり、購読したりするときのROSのデータ型のことを「メッセージ(message)」と呼びます。
    メッセージの型はmsgファイルに記述されており、使用言語に依存しないデータ形式になっています。

    以下に、物体やロボットの位置を表す時によく用いる`geomemtry_msgs/PoseStamped`型のmsgファイルを示します。
    位置情報の時間や座標フレームの情報が含まれるheaderと座標位置を表すposeで定義されています。
    ```
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
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
    ```

- サービス(service)

    「サービス(service)」はノードが他のノードとお互いに通信するための一つの手段です。
    サービスを提供しているノードに引数を渡して、関数の実行結果を戻り値として受け取ることができます。

    呼び出される側のノードは、サービス名とデータ形式の宣言を「アドバタイズ(advertise)」し、呼び出す側のノードは、サービスを「コール(call)」します。

    サービスにおいて送受信されるデータの型はsrvファイルに記述されています。
    メッセージと同様使用言語に依存しないデータ形式ですが、メッセージと異なるのは、引数と戻り値の二つの形式を定義する必要があるところです。

    以下に、srvの例として`std_srvs/SetBool`を示します。
    このように引数と戻り値の間に`---`を入れて定義します。
    ```
    bool data
    ---
    bool success
    string message
    ```

- ROSマスタ(ROS master)

    「ROSマスタ(ROS master)」は、ノード、トピックおよびサービスの名前登録を行い、それぞれのノードが他のノードから見えるようにする役割を担っています。
    通信するノード名とトピック名およびサービス名の対応が決定した後、ノード同士が「peer-to-peer」で通信します。

    ROSマスタとノード間の通信はXML-RPCを用いて行われます。
    ROSマスタを起動するには「roscore」というコマンドを実行します。


- パラメータサーバ(parameter server)

    「パラメータサーバ(parameter server)」は、設定データを複数のノードで共有するための軽量なサーバです。
    各ノードのパラメータを、パラメータサーバで一括して管理できます。
    パラメータサーバもROSマスタ同様に「roscore」コマンドで起動します。

    パラメータサーバで扱える型は、整数・小数・真偽値・辞書・リストになります。

- ROSのデータ通信のまとめ

{{< figure src="../ros_communication.png" caption="ROS通信" >}}

<!-- ### デバイスドライバ

- カメラ
- LiDAR
- IMU -->

### ROSと連動するソフトウェア
ROSは以下のソフトウェアと連動して使うためのパッケージが提供されています。
- OpenCV
    
    コンピュータビジョンの標準的なライブラリ。
    
    OpenCVのデータ形式である、MatクラスとROSのメッセージ形式を変換するcv_bridgeや３次元座標上の物体を２次元画像上に投影する機能であるimage_geometryといったパッケージ(vision_opencv)が提供されています。

- PCL(Point Cloud Library)

    3次元点群処理のライブラリ。

    OpenCV同様PCLのデータ形式とROSのメッセージ形式を変換するパッケージが提供されています。

- OpenSLAM, Navigation Stack

    移動ロボットの自己位置推定と地図生成を同時に行うSLAM(Simultaneous Localization and Mapping)のソースコードを公開するためのプラットフォームと、。

    ROSではOpenSLAMで実装されているgmappingパッケージのラッパーやそれと連携して自律走行を実現するnavigationメタパッケージが提供されています。

- Move it

### 視覚化ツール
- rqt

{{< figure src="../ros_gui.png" caption="rqt window" >}}

 <!-- http://wiki.ros.org/rqt -->

- rviz

{{< youtube i--Sd4xH9ZE >}}
<!-- http://wiki.ros.org/ja/rviz -->

- gazebo

<!-- ### パッケージ管理
- プログラミング言語
- rosdep
-  -->

## 演習
{{< spoiler text="roomba driverを起動し、動作していることを確認する" >}}

- jetsonにアクセスする
    ``` sh
    (開発PC):~$ ssh roomba_dev1
    (jetson):~$
    ```

- docker containerを起動する
  余裕があれば`RUN-DOCKER-CONTAINER.sh`ファイルの中身を確認してみましょう。
    ``` sh
    (jetson):~$ cd ~/team_a/roomba_hack
    (jetson):~/team_a/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh
    root@roomba-dev-jetson:~/roomba_hack#
    ```
  `root@roomba-dev-jetson:~/roomba_hack#`などと表示されればdocker内部に入れています。
  
  今後docker内部であることは(docker)と表記します。

- roomba driverなどを起動するlaunchファイルを起動する
  このタイミングでルンバの電源が入っているかを確認しておきましょう。
    ``` sh
    (jetson)(docker):~/roomba_hack# roslaunch roomba_bringup roomba_bringup.launch
    ```
  起動に成功すればルンバからピッと短い音が鳴り、ターミナルには赤い文字が出続けるはずです。

{{< /spoiler >}}


{{< spoiler text="コントローラーを使って、ロボットを動かす" >}}

- 開発PCでdocker containerを起動する
　　xにはroomba_devの後につく数字を入れてください。
    ``` sh
    (開発PC):~$ cd ~/team_a/roomba_hack
    (開発PC):~/team_a/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
    ```
    
- コントローラーを起動
  コントローラーが開発PCに刺さってることを確認してください。
    ``` sh
    (開発PC)(docker):~/roomba_hack# roslaunch roomba_teleop roomaba_teleop.launch
    ```

- コントローラのモード
    - 移動・停止 
    - 自動・マニュアル
    - ドッキング・アンドッキング

- コントローラによる操縦
    - 移動ロック解除
        L2を押している時のみ移動コマンドが動作します。
    - 左ジョイスティック
        縦方向で前進速度(手前に倒すとバック)、横方向は回転速度に対応しています。
    - 左矢印
        それぞれ、一定に低速度で前進・後退・回転します。

- 正常に起動できているかを確認
  開発PCで新しくターミナルを開いてdockerの中に入ります。
  
  すでに開発PCで起動されているdockerコンテナに入る場合は、
  
  ``` sh
  (開発PC):~/team_a/roomba_hack$ docker exec -it roomba_hack bash
  ```
  または
  ``` sh
  (開発PC):~/team_a/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh
  ```
  のいずれかの方法で入ることができます。
   
  さまざまなコマンドを使ってroombaの情報を取得してみましょう。
    ``` sh
    (開発PC)(docker):~/roomba_hack# rosnode list
    (開発PC)(docker):~/roomba_hack# rostopic list
    (開発PC)(docker):~/roomba_hack# rostopic echo /cmd_vel
    (開発PC)(docker):~/roomba_hack# rqt_graph
    (開発PC)(docker):~/roomba_hack# rviz
    ```

{{< /spoiler >}}

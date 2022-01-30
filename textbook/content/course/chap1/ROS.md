---
title: ROSとは
date: '2022-01-22'
type: book
weight: 21
---

ロボット開発によく用いられるROSの概要を理解する
<!--more-->

## Learn
{{< spoiler text="ROSの概要" >}}
ROS(Robot Operating System)は、ロボット・アプリケーション作成を支援するライブラリとツールを提供するミドルウェアです。
具体的には以下にあげるものをROSは提供しています。

- メッセージ通信

    プロセス間、コンピュータ間の通信ライブラリが提供されています。用途に応じて、多対多や一対多、非同期、同期などの通信形態を選択することができます。

- デバイスドライバ
    
    ロボットに搭載される多くのセンサやアクチュエータがROSのAPIで標準化された形で提供されています。

- ライブラリ
    
    ロボットを動作させるソフトウェア(ナビゲーション、マニピュレーション)の基本機能の大半が提供されています。

- 視覚化ツール

    ロボットの内部状態やセンサ出力を2次元、3次元で視覚化するRvizや3次元動力学シミュレータのGazeboなどが提供されています。

- パッケージ管理

    多種多様なプログラミング言語、依存関係で記述されたプログラム(パッケージ)同士を統合的にセットアップ、ビルド、テスト、リリースすることが可能です。
{{< /spoiler >}}

{{< spoiler text="ROSのメッセージ通信" >}}
- node
- topic
- message
- service
- ROS master
- parameter
{{< /spoiler >}}

{{< spoiler text="デバイスドライバ" >}}

- カメラ
- LiDAR
- IMU

{{< /spoiler >}}

{{< spoiler text="ライブラリ(ROSと連動するソフトウェア)" >}}
ROSは以下のソフトウェアと連動して使うためのパッケージが提供されています。
- OpenCV
    
    コンピュータビジョンの標準的なライブラリ。
    
    OpenCVのデータ形式である、MatクラスとROSのメッセージ形式を変換するcv_bridgeや３次元座標上の物体を２次元画像上に投影する機能であるimage_geometryといったパッケージ(vision_opencv)が提供されています。

- PCL(Point Cloud Library)

    3次元点群処理のライブラリ。

    OpenCV同様PCLのデータ形式とROSのメッセージ形式を変換するパッケージが提供されています。

- OpenSLAM

    移動ロボットの自己位置推定と地図生成を同時に行うSLAM(Simultaneous Localization and Mapping)のソースコードを公開するためのプラットフォーム。

    ROSではOpenSLAMで実装されているgmappingパッケージのラッパーやそれと連携して自律走行を実現するnavigationメタパッケージが提供されています。

- Navigation Stack

- Move it

{{< /spoiler >}}

{{< spoiler text="視覚化ツール" >}}

- rqt
- rviz
- gazebo

{{< /spoiler >}}

{{< spoiler text="パッケージ管理" >}}
- プログラミング言語
- rosdep
- 
{{< /spoiler >}}


## 演習
{{< spoiler text="roomba driverを起動し、動作していることを確認する" >}}
{{< /spoiler >}}

{{< spoiler text="コントローラーを使って、ロボットを動かす" >}}
{{< /spoiler >}}

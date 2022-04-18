---
title: ナビゲーション
date: '2021-01-01'
type: book
weight: 21
---

<!--more-->

## Learn

### Navigationシステム

ナビゲーションは、地図上の任意の目標地点へ、障害物を避けながらなるべく早く自律して移動することが目的です。

ナビゲーションシステムの出力はロボットへの行動指令値(速度など)ですが、入力は以下の4つになります。
- 地図
- 目標位置
- 自己位置推定結果
- リアルタイムのセンサ情報(LiDARスキャン情報など)

ナビゲーションでは、地図全体とロボット周辺(センサで見える範囲)の大きく2つに分けて考えることが多いです。

地図全体を考えるグローバルパスプランでは、地図情報とゴール情報から大まかなゴールまでの経路を算出します。

ロボット周辺を考える ローカルパスプランでは、グローバルで算出した経路に沿うようにしつつ、周辺の障害物情報を避ける行動指令値を算出します。

それぞれの経路を考えるにあたって、経路のコストがどうなるか重要になります。
このコストを表現する方法として、コストマップが用いられることが多いです。

{{< figure src="../overview_tf_small.png" caption="Navigationシステム概要(from [ROS wiki](https://wiki.ros.org/move_base))" >}}

### Cost Map
コストマップは、経路を算出するために用いることから、扱いやすいグリット上の占有格子地図という形で表現されることが多いです。

(空を飛んだり、3次元地形を考えなくていい場合は、基本2次元で表現します。)

経路は格子地図上で、点で扱うことが多いですが、ロボット自身はある程度の大きさを持っているので、スキャン情報で得られた点ギリギリに経路を生成すると、衝突してしまします。

そのため、コストマップでは以下の図のようにスキャンで得られた点(図中の赤点)から、ロボットが入ってほしくない範囲にコスト(図中の青く塗りつぶされているところ)が付与するという表現をします。

{{< figure src="../costmap_rviz.png" caption="コストマップ概要(from [ROS wiki](https://wiki.ros.org/costmap_2d))" >}}

### Global Path Planning

グローバルパスプランの例として、グラフ探索を利用したダイクストラ法やA*法などで経路探索をすることがあります。

{{< figure src="../astar.gif" caption="グローバルパスプランの例(from [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics))" >}}

### Local Path Planning

局所経路計画(Local Path Planning)は、ロボット周辺の障害物を避けながら、目標値へ早く行けるような経路(ロボットの行動)を算出するモジュールです。

代表的なアルゴリズムとしてDynamic Window Approach(DWA)というものがあります。
{{< figure src="../local_plan.png" caption="ローカルパスプラン概要(from [ROS wiki](https://wiki.ros.org/base_local_planner))" >}}

アルゴリズムの概要は以下になります。
1. ロボットの行動空間から行動をサンプル
2. サンプルした行動とロボットの運動モデルを用いて、一定時間シミュレーションをして経路を生成
3. 生成した経路ごとに、コストマップやゴール情報からコストを算出
4. コスト最小の経路を選択し、ロボットの指令値とする
5. 1~4を繰り返す

## 演習
<!-- {{< spoiler text="Dockerfileにnavigationを追加してBuildする" >}}
{{< /spoiler >}} -->

{{< spoiler text="navigationをlaunchして、rviz上で指定した位置までナビゲーションさせてみる" >}}

```
(docker) roslaunch navigation_tutorial navigation.launch
```
 
{{< /spoiler >}}

<!-- {{< spoiler text="navigationをlaunchして、map座標系の位置を指定してナビゲーションさせてみる" >}}
{{< /spoiler >}} -->

{{< spoiler text="navigationのparamをチューニングする" >}}

move baseのパラメータは `navigation_tutorial/params` の中にyaml形式で保存されています。

launchファイルではloadコマンドでyamlを読み込んでいます。

- [move_base](https://wiki.ros.org/move_base#Parameters)
- [base_local_planner](https://wiki.ros.org/base_local_planner#Parameters)
- [costmap_2d](https://wiki.ros.org/costmap_2d#costmap_2d.2Flayered.Parameters)

{{< /spoiler >}}

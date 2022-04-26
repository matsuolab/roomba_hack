---
title: 三次元画像処理
date: '2022-01-22'
type: book
weight: 20
---

<!--more-->

## Learn

今回はRealSenseD435というRGBDカメラを用いて三次元画像処理を行っていきましょう。


### RGBDカメラについて

RGBDカメラとは、カラーの他にデプス(深度)を取得できるカメラのことです。
複雑な動作を行うロボットを動かす際には三次元空間の把握が重要となり、RGBDカメラはよく用いられます。
比較的安価でよく利用されるRGBDカメラとして、Intel社製のRealSenseやMicrosoft社製のXtionなどがあります。

### RealSense

今回はRGBDカメラとしてRealSenseD435を使用します。

ROSで用いる際には標準のラッパー(https://github.com/IntelRealSense/realsense-ros)を使用します。

`roslaunch realsense2_camera rs_camera.launch`を行うとデフォルトのトピックとして
RGB画像の`/camera/color/image_raw`、
デプス画像の`/camera/depth/image_raw`
が利用できます。これらのトピックはいずれも`sensor_msgs/Image`型です。

RealSenseは物理的にRGB画像モジュールとデプス画像モジュールが離れているため、これら2つのトピックはいずれも画像データではあるものの、ピクセルの位置関係が対応しておらずそのままだとうまく画像処理に用いることができません。
そこで、起動時に`align:=true`を指定することで、上記のトピックに加えてデプス画像をRGB画像のピクセルに対応するように変換する`/camera/aligned_depth_to_color/image_raw`トピックを使用できるようにします。
他にも`pointcloud:=true`を指定するとデプス画像から点群を生成することができます。
しかし、この処理は比較的重たいため今回はJetsonではなく、開発用PCでこの処理を行っていくことにします。

それでは、RGB画像`/camera/color/image_raw`と整列されたデプス画像`/camera/aligned_depth_to_color/image_raw`の2種類のトピックを用いて三次元画像処理を行っていきましょう。

### 物体検出

まずはRGB画像`/camera/color/image_raw`のみを用いて三次元ではない画像検出を行っていきましょう。

以下は`/camera/color/image_raw`をSubscribeし、物体検出アルゴリズムであるYOLOv3に入力し、その結果をbounding boxとして描画し、`/detection_result`としてPublishするスクリプトです。

```
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pytorchyolo import detect, models
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cv2
import copy

class ObjectDetection:
    def __init__(self):
        rospy.init_node('object_detection', anonymous=True)

        # Publisher
        self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)

        # Subscriber
        rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback_rgb)

        self.bridge = CvBridge()
        self.rgb_image = None

    def callback_rgb(self, data):
        cv_array = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
        self.rgb_image = cv_array

    def process(self):
        path = "/root/roomba_hack/catkin_ws/src/three-dimensions_tutorial/yolov3/"

        # load category
        with open(path+"data/coco.names") as f:
            category = f.read().splitlines()

        # prepare model
        model = models.load_model(path+"config/yolov3.cfg", path+"weights/yolov3.weights")

        while not rospy.is_shutdown():
            if self.rgb_image is None:
                continue

            # inference
            tmp_image = copy.copy(self.rgb_image)
            boxes = detect.detect_image(model, tmp_image)
            # [[x1, y1, x2, y2, confidence, class]]

            # plot bouding box
            for box in boxes:
                x1, y1, x2, y2 = map(int, box[:4])
                cls_pred = int(box[5])
                tmp_image = cv2.rectangle(tmp_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                tmp_image = cv2.putText(tmp_image, category[cls_pred], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            # publish image
            tmp_image = cv2.cvtColor(tmp_image, cv2.COLOR_RGB2BGR)
            detection_result = self.bridge.cv2_to_imgmsg(tmp_image, "bgr8")
            self.detection_result_pub.publish(detection_result)


if __name__ == '__main__':
    od = ObjectDetection()
    try:
        od.process()
    except rospy.ROSInitException:
        pass
```

コールバック関数で`sensor_msgs/Image`型をnp.ndarray型に変換するために
```
cv_array = self.bridge.imgmsg_to_cv2(data, 'bgr8')
cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
```
という`sensor_msgs/Image`型特有の処理を行ってますが、Subscriberを作成しコールバック関数でデータを受け取るという基本的な処理の流れは`scan`などの他のセンサと同じです。

ここで注意してほしいのはYOLOの推論部分をコールバック関数内で行っていないことです。
一見、新しいデータが入ってくるときのみに推論を回すことは合理的に見えますが、センサの入力に対してコールバック関数内の処理が重いとセンサの入力がどんどん遅れていってしまいます。
コールバック関数内ではセンサデータの最低限の処理の記述にとどめ、重い処理は分けて書くことを意識しましょう。

ここでは既存の物体検出モジュールを使用しましたが、PyTorchなどで作成した自作のモデルも同様の枠組みで利用することができます。


続いて、デプス画像データも統合して物体を検出し、物体までの距離を測定してみましょう。


### 外部パッケージの使用

それではデプス画像からを作成しましょう。



## 演習
<!-- {{< spoiler text="Dockerfileにamclを追加してBuildする" >}}
{{< /spoiler >}} -->

{{< spoiler text="ブランチの切り替え" >}}

```
(jetson, 開発PC) git fetch
(jetson, 開発PC) git checkout feature/move-base
```

{{< /spoiler >}}


{{< spoiler text="(開発PC, jetson)起動準備" >}}

```
(jetson)./RUN-DOCKER-CONTAINER.sh
(jetson)(docker) roslaunch roomba_bringup bringup.launch
(開発PC)./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
```

{{< /spoiler >}}

{{< spoiler text="gmappingで地図作成" >}}

```
(docker) roslaunch navigation_tutorial gmapping.launch
```

地図の保存。map.pgm（画像データ）とmap.yaml(地図情報)が保存される。
```
(docker) rosrun map_server map_saver
```
`~/roomba_hack/catkin_ws/src/navigation_tutorial/map` の下に保存する。

{{< /spoiler >}}

{{< spoiler text="amclをlaunchして、自己位置推定する" >}}

localizationノードと地図サーバーを同時に起動。
```
(docker) roslaunch navigation_tutorial localization.launch
(docker) roslaunch roomba_teleop teleop.launch
(docker) rviz -d /root/roomba_hack/catkin_ws/src/navigation_tutorial/configs/navigation.rviz
```
- 初期位置の指定(rvizの2D Pose Estimate)
- コントローラで移動させてみて自己位置を確認
- rqt_tf_treeを見てみる

{{< /spoiler >}}

{{< spoiler text="amclのparamをチューニングする" >}}
launchファイルの中身を見てみて、値を変えてみる。

各パラメータの意味は[amclのページ](https://wiki.ros.org/amcl#Parameters)を参照。

例えば、・・・
- initial_cov_**を大きくしてみて、パーティクルがちゃんと収束するかみてみる。
- particleの数(min_particles、max_particles)を変えてみて挙動をみてみる。

{{< /spoiler >}}

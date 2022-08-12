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

RealSenseは物理的にRGB画像モジュールとデプス画像モジュールが離れているため、これら2つのトピックはいずれも画像データではあるものの、ピクセルの位置関係が対応しておらずそのままだとうまく画像処理に用いることができませんが、起動時に`align:=true`を指定すると、デプス画像をRGB画像のピクセルに対応するように変換された`/camera/aligned_depth_to_color/image_raw`トピックを使用できるようになります。


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

また、ここでは既存の物体検出モジュールを使用しましたが、PyTorchなどで作成した自作のモデルも同様の枠組みで利用することができます。


続いて、RGB画像に整列されたデプス画像データを統合して物体を検出し、物体までの距離を測定してみましょう。


RGB画像`/camera/color/image_raw`と整列されたデプス画像`/camera/aligned_depth_to_color/image_raw`はそれぞれ独立したトピックであるため、同期を取る必要があります。

画像の同期にはmessage_filters(http://wiki.ros.org/message_filters)がよく使われます。

message_filters.ApproximateTimeSynchronizerを使い以下のようにSubscriberを作成します。
```
rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 1.0).registerCallback(callback_rgbd)

def callback_rgbd(data1, data2):
    bridge = CvBridge()
    cv_array = bridge.imgmsg_to_cv2(data1, 'bgr8')
    cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
    self.rgb_image = cv_array

    cv_array = bridge.imgmsg_to_cv2(data2, 'passthrough')
    self.depth_image = cv_array
```
この例では、1.0秒の許容で'/camera/color/image_raw'と'/camera/aligned_depth_to_color/image_raw'のトピックの同期を取ることができれば、コールバック関数callback_rgbdが呼ばれセンサデータが受けとられます。

それでは、物体を検出し、物体までの距離を測定するスクリプトを見てみましょう。

```
#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pytorchyolo import detect, models
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cv2
import copy

class DetectionDistance:
    def __init__(self):
        rospy.init_node('detection_distance', anonymous=True)

        # Publisher
        self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)

        # Subscriber
        rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 1.0).registerCallback(self.callback_rgbd)

        self.bridge = CvBridge()
        self.rgb_image, self.depth_image = None, None

    def callback_rgbd(self, data1, data2):
        cv_array = self.bridge.imgmsg_to_cv2(data1, 'bgr8')
        cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
        self.rgb_image = cv_array

        cv_array = self.bridge.imgmsg_to_cv2(data2, 'passthrough')
        self.depth_image = cv_array

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
                cx, cy = (x1+x2)//2, (y1+y2)//2
                print(category[cls_pred], self.depth_image[cy][cx]/1000, "m")
            
            # publish image
            tmp_image = cv2.cvtColor(tmp_image, cv2.COLOR_RGB2BGR)
            detection_result = self.bridge.cv2_to_imgmsg(tmp_image, "bgr8")
            self.detection_result_pub.publish(detection_result)


if __name__ == '__main__':
    dd = DetectionDistance()
    try:
        dd.process()
    except rospy.ROSInitException:
```

基本的には物体検出のスクリプトと同じですが、
```
cx, cy = (x1+x2)//2, (y1+y2)//2
print(category[cls_pred], self.depth_image[cy][cx]/1000, "m")
```
でbounding boxの中心座標を変換し、対応する距離をメートル単位で表示しています。

整列されたデプス画像を用いているため、RGB画像に基づき算出した座標をそのまま指定できます。


### 点群の作成

上の例ではRGB画像とデプス画像を用いた三次元画像処理を行うことができました。

しかし、ロボットの自立移動などより複雑な動作をさせることを考えたとき、深度データを三次元空間にマッピングできたほうが位置関係を統一的に扱うことができ便利なこともあります。

それでデプス画像から点群と呼ばれるデータを作成することを考えます。

点群とは三次元座標値(X,Y,Z)で構成された点の集まりのことです。各点の情報として、三次元座標値に加え色の情報(R,G,B)が加わることもあります。
デプス画像はカメラの内部パラメータを用いることによって点群データに変換することができます。(https://medium.com/yodayoda/from-depth-map-to-point-cloud-7473721d3f)

今回はdepth_image_procと呼ばれる、デプス画像を点群データに変換するROSの外部パッケージ(http://wiki.ros.org/depth_image_proc) を使用して点群の変換を行います。

外部パッケージは`~/catkin_ws/src`等のワークスペースに配置し、ビルドしパスを通すことで簡単に使用できます。

depth_image_procのwikiを参考に以下のようなlaunchファイルを作成しました。

```
<?xml version="1.0"?>
<launch>
  <node pkg="nodelet" type="nodelet" name="nodelet_manager" args="manager" />

  <node pkg="nodelet" type="nodelet" name="nodelet1"
        args="load depth_image_proc/point_cloud_xyz nodelet_manager">
    <remap from="camera_info" to="/camera/color/camera_info"/>
    <remap from="image_rect" to="/camera/aligned_depth_to_color/image_raw"/>
    <remap from="points" to="/camera/depth/points"/>
  </node>
</launch>
```
このlaunchファイルを実行すると`/camera/color/camera_info`と`/camera/aligned_depth_to_color/image_raw`をSubscribeし、`/camera/depth/points`をPublishします。

`/camera/color/camera_info`は`sensor_msgs/CameraInfo`型のトピックであり、カメラパラメータやフレームid、タイムスタンプなどを保持しており、点群の変換に利用されます。
`/camera/aligned_depth_to_color/image_raw`はRGB画像に整列されたデプス画像であるため、`/camera/depth/camera_info`ではなく`/camera/color/camera_info`を指定することに注意してください。

`roslaunch three-dimensions_tutorial depth2pc.launch`を行い`/camera/depth/points`トピックをrvizで可視化をすると三次元空間に点群データが表示されているのが確認できます。

## 演習

{{< spoiler text="(開発PC, jetson)起動準備" >}}

```
(jetson)$ ./RUN-DOCKER-CONTAINER.sh
(jetson)(docker)# roslaunch roomba_bringup bringup.launch
(開発PC)$ ./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
```
{{< /spoiler >}}

{{< spoiler text="(開発PC)RealSenseのトピックの可視化" >}}
```
(開発PC)(docker) rviz
```
`/camera/color/image_raw`と`/camera/depth/image_raw`と`/camera/aligned_depth_to_color/image_raw`を可視化して違いを確認してみよう。
{{< /spoiler >}}


{{< spoiler text="(開発PC)物体検出を行う" >}}
```
(開発PC)(docker) cd catkin_ws; catkin_make; source devel/setup.bash
(開発PC)(docker) roscd three-dimensions_tutorial; cd yolov3/weights; ./download_weights.sh
(開発PC)(docker) rosrun three-dimensions_tutorial object_detection.py
rvizで`/detection_result`を表示し結果を確認してみよう。
(開発PC)(docker) rosrun three-dimensions_tutorial detection_distance.py
```
{{< /spoiler >}}

{{< spoiler text="(開発PC)外部パッケージを使用" >}}
```
(開発PC)(docker) cd ~/external_catkin_ws/src 
(開発PC)(docker) git clone https://github.com/ros-perception/image_pipeline
(開発PC)(docker) cd ../; catkin build; source devel/setup.bash
(開発PC)(docker) cd ~/roomba_hack/catkin_ws; source devel/setup.bash
(開発PC)(docker) roslaunch three-dimensions_tutorial depth2pc.launch
(開発PC)(docker) roslaunch navigation_tutorial navigation.launch
```
rvizで`/camera/depth/points`トピックを追加して確認してみよう。
{{< /spoiler >}}

{{< spoiler text="余裕がある人向け" >}}
物体を検出し、特定の物体の手前まで移動するスクリプトを作ってみましょう。

ヒント
- 物体検出結果に基づいて物体部分以外をマスクしたデプス画像をpublishする
- depth2pc.launchでそれをsubscribeし、point(cloud)に変換する
- 変換されたpointからmap座標系での位置を取得する
- navigation_tutorial/scripts/set_goal.py (map座標系で指定した位置・姿勢までナビゲーションするスクリプト)などを参考に、その位置へとナビゲーションする

PyTorchを使用した自作の分類器やネット上の分類器をシステムに組み込んでみましょう。

Lidarに映らない物体も画像ベースで検出しコストマップに追加することでナビゲーション時にぶつからないようにしましょう。
{{< /spoiler >}}

# 学习日志记录

# week5-2

## task1 跑通机械臂

#### 操作

* 直接按照机械臂文档就跑通了 。
* 现在机械臂这边还没有写一个点到点（PTP）的运动控制

#### 补充学习

1. 关于通信：

   socat指令

```
Socat 是 Linux 下的一个多功能的网络工具，名字来由是 「Socket CAT」。其功能与的 Netcat 类似，可以看做是 Netcat 的加强版。

Socat 的主要特点就是在两个数据流之间建立通道，且支持众多协议和链接方式。如 IP、TCP、 UDP、IPv6、PIPE、EXEC、System、Open、Proxy、Openssl、#Socket等。

Socat 的官方网站：http://www.dest-unreach.org/socat/
```

​	网络：remote ssh貌似没有配置成功  一直是无法连接


## task2 相机标定

#### 总体方案： matlab相机标定 + opencv手眼标定

做的标定是不含深度信息的相机镜头标定（畸变和内参）

![image-20231023083706273](.\asset\image-20231023083706273.png)

![image-20231023084039465](.\asset\image-20231023084039465.png)

![image-20231023084837417](.\asset\image-20231023084837417.png)

![image-20231023084900055](.\asset\image-20231023084900055.png)

https://matlab.mathworks.com/

与realsense-ros的数据相比对：

![Screenshot from 2023-10-23 17-01-30](.\Screenshot from 2023-10-23 17-01-30.png)



#### 补充学习：参数 

**distortion_model: plumb_bob**

For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).

**Intrinsic camera matrix for the raw (distorted) images.**
\#   [fx 0 cx]
\# K = [ 0 fy cy]
\#   [ 0 0 1]
\# Projects 3D points in the camera coordinate frame to 2D pixel
\# coordinates using the focal lengths (fx, fy) and principal point
\# (cx, cy).

内参矩阵

**Rectification matrix (stereo cameras only)**
\# A rotation matrix aligning the camera coordinate system to the ideal
\# stereo image plane so that epipolar lines in both stereo images are
\# parallel.

这是双目视觉的校准矩阵

**Projection/camera matrix**
\#   [fx' 0 cx' Tx]
\# P = [ 0 fy' cy' Ty]
\#   [ 0  0  1  0]

* By convention, this matrix specifies the intrinsic (camera) matrix of the processed (rectified) image. That is, the left 3x3 portion is the normal camera intrinsic matrix for the re ctified image.
  It projects 3D points in the camera coordinate frame to 2D pixel coordinates using the focal lengths (fx', fy') and principal point  (cx', cy') - these may differ from the values in K.

* For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will also have R = the identity and P[1:3,1:3] = K.
* For a stereo pair, the fourth column [Tx Ty 0]' is related to the position of the optical center of the second camera in the first camera's frame. We assume Tz = 0 so both cameras are in the same stereo image plane. The first camera always has Tx = Ty = 0. For the right (second) camera of a horizontal stereo pair, Ty = 0 and Tx = -fx' * B, where B is the baseline between the cameras.
* Given a 3D point [X Y Z]', the projection (x, y) of the point onto the rectified image is given by:
  \# [u v w]' = P * [X Y Z 1]'
  \#     x = u / w
  \#     y = v / w
  \# This holds for both images of a stereo pair.

https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html



(小疑惑: realsense的方案是什么：结构光 还是 双目视觉（从viewer里看到有steoro mode，得到的红外数据？）立体模式里看上去像是红外数据，而3D的深度图的显示就是彩色点云显示；看上去应该是两种测量都有，但是为什么realsense的infrastructure1和2的内参矩阵和depth是一样的？）

解答：realsense使用的是主动式 立体双目视觉；两个红外数据来进行立体测量，两者合成产生深度（故两个infra的内参和depth是相同的）。

depth的内参矩阵和color的内参矩阵是不一样的（这也是为什么可能会出现深度数据和颜色图数据的标定。



## task3 手眼标定

#### 方案：opencv + ros2传感器驱动

**opencv使用这些参数时，内参矩阵IntrinsicMatrix和旋转矩阵RotationOfCamera2需要转置后再使用。这点非常重要，否则你立体校正的结果都是错的，何况立体匹配。**

外参矩阵（camera2base）代码流程：

1. board2camera  
   * opencv:  
     * cv2.cvtColor  对图像进行apriltag检测
     * cv2.solvePnP    计算相机与apriltag的位姿
2. end2base 
   *  tf2_ros
     * 静动态坐标转换只要订阅static_tf    多坐标系转换才要用lookup_transform
     * 特定两个关节的坐标转换关系之能调用tf2_ros包，运行tf_echo在终端中，手动得到在坐标转换关系
     * 绕的弯路：一开始想使用tf2_broadcaster来手动发布坐标转换关系，和图片数据一起录下来，但是官方历程给的broadcaster的意思，其实是在已经有一个坐标系下的姿态topic发布之后，订阅再发布该位姿相对于其他关节的变换矩阵，而非实现tf2_echo的功能。
3. 生成trajectory 
   * 先读取合适标定位置的位姿，然后写入文件（或者写一个生成轨迹的文件）
   * 改用的方法：直接自己写入相应位姿
4. handeye 计算
   * opencv:   
     *  cv2.calibrateHandEye https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b
     

5. 获取图像
   * 直接使用ros2 bag record将/camera/color/raw_image记录下来，再使用rosbag2读取其中的图像数据





#### 补充学习：ros2 python

1. python并行处理 async await
2. python参数设置 **argparse:**

* argparse是python用于解析命令行参数和选项的标准模块，用于代替已经过时的optparse模块。

* argparse模块的作用是用于解析命令行参数

* declare_parameter

3. ros2时间戳处理   用于：找到某一时刻的

```
from rclpy.time import Time

get_clock()
```

4. 缓存数据

* 消息过滤器：message_filter

  * 当消息到达消息过滤器的时候，可能并不会立即输出，而是在稍后的时间点里满足一定条件下输出。

  * 基本用法：

    * 消息的订阅与回调

    message_filters::Subscriber< std_msgs::UInt32> sub(nh, "my_topic", 1);
    sub.registerCallback(myCallback);

    等价于：ros::Subscriber sub = nh.subscribe("my_topic", 1, myCallback);

    * tf::MessageFilter

    可以订阅任何的ROS消息，然后将其缓存，直到这些消息可以转换到目标坐标系，然后进行相应的处理（进入回调函数）。tf::MessageFilter的初始化需要message_filters::Subscriber，tf转换，目标坐标系，等待时间。当message_filters::Subscriber的消息能够由tf转换到目标坐标系时，调用回调函数，回调函数由tf::MessageFilter::registerCallback()进行注册。

    * 其他用法	——时间同步

      时间同步器，它接收来自多个源的不同类型的消息，并且仅当它们在具有相同时间戳的每个源上接收到消息时才输出它们，也就是起到了一个消息同步输出的效果。

      TimeSynchronizer筛选器通过包含在其header中的时间戳同步进入的通道，并以采用相同数量通道的单个回调的形式输出它们。

5. rosbag 用于对消息存储

* ros2使用的版本是：rosbag2    (pip3 install rosbags;   import rosbags.rosbag2)
* 试验成功！

6. python学习批量处理字符串（设置变量）

   ```python
   bag_file_name = '%d.bag' % i
   ```

7. tf2

   ```
   ros2 run tf2_ros tf2_echo world turtle1
   ```

8. sys.arg
   * For every invocation of Python, `sys.argv` is automatically a list of strings representing the arguments (as separated by spaces) on the command-line. The name comes from the [C programming convention](http://www.crasseux.com/books/ctutorial/argc-and-argv.html) in which argv and argc represent the command line arguments.





github push现在只能使用ssh方式：git remote sert-url origin <new_url>这个是ssh方式的url

-----

## task4 理论推进

理论部分推进情况：

* 大致看完了视觉的两章（14 15章）

* 重新过了下位姿描述，正在复习机械臂运动学（3 5章）

  

* 计划week6复习完运动学（5 6章），看完轨迹规划大部分（8 9章） ；

  * (这样就还剩下控制部分(10 11 12 13章) 到week7)





---

Tinker

* 关于跟踪的方案：Person_reID_baseline_pytorch/tutorial/README.md at master · layumi/Person_reID_baseline_pytorch (github.com)
* 位姿识别和人脸识别算法测试

---





## 10.24 沟通交流

### 下一步计划

1. 理论学习：运动学（5 6章），看完轨迹规划大部分（8 9章） ；
2. 熟悉仿真：SAPIEN (ucsd.edu)
3. 手眼标定：过程中熟悉代码和ROS

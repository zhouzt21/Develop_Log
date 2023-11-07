# 学习日志记录

# week4 

## task1环境配置和熟悉

测试和学习了tinker_sim
1. 学习内容：对ROS2基本操作的再熟练；解决环境问题；

2. 遇到的问题：

   1. 环境：发现ROS2不能使用conda环境；会反复删除修改编译的工作空间还是直接在终端里source更靠谱
   2. 找不到功能包/找不到可执行文件：检查了一切常见可能的问题之后仍出错，重建立工作空间，终端source
   3. 包依赖：import_error反复出现（解决办法：检查文件路径下是否有；打开Python看是否可以import；重启终端；重建工作空间编译；**不能使用conda环境**）（千万不要卡死在思考这个包的版本问题上，ros自带的包一般不会出问题rosidl）
   4. gazebo不能导入模型

3. 新学习和使用的工具：

   1. 终端中的键盘操作

   2. gazebo

   3. rqt的srv call

   4. rviz的坐标转换查看：fixed frame  (选择自定义的tinker；不是map)

   5. ```
      rosdep install -iyr --from-paths src   #用于配置依赖
      ```



## task2相机数据初探

### 1不同类型数据

分别下驱动开启了realsense和Kinect相机：

realsense参考：https://github.com/tinkerfuroc/tk23_ws/tree/vision/src/tinker_vision/realsense-ros#available-parameters 

成像：

* 正面的四个摄像头，从左向右以次是左红外相机，红外点阵投射仪，右红外相机，和RGB相机。

* 其原理是基于三角测量法，左右红外相机进行测量深度，中间红外点阵投射器相当于补光灯，不打开也能测深度，只是效果不好；最右边的rgb相机用于采集彩色图片，最终可以将**彩色视频流与深度流进行对齐**。

  基于左右图像的视差来求得距离。只是相比于普通彩色rgb相机，红外ir相机是用来接收目标返回的红外光线的，得到的是左右两幅红外灰度图像。若把屋里灯光关掉，黑暗的环境，红外ir相机一样可以生成深度图像，只是质量略有下降。

  

* realsense的外参：

  * 官方解释：Extrinsic from sensor A to sensor B means the **position and orientation** of sensor A **relative to **sensor B.

  depth_to_accel   depth_to_color  depth_to_infra1

* color图
* depth图
* infra话题
  
  * camera_info: d k r p 

![q_Color](.\asset\q_Color.png)

![1_Depth](.\asset\1_Depth.png)



kinect的数据：6种数据源

1. ColorFrameSource 彩色图

2. InfraredFrameSource 红外图（16-bit）

3. DepthFrameSource  
   深度图（16-bit，单位：mm）kinect采的深度数据是short型，即每个像素深度占2个字节，但是16位中只有12位是有用的，2^12 = 4096，单位是毫米，所以范围是4m

4. BodyIndexFrameSource  人物索引图（用一个字节代表人体，最多支持6个人体）

5. BodyFrameSource  人体关节图

6. AudioSource  声音

   ![Screenshot from 2023-10-18 18-37-28](.\asset\Screenshot from 2023-10-18 18-37-28.png)

   ![Screenshot from 2023-10-18 17-51-24](.\asset\Screenshot from 2023-10-18 17-51-24.png)

![Screenshot from 2023-10-18 17-51-36](.\asset\Screenshot from 2023-10-18 17-51-36.png)

方案：

1. kinect sdk本身可以用来人体跟踪？（游旺测试的是传统跟踪算法）
2. 之前用的yolo做跟踪
3. kinect sdk也有关节跟踪的部分
4. 搜一下现在做跟踪的方案

疑惑：

1. 深度数据和彩色图数据需要配准吗？





### 2已有tinker图像数据处理

1. kinect-ros驱动本身自带的内参转换

2. 像素坐标下的二维数组存储三维的点云坐标

   ```python
   def get_array_from_points(points: PointCloud2) -> (np.array, np.array):
       '''
       Get the point array from point cloud
       Returns: point_arr: [H, W, 3], valid_mask: [H, W]
       Kinect PointCloud2 format:
           height: 1
           width: 3145728
           fields:
           - name: x
               offset: 0
               datatype: 7
               count: 1
           - name: y
               offset: 4
               datatype: 7
               count: 1
           - name: z
               offset: 8
               datatype: 7
               count: 1
           - name: rgb
               offset: 16
               datatype: 7
               count: 1
           is_bigendian: false
           point_step: 32
           row_step: 100663296
       PointField datatype:
           uint8 INT8    = 1
           uint8 UINT8   = 2
           uint8 INT16   = 3
           uint8 UINT16  = 4
           uint8 INT32   = 5
           uint8 UINT32  = 6
           uint8 FLOAT32 = 7
           uint8 FLOAT64 = 8
       '''
       h, w = 1536, 2048
       arr = np.frombuffer(points.data, dtype='<f4')
       assert(len(arr) == h * w * 8)
       arr = arr.reshape((h, w, 8))[:, :, :3]
       mask = 1 - np.multiply.reduce(np.isnan(arr), axis=2)
       # remove nans
       arr = np.nan_to_num(arr, nan=0)
       return arr, mask
   ```

3. srv的方式调用识别包(yolov8)

   ![Screenshot from 2023-10-18 17-49-00](.\asset\Screenshot from 2023-10-18 17-49-00.png)

   ![Screenshot from 2023-10-18 17-49-04](.\asset\Screenshot from 2023-10-18 17-49-04.png)

4. 传入机器人坐标系



现在存在的问题：

1. 识别的物体的准确度问题
2. 点云无效数据处理
3. 直接将相机固定在机器人模型中（手动偏移），需要做标定



视觉与其他组的配合：

1. yolo识别处理部分写成service，其他的视觉人调用该服务
   1. 物体识别直接在识别的时候就跑出了类型和点云坐标取平均的质心位置
   2. 识别出的person这一帧图发给人脸识别；
      1. 也可以增加一个缓存图像的部分
   3. 跟踪任务通过比较前后两帧的人 (yolov8可以做跟踪？)

2. grounding dino这种大模型不一定能在单机上跑？



# week5

## task1标定尝试

###  1前置知识

1. 相机模型

   内参 外参矩阵
   
   相机坐标-像素坐标-世界坐标-机器人坐标
   
   ![img](https://img-blog.csdnimg.cn/2855f637f41043c8985603152214f8e9.png)
   
2. 刚体运动的描述  (手眼矩阵)

   1. 四元数: [x,y,z,qx,qy,qz,qw]  ???

   2. 旋转矩阵；

      ```markup
          [r11,r12,r13,x],
          [r21,r22,r23,y],
          [r31,r32,r33,z]
      ```

   3. roll pitch yaw

   4. 欧拉角：[x,y,z,rx,ry,rz]
   
3. 3D视觉相机方案

   目前市面上常有的3D相机方案主要有3种：
   （1）飞行时间法（Time of flight，TOF）：代表公司微软Kinect2，PMD，SoftKinect，联想Phab，在手机中一般用于3D建模、AR应用，AR测距（华为TOF镜头）。
   （2）双目视觉（Stereo Camera）：代表公司Leap Motion，ZED，大疆;
   （3）结构光（Structured-light）：代表公司有奥比中光，苹果iPhoneX（Prime Sense），微软Kinect1，英特尔RealSense，Mantis Vision等，在手机（iPhone，华为）中3D结构光主要用于人脸解锁、支付、美颜等场景。
   各种方案的测量原理、光源、精度与距离、优缺点、应用领域等如下表所示：
   https://blog.csdn.net/weixin_38353277/article/details/128728993

![相机三维视觉方案对比](.\asset\相机三维视觉方案对比.png)

![相机三维视觉方案对比2](.\asset\相机三维视觉方案对比2.png)

![相机三维视觉方案对比3](.\asset\相机三维视觉方案对比3.png)

目前测量方案有主动，被动之分，被动就是采用可见光，好处是不需要额外光源，但是晚上无法使用，主动就是主动发射红外激光做补光，这样晚上也能使用。

1. 结构光适合近距离的测量方案；

   散斑就是是激光照射到粗糙物体或穿透毛玻璃后随机形成的衍射斑点。这些散斑具有高度的随机性，而且会随着距离的不同而变换图案。也就是说空间中任意两处的散斑图案都是不同的。只要在空间中打上这样的结构光，整个空间就都被做了标记，把一个物体放进这个空间，只要看看物体上面的散斑图案，就可以知道这个物体在什么位置了。当然，在这之前要把整个空间的散斑图案都记录下来，所以要先做一次光源标定， 通过对比标定平面的光斑分布，就能精确计算出当前物体距离相机的距离。

   特定波长的Laser 发出的结构光照射在物体表面，其反射的光线被带滤波的camera 相机接收，滤波片保证只有该波长的光线能为camera 所接受。Asic 芯片对接收到的光斑图像进行运算，得出物体的深度数据。

2. 双目视觉仅仅依靠图像进行特征匹配，对附加设备要求低，在使用双目视觉相机前必须对双目中两个摄像头的位置进行精确标定。

3. TOF 并非基于特征匹配，这样在测试距离变远时，精度也不会下降很快，目前无人驾驶以及一些高端的消费类Lidar 基本都是采用该方法来实现。

### 2理论参考

手眼标定一般分为二维（平面）标定、三维（空间）标定。

二维标定

* 常用的是9点标定

三维标定就需要用到深度相机（RGBD相机）。

* 手眼标定算法Tsai-Lenz（easy hand-eye用的）
* 张正友标定方法：棋盘格标定
  * Zhang, Zhengyou. "A flexible new technique for camera calibration." *IEEE Transactions on pattern analysis and machine intelligence* 22.11 (2000): 1330-1334.
  * opencv  matlab
* 原理：

![在这里插入图片描述](https://img-blog.csdnimg.cn/54e98589e74f4b60b2c833fbe253fc5f.png)

B未知。
C也就是我们前面提到的外参的逆矩阵
D是我们最终要求的。
总的来说也就是A、C都是可以知道的。





### 3任务解析

1. 需求：

   1. 相机标定：相机标定主要是为了获得相机的内参矩阵K和畸变参数(k1,k2,k3,p1,p2)
      1. 彩色图和深度图之间还需要标定吗？
      2. 实际上kinect的sdk已经提供了内参矩阵处理？（直接就能从点云订阅到对于深度相机来说的世界坐标）
   2. 手眼标定：准确地确定机器人的手部和视觉系统之间的几何关系。
      1. 眼在手外，eye-to-hand：也即摄像头安装在手臂之外的部分，与机器人的基座（世界坐标系）相对固定，不随着机械臂的运动而运动；
      2. 需要：将深度相机里的世界坐标和机械臂的末端坐标进行标定
         1. 建立整个模型？（机械臂和相机）

2. 具体实现

   1. 相机标定：

      参考：https://blog.csdn.net/qq_41253960/article/details/124928140

      标定材料：https://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check-108.pdf





### 4相关工具学习和测试

二维：

**camera_calibrator**

https://wiki.ros.org/camera_calibration

ros自带：`cameracalibrator.py` subscribes to ROS raw image topics, and presents a calibration window

* the current images from the cameras, highlighting the checkerboard

* run in： monocular and stereo mode



三维：

**easy-hand-eye 流程**

https://github.com/IFL-CAMP/easy_handeye/tree/master

- **sample** the robot position and tracking system output via `tf`,
- **compute** the eye-on-base or eye-in-hand calibration matrix through the OpenCV library's Tsai-Lenz algorithm implementation,
- **store** the result of the calibration,
- **publish** the result of the calibration procedure as a `tf` transform at each subsequent system startup,



直接尝试opencv的函数







---

## 10.18 沟通交流

### 下一步计划

1. 学习和跑通机械臂（ur5驱动 了解tinker模型），熟练moveit；要在物理环境里尝试
2. 需要自己复习
3. 尝试在ROS2做标定（问题：除easy hand-eye之外的包？直接自己计算吗？）
   * 建议：需要自己写一遍内外参矩阵；需要弄明白整个理论过程

### 心得：

1. 单独交流一定要妥善准备学习结果（同时准备好状态：用词和思路要清晰且专业）

2. 单独交流用时：30min左右
3. 最好是按照规律按时交流，否则会破坏多线的可持续性；同时和其他同学一起反而能进入讨论的氛围，同时能在老师状态比较好的时候交流
4. 平时一定要及时记录学习日志，过程记录增强意义感（一开始先顺序记录）+同时也能从和别人讲的角度思考问题

5. 反思科研和技术学习的方式，不同于课程学习；一定要有清晰的思路

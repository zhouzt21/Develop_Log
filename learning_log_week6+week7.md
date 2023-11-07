# 学习日志记录

# week 6-7

## task1 部署tinker urdf至SAPIEN

* 模型和位姿：
  * 转模型格式：使用tinkerbase中的description 用ros2指令转macro为urdf（也即将宏定义转换成普通的urdf文件）
  * 导入模型：直接Load模型就可以使用
  * 加上夹爪：现有应该是ur六自由度，robotiq是八自由度
  * 现有的模型使用的是tinker_base_coarse（scale :1 1 1）和直接导入的robotiq + ur5  + wheel
  * 问题：
    * 不确定tinker_base_coarse和轮子、ur机械臂的尺寸匹配程度
    * 了解robotiq自由度分别的指代？？
    * 重新生成的urdf是基于real模型之上的，故有base_inertial，加上了惯性描述
* 力设置 
  * 外力设置的问题:
    * external force is deprecated and ignored in passive force computation.
* 碰撞模型



### 补充知识：

不同模型文件之间的区别：.stl   .dae .urdf  .xacro

.urdf模型文件编写结构：http://wiki.ros.org/urdf/XML （**详细见专题学习日志**）

.dae: DAE（Digital Asset Exchange）文件格式是**3D交互文件格式**，一般用于多个图形程序之间交换数字数据， DAE是一种3D模型，可被flash 导入。 

.stl:  二进制STL文件用固定的字节数来给出三角面片的几何信息。ASCII码格式的STL文件逐行给出三角面片的几何信息



### 查询文档：

1. ur5: 

The robot itself is an arm composed of extruded aluminum tubes and joints. The
joints are named A:Base, B:Shoulder, C:Elbow and D,E,F:Wrist 1,2,3. The Base is
where the robot is mounted, and at the other end (Wrist 3) the tool of the robot
is attached. By coordinating the motion of each of the joints, the robot can
move its tool around freely, with the exception of the area directly above and
directly below the robot, and of course limited by the reach of the robot (850mm
from the center of the base).

动力学：

两自由度机械臂公式（动力学要求不高, 主要以轨迹为主

2. robotiq 2f 140

https://blog.51cto.com/u_15302822/5688118#ur__robotiq_gripper__robotiq_ft_sensor__gazebo____httpsblogcsdnnetbornfree5511articledetails107529420bug_7

到底初始位姿的维度是多少？
？？？？







## task2 论文阅读

#### Close the Optical Sensing Domain Gap by Physics-Grounded Active Stereo Sensor Simulation 深度传感器仿真

IEEE Transactions on Robotics (T-RO)

1University of California, San Diego, 2Tsinghua University

Off-the-Shelf Real-Time Sensor Simulation  （ real time 60+ FPS ）

1. pipeline:  

   **material acquisition**, ray-tracingbased infrared (IR) image rendering, IR noise simulation, and depth estimation.

2. contribution:

* We propose a multispectral loss function to acquire object material parameters, which include visual appearance loss and neural network based perceptual loss to help eliminate the mismatch in brightness and color in both visible spectrum and infrared spectrum. 
* Our second highlighted innovation is in adding textured light support. Existing ray tracing rendering systems usually do NOT support the rendering of textured lights.（textured light ?）
  * In order to achieve real-time simulation of the depth sensor, we further integrate learning-based denoisers into the renderer such that we can generate high-fidelity IR images with hundreds of FPS using a small number of samples per pixel (SPP). 
  * We build a GPU-accelerated stereo matching module consists of common algorithms in real-world depth sensors. （achieve 200+ FPS under common settings while usual CPU implementations can hardly achieve 1 FPS.）



**active stereovision depth sensor simulation （realsense d415）**

reason:

1. Depth map is more suitable for robotic tasks
2. Depth maps do not involve color information, which is extremely challenging to align between the simulator and the real world.
3. Active stereovision depth sensor has wide adoption：have relatively high accuracy, high spatial resolution, low cost, and robustness to lighting conditions（17
4. Active stereovision depth sensor simulation requires the estimation of fewer parameters ： work at a specific wavelength, mostly infrared
5. Real-time realistic ray tracing techniques have matured just recently
   1. Active stereovision sensors use IR lights to probe the environment and form stereo image pairs.

*  To generate simulation data with a relatively low domain gap, the simulation process must also create material-dependent error patterns similar to real sensors

  

3. 前置：

**A. Active Stereovision Depth Sensor**

depth sensors： passive stereovision, **active stereovision**, structured light, and Time-of-Flight (ToF)

* active stereovision depth sensor ：an IR pattern projector, two IR cameras, and an RGB camera.
  * The projector casts an IR pattern (typically random dots) onto the scene. The two IR cameras capture the scene with the reflected pattern in the IR spectra. The depth map is computed by stereo matching on the two captured IR images.
  * Compared with passive stereovision, active stereovision can measure texture-less areas. 
  * Compared with structured light which uses coded pattern(s), active stereovision is unaffected when multiple devices measure the same area simultaneously. 
  * Compared with ToF, active stereovision has higher spatial resolution and is not affected by multi-path interference.

**B. Physically Based Rendering**

A PBR pipeline seeks to model both the light transport process and the surface material in a physically correct way.

* Light transport process. Many rendering techniques have been proposed to model the light transport process, such as ray casting, recursive path tracing, and photon mapping. These methods are covered by an umbrella term **ray tracing**.
  * ray tracing algorithm calculates the color for each pixel by tracing a path from the camera and accumulates the material-dependent weight along the path to light sources.
  * cutting-edge acceleration approaches： deep-learning-based denoising， superresolution

* Surface material modeling.
  * Bidirectional Scattering Distribution Function双向散射分布函数：describes how light scatters from a surface.
    * However, researchers have proposed many parameterized models to approximate the function.   Among all these efforts, the Disney™ PBR material model

* 仿真环境和真实世界中的深度图建立方式一模一样，才能减少sim2real的gap



#### Maniskill2: A unified benchmark for generalizable manipulation skills 可泛化操作



don't have to collect assets or design tasks by yourself, and can focus on algorithms

* 关键在于更好的使用计算资源（对于深度学习来说，训练速度是很重要的部分）



#### ActiveZero++: Mixed Domain Learning Stereo and Confidence-based Depth Completion with Zero Annotation 深度学习双目重建混合学习框架

* 



#### Part-Guided 3D RL for Sim2Real Articulated Object Manipulation 铰接物体Sim2real操作





-----

词汇：

domain gap==＞在一个数据集上训练好的模型无法应用在另一个数据集上

To the best of our knowledge,  尽我们所知

I would like to note that previous answers made many assumptions about the user's knowledge. This answer attempts to answer the question at a more tutorial level.  

hetergenous 异质的

deform 变形  Peg-in-hole  方枘圆凿 

disparity 差距

---



## task3 理论推进

《机器人学》尚未抽出时间推进。



---

### 旁支：

1. RTU(Remote Terminal Unit)是一种远端测控单元装置，负责对现场信号、工业设备的监测和控制。

2. ament_cmake用户文档

   ament_cmake是ROS 2中基于CMake的软件包的构建系统（特别是C/C ++项目，即使不是全部，也是大多数都使用ament_cmake）。 它是一组脚本，用于增强CMake，并且为软件包的作者增加一些便利功能。了解[CMake](https://link.zhihu.com/?target=https%3A//cmake.org/cmake/help/v3.5/)的基础知识会非常有帮助，其官方教程可以在[这里](https://link.zhihu.com/?target=https%3A//cmake.org/cmake/help/latest/guide/tutorial/index.html)找到。



---

### log反思：

1. 记录流程：任务（立项 分析 方案确定）+ 操作步骤与结果/问题 + 中途补充学习知识 + 反思优化

学习心得：

1. 需要抽象提取和总结需要完成的任务是什么，查找才可能方便
2. 在遇到关于ros1/2相对应的库的问题，应当直接查看官方文档，而不是在网页上找半天技术帖



----

11.6 沟通交流

项目方案：使用大模型做一些移动抓取的sim2real；（需要用上SLAM

下一步计划：

1. 加上机械臂控制器
2. 需要加上雷达（最好能加上tinker的）  （相机可以加上realsense）
3. 看一下tidybot论文 研究一下
4. 需要补充SLAM的理论知识学习



心得：

1. 作了东西就要好好讲，做好认真讲解的基本准备
2. 任务需要均匀完成，不要堆在周末来做，集中处理学习和lab任务的策略不够科学，集中处理应该指以一天为单位；在一周之内应该交叉进行
3. 保证与大家沟通才有可能对自己做的事情有清楚的认识。
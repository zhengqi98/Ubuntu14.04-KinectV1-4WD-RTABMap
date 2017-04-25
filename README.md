# Ubuntu14.04-KinectV1-4WD-RTABMap
<h1 align="left">
	<img width="400" src="https://github.com/xzq-forever/Ubuntu14.04-KinectV1-4WD-RTABMap/blob/master/test1.png" alt="test1">
	<br>
	<br>
</h1>


环境：Ubuntu14.04（64位）+KinectV1+四轮差速驱动小车     注:以下步骤在小车上位机上进行、不适用于KinectV2</br>
## 目录
*   [安装ROS](#安装ROS)
*   [安装KinectV1驱动](#安装KinectV1驱动)
*   [伪造激光数据](#伪造激光数据)
    *   [创建ROS工作空间](#创建ROS工作空间)
    *   [编译depthimage_to_laserscan](#编译depthimage_to_laserscan)
*   [合并Kinect启动文件](#合并Kinect启动文件)
*   [安装RTAB-Map](#安装RTAB-Map)
*   [ROS与底层通信](#ROS与底层通信)
*   [导航](#导航)
*   [远程操控](#远程操控)
    *   [在小车上位机上](#在小车上位机上)
    *   [在远程操控机器上](#在远程操控机器上)
*   [相关资料](#相关资料)
*   [TODO](#TODO)

<h2 id="安装ROS">安装ROS</h2>

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall
```
参考:http://wiki.ros.org/indigo/Installation/Ubuntu</br>

<h2 id="安装KinectV1驱动">安装KinectV1驱动</h2>

```
sudo apt-get install ros-indigo-openni-* ros-indigo-openni2-* ros-indigo-freenect-*
```
测试:</br>
```
roslaunch freenect_launch freenect.launch depth_registration:=true
```
参考:http://www.ncnynl.com/archives/201609/794.html</br>

<h2 id="伪造激光数据">伪造激光数据</h2>
<h4 id="创建ROS工作空间">创建ROS工作空间</h4>

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd .. && catkin_make
echo "source ~/catkin_ws/src/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
参考:http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment</br>
<h4 id="编译depthimage_to_laserscan">编译depthimage_to_laserscan</h4>

```
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/depthimage_to_laserscan
cd .. && catkin_make
```
参考:http://wiki.ros.org/depthimage_to_laserscan</br>

<h2 id="合并Kinect启动文件">合并Kinect启动文件</h2>

```
sudo apt-get install nano
mkdir ~/launch && cd ~/launch
nano scan.launch
```
```
<launch>
  <!-- Kinect cloud to laser scan -->
    <node pkg="depthimage_to_laserscan" type="depthimage_to_laserscan" name="depthimage_to_laserscan">
      <remap from="image"     to="/camera/depth_registered/image_raw"/>
      <remap from="camera_info" to="/camera/depth_registered/camera_info"/>
      <remap from="scan" to="/scan"/>
      <param name="range_max" type="double" value="4"/>
    </node>
    <include file="$(find freenect_launch)/launch/freenect.launch">
       <arg name="depth_registration"         value="true"/>
    </include>
</launch>
```
测试:</br>
```
roslaunch ~/launch/scan.launch
```
<h2 id="安装RTAB-Map">安装RTAB-Map</h2>

```
sudo apt-get install ros-indigo-rtabmap-*
roscd rtabmap_ros/launch
sudo nano rgbd_mapping.launch
```
```
32   <arg name="subscribe_scan"          default="false"/>
```
改为
```
32   <arg name="subscribe_scan"          default="true"/>
```
```
测试:roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start" rtabmapviz:=false
```
注:需要在启动kinect的前提下才能使用</br>

<h2 id="ROS与底层通信">ROS与底层通信</h2>

注：假设下位机已经烧录好程序</br>

```
cd ~/catkin_ws/src
git clone https://github.com/grassjelly/linorobot_4wd.git
git clone https://github.com/grassjelly/lino_pid.git
git clone https://github.com/grassjelly/lino_msgs.git
git clone https://github.com/hbrobotics/ros_arduino_bridge
cd .. && catkin_make
cd ~/launch
nano arduino.launch
```
```
<node name="arduino" pkg="ros_arduino_python" type="arduino_node.py" output="screen">
    <rosparam file="$(find ros_arduino_python)/config/my_arduino_params.yaml" command="load" />
</node>
```
```
roscd ros_arduino_python/config
sudo cp arduino_params.yaml my_arduino_params.yaml
sudo chmod 777 /dev/ttyUSB0
roslaunch ~/launch/arduino.launch
```
用键盘测试:</br>
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
调参:</br>
arduino_params.yaml是基础的参数配置文件，如轮胎直径、轮胎间距等</br>
参考:https://linorobot.org</br>

<h2 id="导航">导航</h2>

```
roslaunch linorobot_4wd navigate.launch
```
调参:</br>
```
cd ~/catkin_ws/src/linorobot_4wd/param && ls
```
里面的五个文件都是导航相关的配置文件，参数说明可参考:</br>
http://www.cnblogs.com/zjiaxing/p/5559382.html</br>
http://wiki.ros.org/navigation/Tutorials/RobotSetup</br>

<h2 id="远程操控">远程操控</h2>

<h4 id="在小车上位机上">在小车上位机上</h4>

ifconfig查看小车上位机(NUCi7)ip地址，假设为10.18.241.138</br>
在每个终端需要</br>
```
export ROS_IP=10.18.241.138&&export ROS_MASTER_URI=http://10.18.241.138:11311/
```
```
终端1:roslaunch ~/launch/scan.launch
终端2:sudo chmod 777 /dev/ttyUSB0
roslaunch ~/launch/arduino.launch
终端3:roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start" rtabmapviz:=false
终端4:roslaunch linorobot_4wd navigate.launch
```

<h4 id="在远程操控机器上">在远程操控机器上</h4>

先进行第一步安装ROS，然后安装rviz</br>
```
sudo apt-get install ros-indigo-rviz-*
```
ifconfig查看本机ip地址，假设为10.18.228.187</br>
在每个终端需要</br>
```
export ROS_IP=10.18.228.187 && export ROS_MASTER_URI=http://10.18.241.138:11311/
```
```
终端1:rosrun rviz rviz
```
接下来在rviz订阅相关话题就可以了</br>

<h2 id="相关资料">相关资料</h2>

- [ROS入门正确姿势](https://zhuanlan.zhihu.com/p/24903381)
- [ROS官网rviz介绍](http://wiki.ros.org/rviz/)
- [ROS官网rtabmap_ros介绍](http://wiki.ros.org/rtabmap_ros)
- [RTAB-Map官网介绍](https://introlab.github.io/rtabmap)
- [RTAB-Map原理中文解析](http://blog.csdn.net/u012700322/article/details/51898726)
- [rtabmap_ros+KinectV1使用](http://www.rosclub.cn/thread-25.html)
- [ROS官网Navigation介绍](https://github.com/sindresorhus/awesome-electron)
- [Navigation解析](http://blog.exbot.net/archives/1129)
- [linorobot介绍](https://linorobot.org)
- [搭建ROS小车底盘A](http://www.ncnynl.com/archives/201612/1203.html)
- [搭建ROS小车底盘B](http://www.ncnynl.com/archives/201703/1414.html)
- [小车校准](http://www.rosclub.cn/post-113.html)
- [ROS与SLAM教程](http://www.ncnynl.com/archives/201701/1231.html)
- [ROS与深度摄像头教程](http://www.ncnynl.com/archives/201703/1436.html)
- [Turtlebot入门教程](http://www.ncnynl.com/archives/201609/787.html)
- [SLAM相关汇总](http://www.slamcn.org)
- [OpenSLAM](http://www.openslam.org)

<h2 id="TODO">TODO</h2>

- [x] 初步构建文档
- [ ] 设定TF转换关系
- [ ] KinectV1->KinectV2
- [ ] NUCi7
- [ ] 4WD->Mecanum
- [ ] 二维+三维显示
- [ ] 里程计融合
- [ ] 看各文件源码

PS:由于我只是单凭回忆记录的，并不是边实验边记录，所以可能有所纰漏。</br>

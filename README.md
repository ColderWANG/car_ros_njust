### 基于激光视觉融合的机器人语义slam自主探索

## 依赖包
系统ubuntu 20.04 LTS,ubuntu 18.04LTS，最好用20因为其支持 python3，18需要另外安装 python3 的cvbridge
安装顺序,先用 fishros 安装ros系统
1. boost-1_71 (ubuntu 20自带) 以及lego官方git的指定依赖包
2. 安装前置包
   >安装x11<br/>
   sudo apt-get install libx11-dev libxext-dev libxtst-dev libxrender-dev libxmu-dev libxmuu-dev<br/>
   安装opengl<br/>
   sudo apt-get install build-essential libgl1-mesa-dev libglu1-mesa-dev<br/>
   sudo apt-get install freeglut3-dev<br/>
   pcl前置<br/>
   sudo apt-get install git build-essential linux-libc-dev<br/>
   sudo apt-get install cmake cmake-gui<br/>
   sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev<br/>
   sudo apt-get install mpi-default-dev openmpi-bin openmpi-common<br/>
   sudo apt-get install libflann1.9 libflann-dev<br/>
   sudo apt-get install libqhull* libgtest-dev<br/>
   sudo apt-get install mono-complete<br/>
   sudo apt-get install libopenni-dev<br/>
   sudo apt-get install libopenni2-dev<br/>
   包前置<br/>
   sudo apt-get install ros-noetic-ackermann-*<br/>
   sudo apt-get install ros-noetic-serial<br/>
   标定包前置<br/>
   sudo apt-get install libgtk2.0-dev pkg-config<br/>
3. eigen-3.3.4
   >cd eigen-3.3.4<br/>
   mkdir build && cd build<br/>
   cmake ..<br/>
   sudo make install<br/>
4. vtk-8.2.0
   >以VTK库示例，未做特殊说明均按照如下方式编译安装,默认安装位置在/usr/local<br/>
   cd VTK-8.2.0<br/>
   mkdir build && cd build<br/>
   cmake ..<br/>
   make -j8<br/>
   sudo make install<br/>
5. pcl-pcl-1.9.1
   > make 用 -j1 ,否则容易报错
6. 新建ros工作空间编译perception_pcl 1.7.3版本
   >mkdir -p pcl_ws/src<br/>
   cd pcl_ws/src/<br/>
   catkin_init_workspace<br/>
   别用 git clone ，否则下载的是最新版,去 github 下载 1.7.3 的zip，移动 pcl_ros,pcl_conversions 到工作空间下编译<br/>
   su<br/>
   source /opt/ros/noetic/setup.bash<br/>
   catkin_make install --cmake-args -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic （这里改成自己系统版本） <br/>


7. gtsam
8. opencv-3.3.4
9.  python3 安装openvino


### 编译安装
进入文件夹catkin_make -j1


## rslidar_sdk注意
1. 子模块 rs_driver 需单独下载或git submodule update
2. 需要改配置文件conf.yaml

## 相机标定以及雷达相机联合标定
1. 相机 usb_cam 一定要从 github 上下载自己编译！！！！，ros-noetic-usb-cam 无法进行后续的标定。
2. sudo apt-get install ros-noetic-camera-calibration 进行标定。

按照 https://github.com/Aaron20127/Camera-lidar-joint-calibration 的方法进行标定。
在 /data 文件夹下有图片提取脚本以及点云提取命令
运行 pcl_ros 命令行要用 ros-noetic-pcl_ros 的
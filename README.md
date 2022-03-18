### 基于激光视觉融合的机器人语义slam自主探索

## 依赖包
系统ubuntu 20.04 LTS,ubuntu 18.04LTS，最好用20因为其支持 python3，18需要另外安装 python3 的cvbridge
安装顺序,先用 fishros 安装ros系统
1. boost-1_71 (ubuntu 20自带) 以及lego官方git的指定依赖包
2. 安装前置包
   >安装x11
   sudo apt-get install libx11-dev libxext-dev libxtst-dev libxrender-dev libxmu-dev libxmuu-dev
   安装opengl
   sudo apt-get install build-essential libgl1-mesa-dev libglu1-mesa-dev
   sudo apt-get install freeglut3-dev
   pcl前置
   sudo apt-get install git build-essential linux-libc-dev
   sudo apt-get install cmake cmake-gui
   sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
   sudo apt-get install mpi-default-dev openmpi-bin openmpi-common
   sudo apt-get install libflann1.9 libflann-dev
   sudo apt-get install libqhull* libgtest-dev  
   sudo apt-get install mono-complete
   sudo apt-get install libopenni-dev   
   sudo apt-get install libopenni2-dev
   包前置
   sudo apt-get install ros-noetic-ackermann-*
   sudo apt-get install ros-noetic-serial
3. eigen-3.3.4
   >cd eigen-3.3.4
   mkdir build && cd build
   cmake ..
   sudo make install
4. vtk-8.2.0
   >以VTK库示例，未做特殊说明均按照如下方式编译安装,默认安装位置在/usr/local
   cd VTK-8.2.0
   mkdir build && cd build
   cmake ..
   make -j8
   sudo make install
5. pcl-pcl-1.9.1
   > make 用 -j1 ,否则容易报错
6. 新建ros工作空间编译perception_pcl 1.7.3版本
   >mkdir -p pcl_ws/src
   cd pcl_ws/src/
   catkin_init_workspace
   别用 git clone ，否则下载的是最新版,去 github 下载 1.7.3 的zip，移动 pcl_ros,pcl_conversions 到工作空间下编译
   su
   source /opt/ros/noetic/setup.bash
   catkin_make install --cmake-args -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic （这里改成自己系统版本） 


7. gtsam
8. opencv-3.3.4
9.  python3 安装openvino


### 编译安装
进入文件夹catkin_make -j1
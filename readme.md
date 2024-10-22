# 如何使用本仓库
```bash
# 下载本仓库
git clone https://github.com/shuaihuafei/ros_boat_ws.git
# 下载本仓库中的子仓库
git submodule update --init --recursive

# 或者直接使用下面的指令克隆主仓库和子仓库
git clone --recurse-submodules https://github.com/shuaihuafei/ros_boat_ws.git

# 注：在主仓库中添加别人的仓库为子仓库的指令为
git submodule add <子仓库地址>
```
# 环境配置
## 基础环境
1. ubuntu20.04并安装ros的noetic版本
2. 在用户家目录下载本仓库及其子仓库
   ```bash
   git clone --recurse-submodules https://github.com/shuaihuafei/ros_boat_ws.git
   ```
## 深度学习环境配置
1. 安装适配当前电脑的显卡驱动
2. 安装cuda11.1和cudnn，并配置环境变量
3. 创建conda虚拟环境，python=3.8，torch=1.8.0
4. 下载yolov11模型文件到src/cv/ultralytics目录下
## 相机
参考：相机ros的[github地址](https://github.com/orbbec/OrbbecSDK_ROS1)  
1. 安装依赖
    ```bash
    sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs libdw-dev
    ```
2. 安装usb检测驱动
    ```bash
    sudo bash src/OrbbecSDK_ROS1/scripts/install_udev_rules.sh
    ```
3. 编译
    ```bash
    catkin build
    ```
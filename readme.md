# 如何下载本仓库
```bash
## 方法一：
# 1.先下载主仓库
git clone https://github.com/shuaihuafei/ros_boat_ws.git
# 2.下载主仓库中的子仓库
git submodule update --init --recursive

## 方法二：
# 或者直接使用下面的指令克隆主仓库和子仓库
git clone --recurse-submodules https://github.com/shuaihuafei/ros_boat_ws.git

## 注：在主仓库中添加别人的仓库为子仓库的指令为
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
    ```bash
    # 创建Python虚拟环境
    conda create -n yolov11-ros python=3.8
    conda activate yolov11-ros
    # 安装Pytorch
    pip install torch==1.8.0+cu111 torchvision==0.9.0+cu111 torchaudio==0.8.0 -f <https://download.pytorch.org/whl/torch_stable.html>
    ```
4. 安装yolov11库
    ```bash
    pip install ultralytics
    ```
5. 下载yolov11模型文件到src/cv/ultralytics目录下
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
## 编译
1. 切换到刚定义的虚拟环境中再编译
    ```bash
    conda activate yolov11-ros
    catkin_make
    ```
2. 编译过程中会报错，根据报错信息，分别安装，然后再编译
    ```bash
    pip install empy==3.3.4
    pip install catkin_pkg
    ```
3. 运行时根据报错信息，缺失'rospkg'包，安装
    ```bash
    pip install rospkg
    ```
4. 运行时报错库冲突，删除冲突库
    ```bash
    rm ${CONDA_PREFIX}/lib/libffi.7.so
    rm ${CONDA_PREFIX}/lib/libffi.so.7
    ```
## 加入Qt后的编译与运行
1. cd到当前功能包下`cd ~/ros_boat_ws`
2. 编译并运行`catkin build && ./devel/lib/app_interface/app_interface`即可打开Qt界面
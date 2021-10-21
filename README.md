# TStoneCalibration
计算机视觉标定工具箱

计算机视觉标定工具箱项目由香港中文大学天石机器人研究所嵌入式AI组发起，目标是做一款可以满足各种视觉设备标定的工具箱。

目前已开发的标定工具有：
* 光学相机内参标定工具

## 编译
项目基于`Qt`和`OpenCV 4`，在Ubuntu 20.04下开发。
* 安装Qt
```bash
sudo apt-get install qt5-default
```
* 安装OpenCV 4
```bash
sudo apt-get install libopencv-core4.2
```

也可以使用OpenCV 3，里面有一些OpenCV的宏定义需要修改一下。

* 编译
```bash
mkdir build
cd build
cmake ..
make
```

## 下载
Release页面有最新的Windows程序下载。


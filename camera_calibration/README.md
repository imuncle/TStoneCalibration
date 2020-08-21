# camera_calibration
传统相机和鱼眼相机标定程序

基于Qt和OpenCV开发，角点检测部分为MATLAB的角点检测算法的C++实现，并优化实现了多棋盘检测和大畸变鱼眼镜头下棋盘的检测。

## 功能列表：
* 角点检测，支持多棋盘检测（代码逻辑见[基于生长的棋盘格角点检测](https://github.com/imuncle/imuncle.github.io/issues/113)）
* 单目传统相机标定，采用张正友标定法，支持模式选择，用户可选择径向畸变参数的个数和是否标定切向畸变
* 单目鱼眼相机标定
* 双目传统/鱼眼相机标定，无需预先单独标定单个相机
* 支持打开摄像头采集图片
* 支持畸变矫正预览和重投影预览
* 支持导出畸变矫正后的图片

## 文件结构
|文件名/文件夹名|功能|
|:--|:--|
|CameraCalibration.cpp|程序主界面和主要逻辑的实现|
|findCorner.cpp|棋盘角点检测算法的实现|
|chessboard.cpp|根据检测到的角点进行棋盘生长|
|sigle_capture.cpp|单个相机采集图像的实现|
|double_capture.cpp|双目相机采集图片的实现|
|choose_two_dir.cpp|立体标定图像文件夹选择对话框|
|res|图标资源文件夹|

## 软件运行截图：

![screenshot](screenshot.jpg)
# Camera_Lidar_Calibration
It's hard to choose accurate pointcloud from pcd files. I reckon that manual adjustment has better performance. This is a tool that help you try this method. It can project pointcloud from lidar to camera real time.


### 相机和激光雷达联合标定实时调参工具

网上开源的各种标定工具普遍有流程繁琐，无标定物标定效果差的问题，经过一个多月的尝试，最终选定用开源工具进行粗标定，然后使用手动调整变换矩阵使得点云与图片对齐的方法

粗标定可以使用：https://github.com/koide3/direct_visual_lidar_calibration

需要保存一个点云PCD文件和对应的图片，在projectCloud.cpp文件中填入相机内参和畸变系数

本代码写于2023年11月，年久失修，本人也忘记怎么跑了，只是提供一个思路。

标定效果图：  

![效果](https://github.com/SZURPVision/Camera_Lidar_Calibration/assets/89376165/66776d88-37af-4809-bf3e-f99519c9b5cd)


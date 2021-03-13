# Intensity-SLAM 
## Intensity-Assisted Simultaneous Localization And Mapping

This is an implementation of paper "Intensity-SLAM: Intensity Assisted Localization and Mapping for Large Scale Environment" [paper](https://arxiv.org/pdf/2102.03798.pdf)

[Wang Han](http://wanghan.pro), Nanyang Technological University, Singapore

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.

ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

### 1.4. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-melodic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

## 2. Build 
### 2.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/wh200720041/intensity_slam.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
### 2.2 Download test rosbag
Download [KITTI sequence 05](https://drive.google.com/file/d/1eyO0Io3lX2z-yYsfGHawMKZa5Z0uYJ0W/view?usp=sharing) or [KITTI sequence 07](https://drive.google.com/file/d/1_qUfwUw88rEKitUpt1kjswv7Cv4GPs0b/view?usp=sharing)

Unzip compressed file 2011_09_30_0018.zip. If your system does not have unzip. please install unzip by 
```
sudo apt-get install unzip 
```

And this may take a few minutes to unzip the file
```
	cd ~/Downloads
	unzip ~/Downloads/2011_09_30_0018.zip
```

### 3.3 Launch ROS
```
    roslaunch intensity_slam intensity_slam.launch
```

## 4.Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).

## 5. Citation
If you use this work for your research, you may want to cite
```
@article{wang2021intensity,
  author={H. {Wang} and C. {Wang} and L. {Xie}},
  journal={IEEE Robotics and Automation Letters}, 
  title={Intensity-SLAM: Intensity Assisted Localization and Mapping for Large Scale Environment}, 
  year={2021},
  volume={6},
  number={2},
  pages={1715-1721},
  doi={10.1109/LRA.2021.3059567}
}
```

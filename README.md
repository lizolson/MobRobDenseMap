# Sparse Maps and CNN-built Point Clouds Fusion for High-Fidelity 3D Map Generation

## Installation of VINS-Fusion
## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).


## 2. Build VINS-Fusion
Clone the repository and catkin_make:
```
    git clone https://github.com/lizolson1/MobRobDenseMap.git
    cd [Target_Dir]/VINS/
    catkin_make
    source /devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3 KITTI GPS Fusion (Stereo + GPS)
Download [KITTI raw dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) to YOUR_DATASET_FOLDER. Take [2011_10_03_drive_0027_synced](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip) for example.
Open four terminals, run rviz, global fusion, PCListerner.py (A subscriber that gathers point cloud for processes in the downstream of our pipeline) and vins respectively. Note that you have source /devel/setup.bash for every single one of them.
Green path is VIO odometry; blue path is odometry under GPS global fusion.
```
    roslaunch vins vins_rviz.launch
    rosrun global_fusion global_fusion_node
    python PCListener.py
    rosrun vins kitti_gps_test ~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml YOUR_DATASET_FOLDER/2011_10_03_drive_0027_sync/ 
```
Press Ctrl+C to exit PCListener.py to get two csv files: GPS_VIO_WGPS_T_WVIO.csv and VF_pointcloud_expanded.csv. The former file contains the vehicle's local pose and global pose at each timestamp as well as the transformation(translation+rotation) between them. The latter file contains the point cloud's locations and pixel coordinates and the corresponding timestamps and other information such as the vehicle's pose at that moment.
<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti.gif" width = 430 height = 240 />


## 4. Acknowledgements
VINS-Fusion is developed by [HKUST-Aerial-Robotics Group](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion). The VINS-Fusion is released under [GPLv3](http://www.gnu.org/licenses/) license.


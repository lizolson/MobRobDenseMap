# Sparse Maps and CNN-built Point Clouds Fusion for High-Fidelity 3D Map Generation

## Overview
We highlight the prerequisites in Section 1. We have forked the VINS-FUSION repo, and modified it to publish messages necessary for our project. We explain how to use our repo for VINS-FUSION in Section II. This step can be bypassed in testing, as examples of its outputted files are included. Section III explains how to pull in the KITTI data, but the first 100 sample frames are included in 'left/' and 'right/'. In Section IV, we explain how we used the DispNet docker image for our project, as well as the supplementary files we made. This section may also be bypassed, as we include the disparity maps of these 100 sample images. Additionally, in Section V we explain how to generate the point clouds. In Section VI, we show how to align them based on odometry data, perform ICP, and run RANSAN to transform them in the global frame. Section VII lists our acknowledgements. **For simplest testing, we recommend running the provided first 100 images from a KITTI dataset, and using the provided outputs from Sections II and IV, so you can start at Section V. 

## 1. Prerequisites 

### 1.1 **Ubuntu** 
Ubuntu 64-bit 16.04 or 18.04.


### **ROS**
Only needed for Section II
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Ceres Solver**
Only needed for Section II
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3 **OpenCV**
Needed for Sections IV and V
https://pypi.org/project/opencv-python/

### 1.4 **Open3D**
Needed for Sections IV, V, VI. 




## 2. Build VINS-Fusion
We modify this portion of our code from VINS-FUSION: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion 
Clone the repository and catkin_make:
```
    git clone https://github.com/lizolson1/MobRobDenseMap.git
    cd [Target_Dir]/VINS/
    catkin_make
    source /devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3. KITTI GPS Fusion (Stereo + GPS)

Download [KITTI raw dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) to YOUR_DATASET_FOLDER. Take [2011_10_03_drive_0027_synced](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip) for example.
Open four terminals, run rviz, global fusion, PCListerner.py (A subscriber that gathers point cloud for processes in the downstream of our pipeline) and vins respectively. Note that you have source /devel/setup.bash for every single one of them.
Green path is VIO odometry; blue path is odometry under GPS global fusion.
```
    roslaunch vins vins_rviz.launch
    rosrun global_fusion global_fusion_node
    python PCListener.py
    rosrun vins kitti_gps_test ~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml YOUR_DATASET_FOLDER/2011_10_03_drive_0027_sync/ 
```
Press Ctrl+C to exit PCListener.py to get two csv files: GPS_VIO_WGPS_T_WVIO.csv and VF_pointcloud_expanded.csv. The former file contains the vehicle's local pose and global pose at each timestamp as well as the transformation (translation+rotation) between them. The latter file contains the point cloud's locations and pixel coordinates and the corresponding timestamps and other information such as the vehicle's pose at that moment.

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti.gif" width = 430 height = 240 />

## 4. Disparity Estimation
To generate the disparity estimation, we provide code for creating disparity maps with Semi-Global Block Matching. This will create disparity maps in the PFM format. To run this: 
```
python sgm.py left/ right/ sgmmaps/ 
```

We also used DispNet to generate disparity maps: https://github.com/lmb-freiburg/dispnet-flownet-docker

We include a testfile generation script create the textfiles fed to DispNet for disparity estimation. To generate these lists for the first 100 images: 
```
python testfiles.py 50
```
To run DispNet on our dataset, we modified their docker script to run in bash mode, and map in our left and right images. The DispNet docker image must first be built using the DispNet-FlowNet Docker image (Steps 0, 1 on their repo). Our docker.sh script must be modified to map in the left and right image directories (changing the file paths to be the absolute path on your local machine). This step lines 149-150. Then run this docker with 
```
$ bash docker.sh -n DispNetCorr1D -v data/disparity-left-images.txt data/disparity-right-images.txt data/disparity-outputs.txt
```

From within the docker's bash mode, run: 
```
python demo.py /input-output/ /input-output/left.txt /input-output/right.txt /input-output/disp.txt 0
```
where 0 is the GPU device number you would like to use. 

## 5. Point Cloud Generation


## 6. Alignment, Refinement, and Global Frame Transformation

## 7. Acknowledgements
VINS-Fusion is developed by [HKUST-Aerial-Robotics Group](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion). The VINS-Fusion is released under [GPLv3](http://www.gnu.org/licenses/) license.


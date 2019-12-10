# ORB-SLAM With Absolute Scale Estimation and Correction

This project attempts to recover the absolute scale in the SLAM map produced by ORB-SLAM. This is based on the paper 
[Ground Plane based Absolute Scale Estimation for Monocular Visual Odometry](https://arxiv.org/pdf/1903.00912.pdf) and
[Reliable Scale Estimation and Correction for Monocular Visual Odometry](https://drive.google.com/file/d/0B73o7D_54u1LcFRPeWlIR1VubzA/view).

## Summary of the paper:  
To estimate scale, we use the assumption that the ground is planar. Then, between frame, the ground plane will be related by
a homography matrix. Firstly, we recover the homography matrix. Then, separately, we recover the relative pose between the two frames.
Using those two information, we can approximate the ground plane, represented using it's normal vector and the distance to the plane.
The paper then do another optimisation to refine the plane. Finally, the paper use kalman filter to on the estimated scale.

To correct the scale, the paper suggest the following:
- Correct the scale if all the following condition is fulfilled:
    - The estimated normal is close to the prior normal
    - Velocity reached a minimum threshold
    - Absolute value of (*Scale drift ratio* - 1) is more than 0.075
- We will now correct the current local map
    - All camera poses and map points are transformed to *local coordinate system*.
    - The local map points and the relative camera poses will be *re-scaled* by s.
    - All camera poses and map points are transformed back to global coordinate system.
    - Local BA is applied to refine the poses and points.
So far, scale estimation has been implemented, but most (if not all) refinement is yet to be done.  
Scale correction is still not very good. We have yet to find a way to inject the scale mid-operation. This is further discussed below.

## Other resources that might be helpful:
- [Author's implementation of the scale estimation](https://sites.google.com/site/dingfuzhou/projects/reliable-scale-estimation-and-correction-for-visual-odometry)
- [ORB-SLAM](https://zaguan.unizar.es/record/32799/files/texto_completo.pdf)
- [ORB-SLAM2](https://arxiv.org/pdf/1610.06475.pdf)
- [Appearance-Guided Monocular Omnidirectional Visual Odometry for Outdoor Ground Vehicles](http://rpg.ifi.uzh.ch/docs/IEEE_TransRobotics_scaramuzza.pdf)  
Part II D discuss more on the math behind decomposition on homography matrix.
- [ORB-SLAM: a Versatile and Accurate Monocular SLAM System](http://cseweb.ucsd.edu/~mkchandraker/classes/CSE291/2017/Presentations/07_Real_Time_SFM_Sudhanshu.pdf)  
This presentation is a good resources for ORB-SLAM

## Scale estimation result:
- This is tested by inserting the scale when we initialise the map. The estimated scale will be computed using mCurrentFrame 
and mLastFrame.  
This is using the recommendation from [this issue](https://github.com/raulmur/ORB_SLAM2/issues/478#issuecomment-471955661) on ORB-SLAM2.
- The result is quite good. However, as soon as turning is introduced, the result turns really bad (scale drift). Also, since 
no filter is applied, the estimated scale is not robust.

![](image/Ground%20Truth%20and%20Author's%20Code.png)
*Ground Truth and Estimated Scale on Author's Code, Sequence 04*  

![](image/Scale%20Estimation%20over%20Time.png)
*Estimated Scale in Current Implementation*

## Scale correction:
- The paper said to rescale every keyFrames and mapPoints in the "current local map".  
However, on further investigation, if the current local map mentioned is referring to the local map in the Tracking.cc, then the local map is not consecutive.
In between keyframe that is in local map, there can be keyframes that is not in local map. This makes the rescaling result 
weird, since not all consecutive keyframes are corrected.  
 **REVISION**: looks like most of the time it is ordered, but some frames might be missing.
Most of the time mLastKeyFrame is +1 of the largest index in local map
- Other definition that we can say about the local map is that it is the last X key frames.  
However, if we want to correct the last X keyframes, we will need to have access to Map::mspKeyFrames and Map::mspKeyFrames, 
both of which are protected attributes of Map. In addition, we have to make sure no other threads are updating to any keyframes 
that we are modifying (possibly by setting up mutex).
- The term *local coordinate system* and *local coordinate center* is also very ambiguous. I tried to use mCurrentFrame and mpReferenceKF,
but both of them failed to give satisfactory result.
- In the end, I also tried to use consecutive poses from local map. Sometime it's good, but once there is a missing frames, everything goes wrong.

## Implementation:
Most of the code added is in Tracking.cc . List of modified files (compared to master):
- Tracking.cc (and corresponding Tracking.h)
    - EstimateScale  
        Estimates the scale using the current frame and the previous frame. It return the scale, and save the distance to the 
        ground plane and the normal vector to the reference given.
        
        It first match the points between the previous frame and the current frame
        using ORB features. Then, we find the relative pose between the two frames using essential matrix decomposition.
        We filter out the outlier from that decomposition. After that, we find homography matrix relating previous frame 
        and current frame. Finally, we find the initial value for the d and n. In paper, after finding the initial value,
        we then optimise the value further, but this is not done here, so the initial value is set to be the final value.
        
        After we get d, we can get scale = actual_height / d.
    - solveRT  
        Given points of 2 frames, find the relative pose(R, t) between the first and the second frame.
    - solveH  
        Given points of 2 frames, find homography matrix relating the two frame's ground plane. The ground plane is defined as
        the area inside the ROI defined in this function.  
    - InitialSolver  
        Given R, t, and H, linearly approximate the ground plane's normal vector and the distance to the ground plane. 
        The linear approximation is defined in [Appearance-Guided Monocular Omnidirectional Visual Odometry for Outdoor Ground Vehicles](http://rpg.ifi.uzh.ch/docs/IEEE_TransRobotics_scaramuzza.pdf) part II D.
        The distance to the ground plane will be the height of the camera in the scaled world.
    - filterSrcAndDstPointsBasedOnMask  
        Since a lot of filter by cv::Mat mask needs to be done to both srcPoint and dstPoint, I make a function to do just that.
        It iterates over all points, if the mask is 0, then other point with mask = 1 will replace it. In the end, the vector is resized
        to the number of point with mask = 1.
    - Rescale  
        Triggered whenever there is a need for rescaling, in the current implementation is every interval determined in 
        Tracking::SCALE_UPDATE_PERIOD.  
        It first finds the scale defined in EstimateScale(). Then, it checks the quality of the scale by comparing the normal vector
        with the prior normal vector. If the difference in angle (found by dot product) is less than Tracking::NORMAL_ANGLE_THRESHOLD (default = 5 degrees)
        then the process will continue. Else, nothing is done, since the quality is not very good.  
        
        **THIS PART IS STILL WRONG**   
        Now we iterates over all KeyFrames in local map (mvpLocalKeyFrames), transform them to the local coordinate center that you think is correct.
        Rescale by multiplying the translation part with scale / old_scale. Then transform the back to global coordinate center.  
        The same is done to all MapPoints in local map (mvpLocalMapPoints). Since the coordinate is not homogenous, we first convert the coordinate to homogenous form.
        Then, we transform them to the local coordinate center, rescale by multiplying with scale / old_scale, then transform back to global coordinate center.
        Convert the coordinate back to non-homogenous form. 
    - Track  
        If some time has passed since the last rescaling, I express the need for rescaling there.
    - CreateInitialMapMonocular  
        I set the baseline of the first 2 frames to be *scale*. Every map points is scaled by the same *scale*.
    - CreateNewKeyFrame  
        After creating new frame, if there is a need for rescaling, Rescale() will be invoked.
- Frame.cc (and corresponding Frame.h)  
I planned to use AKAZE as the descriptor, but I changed my mind. To use AKAZE, I planned to add Image information to the Frame.
However, this might cause more memory consumption. So, the added code is commented. Other than that, nothing is changed.
- System.h  
Add getter to pointer to Tracking, to get ScaleHistory to be saved to file.
- mono_kitti.cc  
Since my computer is having issues with framerate, I investigate the time used to track each image. I add the code to save the 
time tracked to a file. Also, I output maximum tracking time.

## Issues/TODO:
- The computation (especially findHomography) takes quite some time (0.5 second).
- (Almost) no refinement has been done.
- Scale correction is still wrong
- Haven't handle case when findHomography failed
    - Sai's approach is to redo the findHomography
    - Can also just not do the scale correction at that time
- In the current implementation, if the scale's quality is not good (normal vector is not close enough), we wait for another interval.  
Maybe that is too long. What if we make such that if the quality not good, then after X frames we try again.

-------------------------------------------------------------

# ORB-SLAM2
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**13 Jan 2017**: OpenCV 3 and Eigen 3.3 are now supported.

**22 Dec 2016**: Added AR demo (see section 7).

ORB-SLAM2 is a real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. We provide examples to run the SLAM system in the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) as stereo or monocular, in the [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset) as RGB-D or monocular, and in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as stereo or monocular. We also provide a ROS node to process live monocular, stereo or RGB-D streams. **The library can be compiled without ROS**. ORB-SLAM2 provides a GUI to change between a *SLAM Mode* and *Localization Mode*, see section 9 of this document.

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>


### Related Publications:

[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORB-SLAM2 is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM2 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM2 (Monocular) in an academic work, please cite:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

if you use ORB-SLAM2 (Stereo or RGB-D) in an academic work, please cite:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 12.04**, **14.04** and **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## ROS (optional)
We provide some examples to process the live input of a monocular, stereo or RGB-D camera using [ROS](ros.org). Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed.

# 3. Building ORB-SLAM2 library and examples

Clone the repository:
```
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.

# 4. Monocular Examples

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```

## KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

# 5. Stereo Examples

## KITTI Dataset

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/mav0/cam0/data PATH_TO_SEQUENCE/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```

# 6. RGB-D Example

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# 7. ROS Examples

### Building the nodes for mono, monoAR, stereo and RGB-D
1. Add the path including *Examples/ROS/ORB_SLAM2* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM2:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM2/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Running Monocular Augmented Reality Demo
This is a demo of augmented reality where you can use an interface to insert virtual cubes in planar regions of the scene.
The node reads images from topic `/camera/image_raw`.

  ```
  rosrun ORB_SLAM2 MonoAR PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Running Stereo Node
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM2/Stereo. You will need to provide the vocabulary file and a settings file. If you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**.

  ```
  rosrun ORB_SLAM2 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```
  
**Example**: Download a rosbag (e.g. V1_01_easy.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab:
  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
  ```
  
Once ORB-SLAM2 has loaded the vocabulary, press space in the rosbag tab. Enjoy!. Note: a powerful computer is required to run the most exigent sequences of this dataset.

### Running RGB_D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM2/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
# 8. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular, stereo and RGB-D cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the ORB-SLAM2 library and how to pass images to the SLAM system. Stereo input must be synchronized and rectified. RGB-D input must be synchronized and depth registered.

# 9. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 


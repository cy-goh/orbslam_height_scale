## Monocular VO Scale Estimation

This repository sort of combines and compares the work of Dingfu Zhou's ***Ground Plane based Absolute Scale Estimation for Monocular Visual Odometry*** and Xiangwei Wang's ***Monocular Visual Odometry Scale Recovery using Geometrical Constraint***. Both used the mounted camera height to recover metric scale from monocular VO.

For the former's code, it was in Matlab so I translated into python. He used a predefined ROI to get ground points.

In the latter's repository, some of the libraries used are missing so I replaced with OpenCV. Secondly, I think there are also some errors in the conditions the author made to qualify ground points. I combine the addtional ground points found, with matched points inside the ROI to get more points for homography estimation. 

Each step in the notebook is documented with intermediate results (image) displayed. 

### Results

#### Kitti Seq 06

![alt text](experiments/georoi_vs_roi_seq6_scatter.png "Comparison results")
![alt text](experiments/georoi_vs_roi_seq6_wma_filter_angle5.5.png "Comparison results")

#### Kitti Seq 00
![alt text](experiments/georoi_vs_roi_seq0_scatter.png "Comparison results")
![alt text](experiments/georoi_vs_roi_seq0_wma_filter_angle5.5.png "Comparison results")

#### Kitti Seq 05
![alt text](experiments/georoi_vs_roi_seq5_scatter.png "Comparison results")
![alt text](experiments/georoi_vs_roi_seq5_wma_filter_angle5.5.png "Comparison results")

#### Kitti Seq 07
![alt text](experiments/georoi_vs_roi_seq7_scatter.png "Comparison results")
![alt text](experiments/georoi_vs_roi_seq7_wma_filter_angle5.5.png "Comparison results")

#### Todo

Inject these scale estimations into ORB SLAM.

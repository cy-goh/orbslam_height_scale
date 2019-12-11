# Author's Code
Mirror of the code given by the author of the paper [Reliable Scale Estimation and Correction for Monocular Visual Odometry](https://drive.google.com/file/d/0B73o7D_54u1LcFRPeWlIR1VubzA/view).
Actual code is from [here](https://sites.google.com/site/dingfuzhou/projects/reliable-scale-estimation-and-correction-for-visual-odometry)

## Notes 
The entry point of this algorithm starts from a 2 sets of point matches of 2 consecutive images. Point detection, descriptors and matching are done by the matcherMex (see below).

Compared  the actual project, I deleted all the training images from ./04/image_0 , since the file size is quite big.  
If you want to run this, please don't forget to download the dataset also. The dataset is image_0 from sequence 04 of KITTI
dataset, link [here](http://www.cvlibs.net/datasets/kitti/eval_odometry.php), take the grayscale one.

* For the case of ORB SLAM, scale estimation seemed to work with ORB descriptors and the ROI defined here (H_solver.m) *

## DemoNormalDirection.m
Here, the author just want to demonstrate the result of the feature matching and find the initial solution to the equation
that we want to optimise.  
For each two images, it match the feature using matcherMex. Then, it finds Rt by calling Rt_solver.m . It does a filter
of the outliers when finding Rt. Then it finds H using H_solver.m . It also filters the outlier out. Finally, it does an
approximation of the initial value for the distance to the ground plane (d0) and the normal of the ground plane (n).

## ScaleEstimation.m
This is the full version of the code, after doing what is in DemoNormalDirection.m , it then does the optimisation using 
Nelder Mead simplex method by calling Melder_Mead_simplex_Sparse.m . It also does kalman filter on the distance and normal. 
Finally, it finds the scale by dividing the actual height and the estimated height from kalman filter. The scale is saved and
compared with the ground truth scale. Ground truth scale is obtained by finding the norm of the translation part on the poses.
A graph is then shown, plotting the scale estimated over time compared to the ground truth.  
![](../../image/Ground%20Truth%20and%20Author's%20Code.png)

## matcherMex.mexw64
This windows 64-bit binary is a matcher compiled from [libviso library](http://www.cvlibs.net/software/libviso/). (point detection + feature descriptor + matching) We don't use matcher from libviso2 because it is 
in GPL3. Github mirror [here](https://github.com/akhil22/libviso2).

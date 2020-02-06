/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/imgproc.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>
#include <chrono>

#include <fstream>
#include <stdlib.h>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

inline float cosine_angle(cv::Mat n1, cv::Mat n2) {
    return acos(n1.dot(n2)/(norm(n1)*norm(n2)) );
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


bool Tracking::solveH(vector<Point2f> &srcPoint, vector<Point2f> &dstPoint, cv::Mat &H, Mat &mask)
{
    // This ROI defines the ground plane in front of the vehicle. The paper suggests that a basic road detector is used
    // to pre-determine the ROI needed, but for now, we just use the ROI defined in the author's code.
    vector<Point2f> ROI = {Point2f(500,230), Point2f(700, 230), Point2f(800, 380), Point2f(400, 380)};

    vector<Point2f> srcPointROI, dstPointROI;

    for(int i = 0 ; i < (int)srcPoint.size(); ++i) {
        // ISSUES: We only check if the source point is in ROI or not
        //         upon investigation, it is found that some of the point is match to the cloud etc.
        //         this might or might not affect findHomography badly
        // If srcPoint[i] is in / on the polygon defined in ROI, push both src and dst Point.
        if (cv::pointPolygonTest(ROI, srcPoint[i], false) >= 0) {
            srcPointROI.push_back(srcPoint[i]);
            dstPointROI.push_back(dstPoint[i]);
        }
    }

    // This commented code shows the keypoint after the last Frame's keypoint is cropped at ROI
    // The result will be saved as a file indicated in filename

//    cv::Mat imgTmp;
//    mImGray.copyTo(imgTmp);
//    for(cv::Point2f p : srcPoint)cv::circle(lastImgTmp, p, 5, cv::Scalar(255,0,0), -1);
//    cv::Mat lastImgTmp;
//    mLastImGray.copyTo(lastImgTmp);
//    for(cv::Point2f p : dstPoint)cv::circle(imgTmp, p, 5, cv::Scalar(0,0,255), -1);
//    cv::Mat imgConcat;
//    cv::vconcat(imgTmp, lastImgTmp, imgConcat);
//    string filename = "/media/sgp1053c/DATA/steven/ORB_SLAM2/imageTmpCombine/" + to_string(mLastFrame.mTimeStamp*10) + " to " + to_string(mCurrentFrame.mTimeStamp*10) + ".png";
//    bool res = cv::imwrite(filename, imgConcat);
//    if(res)cout << "image written to " << filename << endl;
//    else cout << "cannot write!" << endl;

    // Now we find the Homography matrix between the two ground planes.
    // H = findHomography(srcPointROI, dstPointROI, cv::RANSAC, 1.5, mask);	
    //TODO: check this value
    H = findHomography(srcPointROI, dstPointROI, cv::RANSAC, 2., mask);	
    H.convertTo(H, CV_32F);

    // TODO: refine homography matrix

    if(isfinite(H.at<float>(0,0)))
    {
        cout << "Estimated Homography succesfully" << endl;
        return true;
    }
    else
    {
        cout << "Something is wrong with HOMOGRAPHY matrix" << endl;
        return false;
    }
}

void Tracking::filterSrcAndDstPointsBasedOnMask(vector<cv::Point2f> &srcPoints, vector<cv::Point2f> &dstPoints, cv::Mat mask)
{
    int idx = 0; // idx will be the first index in which the mask = 0, also corresponds to the number of points with mask = 1
    for(MatConstIterator_<double> it = mask.begin<double>(); it!= mask.end<double>(); ++it)
    {
        if(*it>0)
        {
            srcPoints[idx] = srcPoints[it-mask.begin<double>()];
            dstPoints[idx++] = dstPoints[it-mask.begin<double>()];
        }
    }
    srcPoints.resize(idx);
    dstPoints.resize(idx);
}

void Tracking::solveRT(vector<cv::Point2f> &srcPoints, vector<cv::Point2f> &dstPoints, cv::Mat &R, cv::Mat &t, Mat &mask)
{
    cv::Mat E;
//    cv::Mat fRANSAC = cv::findFundamentalMat(srcPoints, dstPoints, cv::FM_RANSAC, 0.5, 0.999, mask);
//    fRANSAC.convertTo(fRANSAC, CV_32F);
//    E= mK.t() * fRANSAC * mK;
//    cout << E << endl;
  	 auto start = std::chrono::high_resolution_clock::now();
  	 cout << "points " << srcPoints.size() << endl;
    E = cv::findEssentialMat(srcPoints, dstPoints, mK, cv::RANSAC, 0.999, 1, mask);
     	auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    cout << "duration of essentialmat  " << duration << endl;
    cv::recoverPose(E, srcPoints, dstPoints, mK, R, t, mask);



    // refine matches using epipolar constraint
    // TODO: finish this code (follow the steps from the code)
//    cv::Mat homog_srcPoints, homog_dstPoints;
//    cv::convertPointsToHomogeneous(srcPoints, homog_srcPoints);
//    cv::convertPointsToHomogeneous(dstPoints, homog_dstPoints);
//    cv::Mat F_dis = cv::Mat::zeros(srcPoints.rows, 1, CV_32F);
//    for(int i=0;i<srcPoints.rows;i++)
//    {
//        cout << fRANSAC.type() << " " << homog_srcPoints.row(i).type() << endl;
//        cv::Mat abc = fRANSAC *homog_srcPoints.row(i)  ;
//        float den = cv::sqrt(abc.at<double>(0,0) * abc.at<double>(0,0) + abc.at<double>(0,1) * abc.at<double>(0,1));
//        cv::Mat dis = homog_dstPoints.row(i) * abc.t();
//        F_dis.at<double>(i) = cv::sqrt(dis.at<double>(0,0) * dis.at<double>(0,0) /den);
//    }
//    mask = F_dis < 0.5;
}

void Tracking::InitialSolver(cv::Mat R, cv::Mat t, cv::Mat H, float &d0, cv::Mat &n)
{
    Eigen::MatrixXf A_Eigen = Eigen::MatrixXf::Zero(9, 4);
    Eigen::MatrixXf B_Eigen = Eigen::MatrixXf::Zero(9, 1);

    cv::Mat K, h;
    H.convertTo(H, CV_32FC1);
    mK.convertTo(K, CV_32FC1);
    t.convertTo(t, CV_32FC1);
    R.convertTo(R, CV_32FC1);
    // This part is copied from sai's code

    h = mK.inv() * H * mK;//implemented just the homography decomposition part from their paper into this section - this is based on method proposed in 2008 paper to estimate scale
    //the 2014 paper author suggested to perform one more optimization and add kalman filtering to improve scale estimation accuracy, but that is not yet implemented here

    A_Eigen(0, 0) = -h.at<float>(0, 0);
    A_Eigen(0, 1) = t.at<float>(0, 0);
    A_Eigen(0, 2) = 0.0;
    A_Eigen(0, 3) = 0;
    A_Eigen(1, 0) = -h.at<float>(0, 1);
    A_Eigen(1, 1) = 0.0;
    A_Eigen(1, 2) = t.at<float>(0, 0);
    A_Eigen(1, 3) = 0;
    A_Eigen(2, 0) = -h.at<float>(0, 2);
    A_Eigen(2, 1) = 0.0;
    A_Eigen(2, 2) = 0.0;
    A_Eigen(2, 3) = t.at<float>(0, 0);

    A_Eigen(3, 0) = -h.at<float>(1, 0);
    A_Eigen(3, 1) = t.at<float>(1, 0);
    A_Eigen(3, 2) = 0.0;
    A_Eigen(3, 3) = 0;
    A_Eigen(4, 0) = -h.at<float>(1, 1);
    A_Eigen(4, 1) = 0;
    A_Eigen(4, 2) = t.at<float>(1, 0);
    A_Eigen(4, 3) = 0;
    A_Eigen(5, 0) = -h.at<float>(1, 2);
    A_Eigen(5, 1) = 0;
    A_Eigen(5, 2) = 0.0;
    A_Eigen(5, 3) = t.at<float>(1, 0);

    A_Eigen(6, 0) = -h.at<float>(2, 0);
    A_Eigen(6, 1) = t.at<float>(2, 0);
    A_Eigen(6, 2) = 0.0;
    A_Eigen(6, 3) = 0;
    A_Eigen(7, 0) = -h.at<float>(2, 1);
    A_Eigen(7, 1) = 0;
    A_Eigen(7, 2) = t.at<float>(2, 0);
    A_Eigen(7, 3) = 0;
    A_Eigen(8, 0) = -h.at<float>(2, 2);
    A_Eigen(8, 1) = 0;
    A_Eigen(8, 2) = 0.0;
    A_Eigen(8, 3) = t.at<float>(2, 0);

    B_Eigen(0, 0) = R.at<float>(0, 0);
    B_Eigen(1, 0) = R.at<float>(0, 1);
    B_Eigen(2, 0) = R.at<float>(0, 2);
    B_Eigen(3, 0) = R.at<float>(1, 0);
    B_Eigen(4, 0) = R.at<float>(1, 1);
    B_Eigen(5, 0) = R.at<float>(1, 2);
    B_Eigen(6, 0) = R.at<float>(2, 0);
    B_Eigen(7, 0) = R.at<float>(2, 1);
    B_Eigen(8, 0) = R.at<float>(2, 2);
    B_Eigen = -B_Eigen;

    //first way of solving Ax=B -> as good as possible
    Eigen::Vector4f X;
    // X = A_Eigen.fullPivHouseholderQr().solve(B_Eigen);
    // d0 = 1.0 / sqrt(pow(X(1), 2.0) + pow(X(2), 2.0) + pow(X(3), 2.0));
    // cout << "d0 is" << d0 << endl;
    // cout << "R is " << R << endl;
    // cout << "t is " << t << endl;
    // cout << "X is " << X << endl;
    // n = (cv::Mat_<float>(3,1) << X(1,0),
    //                              X(2,0),
    //                              X(3,0));
    // n = n / cv::norm(n);

    //second way of solving Ax=B
   X = A_Eigen.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(B_Eigen);
   d0 = 1.0 / sqrt(pow(X(1), 2.0) + pow(X(2), 2.0) + pow(X(3), 2.0));
   n = (cv::Mat_<float>(3,1) << X(1,0),
           X(2,0),
           X(3,0));
   n = n / cv::norm(n);
       cout << "d0 is" << d0 << endl;
    cout << "R is " << R << endl;
    cout << "t is " << t << endl;
    cout << "X is " << X << endl;

//    ==================================================================================================================

    // This part is adapted from the paper's code
//    cv::Mat mKInv = mK.inv();
//    cv::Mat h = (mKInv * H) * mK;
//    cv::Mat AA = (cv::Mat_<float>(9,4) << -h.at<float>(0,0), t.at<float>(0,0), 0, 0,
//                                          -h.at<float>(0,1), 0, t.at<float>(0,0), 0,
//                                          -h.at<float>(0,2), 0, 0, t.at<float>(0,0),
//
//                                          -h.at<float>(1,0), t.at<float>(1,0), 0, 0,
//                                          -h.at<float>(1,1), 0, t.at<float>(1,0), 0,
//                                          -h.at<float>(1,2), 0, 0, t.at<float>(1,0),
//
//                                          -h.at<float>(2,0), t.at<float>(2,0), 0, 0,
//                                          -h.at<float>(2,1), 0, t.at<float>(2,0), 0,
//                                          -h.at<float>(2,2), 0, 0, t.at<float>(2,0));
//
//    cv::Mat B = -(cv::Mat_<float>(9,1) << R.at<float>(0,0),
//                                          R.at<float>(0,1),
//                                          R.at<float>(0,2),
//                                          R.at<float>(1,0),
//                                          R.at<float>(1,1),
//                                          R.at<float>(1,2),
//                                          R.at<float>(2,0),
//                                          R.at<float>(2,1),
//                                          R.at<float>(2,2));
//    cv::Mat x;
//    cv::solve(AA, B, x, DECOMP_QR);
//    d0 = 1.0 / cv::norm(x(cv::Range(1,4), cv::Range::all()));
//    n = x(cv::Range(1,4), cv::Range(0,1));
//    n = n / cv::norm(n);
}

float Tracking::EstimateScale(float &d, cv::Mat &n, cv::Mat refR, cv::Mat refT)
{
    // TODO: make this inside config file instead of hard coded in source code
    // actual_height taken from http://www.cvlibs.net/datasets/kitti/eval_odometry_detail.php?&result=d568463c0d37f8029259debe61cc3b0f793818f2
    const float actual_height = 1.7;


    // This part of code is me wanting to use AKAZE descriptor instead of the readily available ORB descriptor.
    // In the end, I did not use AKAZE, so I commented this out.
    // In order to try this effortlessly, I put the image in the Frame
    // no check has been done to the memory used after this addition
    // if the memory requirement suddenly very big, that is one of the cause
//    cv::Mat tmpCur, tmpPrev;
//    cv::undistort(mCurrentFrame.im, tmpCur, mK, mDistCoef);
//    cv::undistort(mLastFrame.im, tmpPrev, mK, mDistCoef);
//    vector<cv::KeyPoint> kpCur, kpPrev;
//    cv::Mat descCur, descPrev;
//    cv::Ptr<cv::AKAZE> fdetector = cv::AKAZE::create();
//    fdetector->setThreshold(0.1e-4);
//    fdetector->detectAndCompute(tmpCur, cv::Mat(), kpCur, descCur);
//    fdetector->detectAndCompute(tmpPrev, cv::Mat(), kpPrev, descPrev);

    // Match the points between the last frame and the current frame
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    vector<cv::DMatch> matches;
    vector<vector<cv::DMatch> > nn_matches;
    matcher.knnMatch(mLastFrame.mDescriptors, mCurrentFrame.mDescriptors, nn_matches, 2);

    vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < nn_matches.size(); i++) {
        DMatch first = nn_matches[i][0];
        float dist1 = nn_matches[i][0].distance;
        float dist2 = nn_matches[i][1].distance;
        if (dist1 < 0.80 * dist2) {
            good_matches.push_back(nn_matches[i][0]);
        }
    }

    matches.swap(good_matches);

    // matcher.match(mLastFrame.mDescriptors, mCurrentFrame.mDescriptors, matches);
    vector<cv::Point2f> srcPoint, dstPoint;
    for(cv::DMatch pairMatch: matches)
    {
        srcPoint.emplace_back(mLastFrame.mvKeys[pairMatch.queryIdx].pt);
        dstPoint.emplace_back(mCurrentFrame.mvKeys[pairMatch.trainIdx].pt);
    }

    cv::Mat R, t, H, mask;

    // estimate R and t using srcPoint and dstPoint
    // filter srcPoint and dstPoint to only contain the inliers
    	 auto start = std::chrono::high_resolution_clock::now();

    solveRT(srcPoint, dstPoint, R, t, mask);
 	auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    cout << "duration of solvert " << duration << endl;
    filterSrcAndDstPointsBasedOnMask(srcPoint, dstPoint, mask);
//    cout << "Number of inlier after solveRT: " << inlierIndex.size() << endl;


    // estimate H
    // filter srcPoint and dstPoint to only contain the inliers
    // TODO: handle case if homography failed
        //auto start = chrono::high_resolution_clock::now();
    bool retval = solveH(srcPoint, dstPoint, H, mask);

	// auto end = std::chrono::high_resolution_clock::now();
 //    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
 //        cout << "duration of homography " << duration << endl;

    filterSrcAndDstPointsBasedOnMask(srcPoint, dstPoint, mask);

//    cout << "Number of inlier after solveH: " << inlierIndex.size() << endl;

    // estimate d
	// auto start = std::chrono::high_resolution_clock::now();
    if (refR.empty() || refT.empty())
        InitialSolver(R, t, H, d, n);
    else {
        cout << "t: " << t << endl;
        cout << "Ref t: " << refT << endl;    
        InitialSolver(refR, refT, H, d, n);
    }

 	// auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    // cout << "duration of init solver " << duration << endl;

    // TODO: refine the height using the given equation from the paper + KalmanFilter

    // find scale
    float scale = actual_height/d;
//    cout << "Estimated height: " << scaled_height << endl;
//    cout << "SCALE: " << scale << endl;
    //return scale;
    return scale;
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray.copyTo(mLastImGray);
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            //FIXME: turn this on/off
            // [SCALE CORRECTION] I (tried to) rescale the map every given interval
            // Comment this section if you don't want to have scale correction
           if(mCurrentFrame.mTimeStamp - mTimeStampLastUpdate > SCALE_UPDATE_PERIOD)
           {
               // This signals the need of rescaling .
               // When the next key frame has been addeed, if this variable is true, then rescaling will be done.
                bNeedRescale = true;
               // This part of code is to just see the scale estimation performance.
            //    float d;
            //    cv::Mat n;
            //    float scale = EstimateScale(d,n);
            //    scaleHistory.push_back(scale);
               mTimeStampLastUpdate = mCurrentFrame.mTimeStamp;
           }

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;
    float d;
    cv::Mat n;
	auto start = std::chrono::high_resolution_clock::now();

    cv::Mat R = pKFcur->GetRotation();
    // cv::Mat R = pKFcur->GetRotation().t();
    cv::Mat t = pKFcur->GetPose().rowRange(0, 3).col(3);
    // cv::Mat t = pKFcur->GetCameraCenter();

    const float pitch_rot = 4. * M_PI/180.;
    Eigen::Vector4f v(0, 1, 0, 1);
    Eigen::Matrix4f eigRot = Eigen::Matrix4f::Identity();
    eigRot(1, 1) = cos(pitch_rot);
    eigRot(1, 2) = -sin(pitch_rot);
    eigRot(2, 1) = sin(pitch_rot);
    eigRot(2, 2) = cos(pitch_rot);
    Eigen::Vector4f normal_prior = eigRot * v;
    cv::Mat prior_cv(3, 1, CV_32F);
    prior_cv.at<float>(0, 0) = normal_prior(0);
    prior_cv.at<float>(1, 0) = normal_prior(1);
    prior_cv.at<float>(2, 0) = normal_prior(2);
    cout << "prior is " << prior_cv << endl;
     
    float scale = EstimateScale(d, n, R, t);
	auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    // cout << "duration of scale " << duration << endl;

    // cout << "scale is " << scale << endl;
    cout << "normal is " << n << endl;
    
    float angle = cosine_angle(prior_cv, n);
    cout << "calculated angle: " << angle * 180/M_PI << endl;
    if (angle > NORMAL_ANGLE_THRESHOLD) {
        cout << "POOR INITIALIZATION..... " << endl;
        exit(1);
    }
    

    if(scale<0 || pKFcur->TrackedMapPoints(1)<100)
    // if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    // Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*scale;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();

    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            //pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
             pMP->SetWorldPos(pMP->GetWorldPos()*scale);
        }
    }

//  Initialise time stamp on last update with the current frame's time stamp
    mTimeStampLastUpdate = mCurrentFrame.mTimeStamp;
    bNeedRescale = false;
    oldScale = scale;
    n.copyTo(prevNormal);
    scaleHistory.clear();

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;

    // After we insert the keyframes, if we need to rescale, then we do the rescaling.
    if(bNeedRescale)
        Rescale();
}

bool compareKeyFrameId(KeyFrame* kf1, KeyFrame* kf2)
{
    return (kf1->mnFrameId <  kf2->mnFrameId);
}



// Ground Plane based Absolute Scale Estimation for Monocular Visual Odometry
// This rescaling function is implementing the scale correction proposed by this paper
// https://arxiv.org/pdf/1903.00912.pdf
void Tracking::Rescale()
{
        static int c = 0;
        // if (c > 2) return;

    cout << "===================\nRescaling..." << endl;

    // I think that if you want to modify the KeyFrame and MapPoints, still need to make sure that no other thread is using.
    // I don't know how to turned off the GBA in Tracking thread, so I just turned off the loop closing thread
    // TODO: find ways to terminate GBA from Tracking thread

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // TODO: It is assumed that we turned off the Loop Closing Module
    //       If you need that module, need to make sure GBA is stopped before continuing

    if (mpLoopClosing->isRunningGBA())
    {
        mpLoopClosing->StopGBA();
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        //cout << "Waiting..." << endl;
        usleep(1000);
    }

    // Ensure current keyframe is updated
    mpLastKeyFrame->UpdateConnections();

    // First, we get the estimated scale
    float d;
    cv::Mat n;
    cv::Mat V = mVelocity.clone();
    cv::Mat VR = V.rowRange(0,3).colRange(0,3);
    cv::Mat Vt = V.rowRange(0,3).col(3);
    float scale = EstimateScale(d,n, VR, Vt);
    //float scale = EstimateScale(d,n);
    cout << "Vt:" << endl << Vt << endl; 
    cout << "Scale, previous scale, angle: " << scale << " " << oldScale << " " << acos(n.dot(prevNormal)/(norm(n)*norm(prevNormal)))* 180 / M_PI << endl;
    cout << "Current NOrmal: " << n << endl;

    const float pitch_rot = 4. * M_PI/180.;
    Eigen::Vector4f v(0, 1, 0, 1);
    Eigen::Matrix4f eigRot = Eigen::Matrix4f::Identity();
    eigRot(1, 1) = cos(pitch_rot);
    eigRot(1, 2) = -sin(pitch_rot);
    eigRot(2, 1) = sin(pitch_rot);
    eigRot(2, 2) = cos(pitch_rot);
    Eigen::Vector4f normal_prior = eigRot * v;
    cv::Mat prior_cv(3, 1, CV_32F);
    prior_cv.at<float>(0, 0) = normal_prior(0);
    prior_cv.at<float>(1, 0) = normal_prior(1);
    prior_cv.at<float>(2, 0) = normal_prior(2);

    //cout << "current frame pose: \n" << mCurrentFrame.mTcw<< endl;
    float angle = cosine_angle(prior_cv, n);
    cout << "new normal angle is " << angle * 180 / M_PI << endl;


    // If the angle between normal from estimation and the last normal is greater that 5 degrees, do not rescale
     if(angle < NORMAL_ANGLE_THRESHOLD)
    //  if(acos(n.dot(prevNormal)/(norm(n)*norm(prevNormal))) < NORMAL_ANGLE_THRESHOLD)
    //  if(acos(n.dot(prevNormal)/(norm(n)*norm(prevNormal))) < NORMAL_ANGLE_THRESHOLD && fabs(oldScale/scale - 1) > 0.075)
    {
        // Since the scale is good, we accept the scale and perform the rescaling
        cout << "Scale accepted" << endl;
        c++;
        //unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        // TODO: rescaling is very wrong

//        cout << "Local Coordinate Center ID: " <<  mpLastKeyFrame->mnId << endl;
//        sort(mvpLocalKeyFrames.begin(), mvpLocalKeyFrames.end(), KeyFrame::lId);

        // Convert local map to local coordinate system
        //   Here, I assume that the local coordinate center is the mCurrentFrame
        // Rescale using scale from scale estimation
        // Convert back to global coordinate system

        //added by CY: this is from KeyFrame.cc setPose. Get Twc so as to setup local Coordinate System
        // cv::Mat TcwCorrected = mCurrentFrame.mTcw.clone();
        // TcwCorrected.rowRange(0,3).col(3) = TcwCorrected.rowRange(0, 3).col(3) * scale / oldScale;
        // //cv::Mat Tlc = mLastFrame.mTcw * mCurrentFrame.mTcw.inv();
        // // float oldScale1 = cv::norm(Tlc.rowRange(0,3).col(3));
        // //Tlc.rowRange(0, 3).col(3) = Tlc.rowRange(0,3).col(3) * scale / oldScale;// / oldScale1;
        // //cv::Mat correctedCurFramePose = Tlc.inv() * mLastFrame.mTcw;
        // // mCurrentFrame.mTcw = Tlc.inv() * mLastFrame.mTcw;
        // //mpLastKeyFrame->SetPose(correctedCurFramePose);
        // mpLastKeyFrame->SetPose(TcwCorrected);
        // mpLastKeyFrame->UpdateConnections();

        // cv::Mat Twc = mCurrentFrame.mTcw.clone().inv();

        cout << "current id " << mCurrentFrame.mnId << " " << mpLastKeyFrame->mnFrameId << endl;
        // cout << mCurrentFrame.mTcw << endl << "Reference:" << mCurrentFrame.mpReferenceKF->mnFrameId << endl;
        // cout << "compare " << mpLastKeyFrame->GetPose() << mCurrentFrame.mTcw << endl;

        std::vector<KeyFrame*> sortedLocalKeyFrames(mvpLocalKeyFrames);
        sort(sortedLocalKeyFrames.begin(), sortedLocalKeyFrames.end(), compareKeyFrameId);

        ofstream beforeFile("before.txt"), afterFile("after.txt");
        std::vector<KeyFrame*> mykfs = mpMap->GetAllKeyFrames();
        for (int i =0; i < mykfs.size(); i++){
            KeyFrame* pKF = mykfs[i];
            if (pKF->isBad())
                continue;
            cv::Mat t = pKF->GetCameraCenter();
            beforeFile << t.at<float>(0) << "," << t.at<float>(1) << "," << t.at<float>(2) << endl;
        }

        // This is for the KeyFrames
        KeyFrame* firstFrame = sortedLocalKeyFrames[0];
        cv::Mat Twf = firstFrame->GetPoseInverse();
        cv::Mat Tfw = firstFrame->GetPose();
        cv::Mat Rfw = Tfw.rowRange(0, 3).colRange(0, 3);
        cv::Mat tfw = Tfw.rowRange(0, 3).col(3);
        g2o::Sim3 g2oSfw(Converter::toMatrix3d(Rfw), Converter::toVector3d(tfw), 1.);         

        // for(int i = 0 ; i < (int)mvpLocalKeyFrames.size() ; i++)
        for(int i = 1 ; i < sortedLocalKeyFrames.size() ; i++)
        {
//            cout << "(" << mvpLocalKeyFrames[i]->mnId << ", " << mvpLocalKeyFrames[i]->mnFrameId << ") "; // (KeyFrameID, FrameID)
/*             cv::Mat pose = mpReferenceKF->GetPose() * mvpLocalKeyFrames[i]->GetPoseInverse();
            pose.col(3).rowRange(0,3) = pose.col(3).rowRange(0,3) * scale / oldScale;
            pose = mpReferenceKF->GetPoseInverse() * pose.clone();
            mvpLocalKeyFrames[i]->SetPose(pose.inv()); */
//            mvpLocalKeyFrames[i]->UpdateConnections();

            //added by CY
            //comments: ok now i got the scale from Tic
            cv::Mat Tiw = sortedLocalKeyFrames[i]->GetPose();
            cv::Mat Tif = Tiw * Twf; //Tiw * Twc
            cv::Mat Rif = Tif.rowRange(0, 3).colRange(0, 3);
            cv::Mat tif = Tif.rowRange(0, 3).col(3);
            g2o::Sim3 g2oSif(Converter::toMatrix3d(Rif), Converter::toVector3d(tif), 1/scale);

            g2o::Sim3 g2oSiwCorrected = g2oSif * g2oSfw;

            //Tif.rowRange(0, 3).col(3) = Tif.rowRange(0,3).col(3) * scale;//  / oldScale;
            //cv::Mat TiwCorrected = Tif * firstFrame->GetPose();
            Eigen::Matrix3d eigR = g2oSiwCorrected.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oSiwCorrected.translation();
            double s = g2oSiwCorrected.scale();
            eigt *= 1./s;
            cv::Mat TiwCorrected = Converter::toCvSE3(eigR, eigt);
            sortedLocalKeyFrames[i]->SetPose(TiwCorrected);

            cv::Mat t = sortedLocalKeyFrames[i]->GetCameraCenter();
            afterFile << t.at<float>(0) << "," << t.at<float>(1) << "," << t.at<float>(2)<< endl;

            sortedLocalKeyFrames[i]->UpdateConnections();

            // cout << "before: " << tmp << endl;
            // cout << "after: " << TiwCorrected << endl;
            //end added by CY

            cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
            // cv::Mat RiwCorrected = TiwCorrected.rowRange(0, 3).colRange(0, 3);
            cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
            // cv::Mat tiwCorrected = TiwCorrected.rowRange(0, 3).col(3);
            g2o::Sim3 g2oiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.);
            // g2o::Sim3 g2oiwCorrected(Converter::toMatrix3d(RiwCorrected), Converter::toVector3d(tiwCorrected), 1./scale/* /oldScale */);
            // g2o::Sim3 g2owiCorrected = g2oiwCorrected.inverse();
            g2o::Sim3 g2oSwiCorrected = g2oSiwCorrected.inverse();

            vector<MapPoint*> mapPointsMatches =sortedLocalKeyFrames[i]->GetMapPointMatches();
            for(size_t indexMP = 0; indexMP < mapPointsMatches.size(); indexMP++)
            {
                MapPoint* mapPointi = mapPointsMatches[indexMP];
                if (!mapPointi || mapPointi->isBad())
                    continue;
                if (mapPointi->mnCorrectedByKF == mpLastKeyFrame->mnId)
                    continue;

                cv::Mat P3Dw = mapPointi->GetWorldPos();
                Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = g2oSwiCorrected.map(g2oiw.map(eigP3Dw));
                cv::Mat correctedMapPoint = Converter::toCvMat(eigCorrectedP3Dw);
                mapPointi->SetWorldPos(correctedMapPoint);
                mapPointi->mnCorrectedByKF = mpLastKeyFrame->mnId;
                mapPointi->mnCorrectedReference = sortedLocalKeyFrames[i]->mnId;
                mapPointi->UpdateNormalAndDepth();
            }
        }

        cv::Mat Tcf = mpLastKeyFrame->GetPose() * Twf;
        Tcf.rowRange(0, 3).col(3) = Tcf.rowRange(0,3).col(3) * scale;//  / oldScale;
        cv::Mat TcwCorrected = Tcf * firstFrame->GetPose();
        mpLastKeyFrame->SetPose(TcwCorrected);
        mCurrentFrame.SetPose(TcwCorrected);

        mpLastKeyFrame->UpdateConnections();


//        cout << endl;
        beforeFile.close(); afterFile.close();
        // exit(1);

        //commented by cy: I dont think this is needed
        //cv::Mat curPose = mpReferenceKF->GetPose() * mLastFrame.mTcw.inv();
        //curPose.col(3).rowRange(0,3) = curPose.col(3).rowRange(0,3) * scale / oldScale;
        //curPose = mpReferenceKF->GetPoseInverse() * curPose.clone();
        //mLastFrame.SetPose(curPose);

        //mVelocity = mpReferenceKF->GetPose() * mVelocity.clone();
        //mVelocity.col(3).rowRange(0,3) = mVelocity.col(3).rowRange(0,3) * scale / oldScale;
        //mVelocity = mpReferenceKF->GetPoseInverse() * mVelocity.clone();

        //added by CY
        cv::Mat bv = mVelocity.clone();
        // float oldScale = cv::norm(mVelocity.rowRange(0,3).col(3));
        mVelocity.col(3).rowRange(0,3) = mVelocity.col(3).rowRange(0,3) * scale;// / oldScale;
        // cout << "scale: " << oldScale << " " << scale << endl;
        // cout << "Before velocity " << bv << endl << mVelocity << endl;

        //  This part is for the map points, I haven't finish this one since if the keyframe is wrong, I think the map points will
        //  be even more wrong.
        // TODO: finish this part
        /*for(int i = 0 ; i < (int)mvpLocalMapPoints.size() ; i++)
        {
            cv::Mat poseHomog = cv::Mat::ones(4,1,CV_32F), pose;
            mvpLocalMapPoints[i]->GetWorldPos().copyTo(poseHomog.rowRange(0,3)); //poseHomog is T_pw

            //  cout << "before pt " << poseHomog.rowRange(0,3);
        
            poseHomog = Twc * poseHomog; //Tpc
            //float oldScale = cv::norm(poseHomog.rowRange(0,3).col(0));

            //point from current frame, transform back to global coordinate
            poseHomog.rowRange(0,3).col(0) = poseHomog.rowRange(0,3).col(0) * scale / oldScale;
            cv::Mat pGlobal =  TcwCorrected * poseHomog;
            pose = pGlobal.rowRange(0,3);
            //  cout << " after pt " << pose << endl;

            //  g2o::Sim3

            mvpLocalMapPoints[i]->SetWorldPos(pose);
            mvpLocalMapPoints[i]->UpdateNormalAndDepth();


            // cout << "size " << pGlobal.rows << " " << pGlobal.cols << endl;
            //poseHomog = mpReferenceKF->GetPose() * poseHomog.clone();
            //poseHomog.rowRange(0,3).col(3) = poseHomog.rowRange(0,3).col(3) * scale / oldScale;
            // poseHomog = mpReferenceKF->GetPoseInverse() * poseHomog.clone();
            //pose = poseHomog.col(0).rowRange(0,3);
            //mvpLocalMapPoints[i]->SetWorldPos(pose);
        }*/


        oldScale = scale;
        n.copyTo(prevNormal);
        bNeedRescale = false;
        mTimeStampLastUpdate = mCurrentFrame.mTimeStamp;

        //    Do local BA for refinement (optional)

    }
    // Allow the Local Mapper to continue
    UpdateLocalMap();
    mpLocalMapper->Release();
    
    mpMapDrawer->DrawMapPoints();

    // bNeedRescale = false;
    // bool test = false;
    // Optimizer::LocalBundleAdjustment(mpLastKeyFrame, &test, mpMap);

}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM

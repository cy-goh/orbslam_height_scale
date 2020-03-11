#include "Tracking.h"
#include "Converter.h"
#include "Utils.h"
#include <assert.h>
#include <numeric>

#include <opencv2/imgproc/imgproc.hpp>

namespace ORB_SLAM2
{

// bool CompareKeyFrameId(KeyFrame *kf1, KeyFrame *kf2)
// {
//     return (kf1->mnFrameId < kf2->mnFrameId);
// }

vector<int> Tracking::GetGroundPts(vector<Point2f> &srcPoint, vector<Point2f> &dstPoint)
{
    return GetPointsByROI(srcPoint, dstPoint);
}

vector<int> Tracking::GetPointsByROI(vector<Point2f> &srcPoint, vector<Point2f> &dstPoint)
{
    //based on org author's ROI
    const vector<Point2f> ROI = {Point2f(500, 230), Point2f(700, 230), Point2f(800, 380), Point2f(400, 380)};
    vector<int> results;

    for (int i = 0; i < srcPoint.size(); ++i)
    {
        // FIXME: only src points are checked if in ROI
        if (cv::pointPolygonTest(ROI, srcPoint[i], false) >= 0)
            results.push_back(i);
    }

    return results;
}

vector<int> Tracking::GetPointsByTriangulation(vector<Point2f> &srcPoint, vector<Point2f> &dstPoint, const cv::Mat R, const cv::Mat t)
{
    // static int i = 0;

    //do triangulation to get 3D points
    cv::Mat points3D;

    vector<int> results, triangleIndices;

    //P0
    cv::Mat origin = cv::Mat::zeros(3, 4, CV_32F);
    cv::Mat I = cv::Mat::eye(3, 3, CV_32F);
    I.copyTo(origin.rowRange(0, 3).colRange(0, 3));

    //P1
    cv::Mat ext = cv::Mat::zeros(3, 4, CV_32F);
    R.copyTo(ext.rowRange(0, 3).colRange(0, 3));
    t.copyTo(ext.rowRange(0, 3).col(3));

    cv::Mat srcProj = mK * origin;
    cv::Mat dstProj = mK * ext;
    if (srcPoint.empty())
        return results;

    cv::triangulatePoints(srcProj, dstProj, srcPoint, dstPoint, points3D);

    vector<Point2f> resultsLoc;
    cv::Rect rect(0, 0, mCurrentFrame.im.cols, mCurrentFrame.im.rows);
    Subdiv2DIndex delaunay(rect);

    vector<Point2f> &refPoints = srcPoint;

    if (refPoints.size() < 3)
    {
        return results;
    }

    delaunay.insert(refPoints);
    delaunay.getTrianglesIndices(triangleIndices);
    // cv::Mat img = mCurrentFrame.im.clone();

    // for (int i = 0; i < triangleIndices.size(); i+= 3)
    // {
    //     cv::Point pt1 = Point(refPoints[triangleIndices[i]]);
    //     cv::Point pt2 = Point(refPoints[triangleIndices[i+1]]);
    //     cv::Point pt3 = Point(refPoints[triangleIndices[i+2]]);
    //     line(img, pt1, pt2, Scalar(255,0,0), 1, CV_AA, 0);
    //     line(img, pt1, pt3, Scalar(255,0,0), 1, CV_AA, 0);
    //     line(img, pt2, pt3, Scalar(255,0,0), 1, CV_AA, 0);
    // }
    // std::stringstream fs;
    // fs << "/home/cy/Desktop/ground/delay" << i << ".png";
    // cv::imwrite(fs.str(), img);

    assert(triangleIndices.size() % 3 == 0);

    cv::Mat b = cv::Mat::ones(3, 1, CV_32F);

    for (int i = 0; i < triangleIndices.size(); i += 3)
    {
        cv::Mat triMat(3, 3, CV_32F);

        cv::Mat p1 = points3D.col(triangleIndices[i]);
        p1 /= p1.at<float>(3, 0);
        p1.rowRange(0, 3).col(0).copyTo(triMat.col(0));

        cv::Mat p2 = points3D.col(triangleIndices[i + 1]);
        p2 /= p2.at<float>(3, 0);
        p2.rowRange(0, 3).col(0).copyTo(triMat.col(1));

        cv::Mat p3 = points3D.col(triangleIndices[i + 2]);
        p3 /= p3.at<float>(3, 0);
        p3.rowRange(0, 3).col(0).copyTo(triMat.col(2));

        triMat = triMat.t();
        cv::Mat normMat = triMat.inv() * b; //should be size 3x1
        assert(normMat.cols == 1 && normMat.rows == 3);

        float normLength = cv::norm(normMat);
        float height = 1. / normLength;

        //if (normMat.at<float>(1, 0) < 0.)
        //    continue;

        float angle = acos(mPriorNormal.dot(normMat / normLength)) * 180 / M_PI;

        if (normMat.at<float>(1, 0) > 0 && fabs(angle) < 5. && p1.at<float>(1, 0) > 0. && p2.at<float>(1, 0) > 0. && p3.at<float>(1, 0) > 0.)
        {
            results.push_back(triangleIndices[i]);
            results.push_back(triangleIndices[i + 1]);
            results.push_back(triangleIndices[i + 2]);

            // resultsLoc.push_back(refPoints[triangleIndices[i]]);
            // resultsLoc.push_back(refPoints[triangleIndices[i+1]]);
            // resultsLoc.push_back(refPoints[triangleIndices[i+2]]);
            // cout << points3D.col(triangleIndices[i])<<endl;
            // cout << points3D.col(triangleIndices[i+1])<<endl;
            // cout << points3D.col(triangleIndices[i+2])<<endl;
            // cout << normMat << endl;
            // cout << "========================================"<<endl;
        }
    }

    //TODO: remove this
    // cv::Mat img2 = mCurrentFrame.im.clone();
    // cv::cvtColor(img2, img2, CV_GRAY2BGR);
    // for (int i = 0 ;  i < resultsLoc.size(); i++)
    // {
    //     Scalar colors[5] = {Scalar(0, 255, 255) , Scalar(255,0, 0), Scalar(0, 255, 0), Scalar(0, 0, 255), Scalar(128, 0, 128)};
    //     auto color = colors[(i/3)%5];
    //     cv::circle(img2, resultsLoc[i], 3, color );
    // }
    // std::stringstream fs2;
    // fs2 << "/home/cy/Desktop/ground/" << i++ << ".png";
    // cv::imwrite(fs2.str(), img2);

    return results;
}

void Tracking::FilterSrcAndDstPointsBasedOnMask(vector<cv::Point2f> &srcPoints, vector<cv::Point2f> &dstPoints, cv::Mat mask)
{
    int idx = 0; // idx will be the first index in which the mask = 0, also corresponds to the number of points with mask = 1
    for (MatConstIterator_<uchar> it = mask.begin<uchar>(); it != mask.end<uchar>(); ++it)
    {
        if (*it > 0)
        {
            srcPoints[idx] = srcPoints[it - mask.begin<uchar>()];
            dstPoints[idx++] = dstPoints[it - mask.begin<uchar>()];
        }
    }
    srcPoints.resize(idx);
    dstPoints.resize(idx);
}

bool Tracking::SolveH(vector<Point2f> &srcPoint, vector<Point2f> &dstPoint, cv::Mat &H, Mat &mask, cv::Mat &R, cv::Mat &t)
{
    /////////// experiment /////////////////
    vector<int> pts1 = GetPointsByTriangulation(srcPoint, dstPoint, R, t);
    vector<int> pts2 = GetGroundPts(srcPoint, dstPoint);
    vector<int> pts12;

    pts12.reserve( pts1.size() + pts2.size() ); // preallocate memory
    pts12.insert( pts12.end(), pts1.begin(), pts1.end() );
    pts12.insert( pts12.end(), pts2.begin(), pts2.end() );

    vector<Point2f> srcPointROI, dstPointROI;
    for (auto i : pts12)
    {
        srcPointROI.push_back(srcPoint[i]);
        dstPointROI.push_back(dstPoint[i]);
    }

    if (srcPointROI.empty())
        return false;

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
    H = findHomography(srcPointROI, dstPointROI, cv::RANSAC, 1.5, mask);
    H.convertTo(H, CV_32F);

    return !H.empty();
}

void Tracking::SolveRT(vector<cv::Point2f> &srcPoints, vector<cv::Point2f> &dstPoints, cv::Mat &R, cv::Mat &t, Mat &mask)
{
    cv::Mat E;
    //    cv::Mat fRANSAC = cv::findFundamentalMat(srcPoints, dstPoints, cv::FM_RANSAC, 0.5, 0.999, mask);
    //    fRANSAC.convertTo(fRANSAC, CV_32F);
    //    E= mK.t() * fRANSAC * mK;
    //    cout << E << endl;
    E = cv::findEssentialMat(srcPoints, dstPoints, mK, cv::RANSAC, 0.999, 1, mask);
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

    h = mK.inv() * H * mK; //implemented just the homography decomposition part from their paper into this section - this is based on method proposed in 2008 paper to estimate scale
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
    // d0 = 1.0 / sqrt(pow(X(1), 2.0) + pow(X(2), 2.0) + pow(X(3), 2.0));
    n = (cv::Mat_<float>(3, 1) << X(1, 0),
         X(2, 0),
         X(3, 0));
    n = n / cv::norm(n);
    // cout << "d0 is" << d0 << endl;
    // cout << "R is " << R << endl;
    // cout << "t is " << t << endl;
    // cout << "X is " << X << endl;

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
    vector<vector<cv::DMatch>> nn_matches;
    matcher.knnMatch(mLastFrame.mDescriptors, mCurrentFrame.mDescriptors, nn_matches, 2);

    vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < nn_matches.size(); i++)
    {
        if (nn_matches[i][0].distance < 0.85 * nn_matches[i][1].distance)
        {
            good_matches.push_back(nn_matches[i][0]);
        }
    }

    matches.swap(good_matches);

    // matcher.match(mLastFrame.mDescriptors, mCurrentFrame.mDescriptors, matches);
    vector<cv::Point2f> srcPoint, dstPoint;
    for (cv::DMatch pairMatch : matches)
    {
        srcPoint.emplace_back(mLastFrame.mvKeys[pairMatch.queryIdx].pt);
        dstPoint.emplace_back(mCurrentFrame.mvKeys[pairMatch.trainIdx].pt);
    }

    cv::Mat R, t, H, mask;

    // estimate R and t using srcPoint and dstPoint
    // filter srcPoint and dstPoint to only contain the inliers
    auto start = std::chrono::high_resolution_clock::now();

    SolveRT(srcPoint, dstPoint, R, t, mask);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    // cout << "duration of solvert " << duration << endl;
    FilterSrcAndDstPointsBasedOnMask(srcPoint, dstPoint, mask);
    //    cout << "Number of inlier after solveRT: " << inlierIndex.size() << endl;

    // estimate H
    // filter srcPoint and dstPoint to only contain the inliers
    // TODO: handle case if homography failed
    //auto start = chrono::high_resolution_clock::now();
    bool success = SolveH(srcPoint, dstPoint, H, mask, R, t);
    if (!success)
    {
        n = (cv::Mat_<float>(3, 1) << 0, 0, 1); //just give a ridiculous normal
        return 1.;
    }

    // auto end = std::chrono::high_resolution_clock::now();
    //    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    //        cout << "duration of homography " << duration << endl;

    FilterSrcAndDstPointsBasedOnMask(srcPoint, dstPoint, mask);

    //    cout << "Number of inlier after solveH: " << inlierIndex.size() << endl;

    // estimate d
    // auto start = std::chrono::high_resolution_clock::now();
    if (refR.empty() || refT.empty())
        InitialSolver(R, t, H, d, n);
    else
        InitialSolver(refR, refT, H, d, n);

    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    // cout << "duration of init solver " << duration << endl;

    // TODO: refine the height using the given equation from the paper + KalmanFilter

    // find scale
    float scale = actual_height / d;
    //    cout << "Estimated height: " << scaled_height << endl;
    //    cout << "SCALE: " << scale << endl;
    //return scale;
    return scale;
}

// Ground Plane based Absolute Scale Estimation for Monocular Visual Odometry
// This rescaling function is implementing the scale correction proposed by this paper
// https://arxiv.org/pdf/1903.00912.pdf
float Tracking::ManageScale()
{
    // I think that if you want to modify the KeyFrame and MapPoints, still need to make sure that no other thread is using.
    // I don't know how to turned off the GBA in Tracking thread, so I just turned off the loop closing thread
    // TODO: find ways to terminate GBA from Tracking thread

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop

    static int consecutive = 0;

    // First, we get the estimated scale
    float d;
    cv::Mat n;
    cv::Mat VR = mVelocity.rowRange(0, 3).colRange(0, 3);
    cv::Mat Vt = mVelocity.rowRange(0, 3).col(3);
    float scale = EstimateScale(d, n, VR, Vt);

    //cout << "current frame pose: \n" << mCurrentFrame.mTcw<< endl;
    float angle = CosineAngle(mPriorNormal, n) * 180. / M_PI;
    // cout << "new normal angle is " << angle * 180 / M_PI << endl;

    float scaledTx = cv::norm(mVelocity.col(3).rowRange(0, 3) * scale);
    mTxFile << mCurrentFrame.mnId << "," << scaledTx << endl;

    // If the angle between normal from estimation and the last normal is greater that 5 degrees, do not rescale
    if (angle < 7.5)
    //  if(acos(n.dot(prevNormal)/(norm(n)*norm(prevNormal))) < NORMAL_ANGLE_THRESHOLD)
    //  if(acos(n.dot(prevNormal)/(norm(n)*norm(prevNormal))) < NORMAL_ANGLE_THRESHOLD && fabs(oldScale/scale - 1) > 0.075)
    {
        consecutive++;  

        if (mWindow.size() >= mWeights.size()) 
            mWindow.erase(mWindow.begin()); //rm first element

        mWindow.push_back(scale);
        cout << "pushing scale:" << scale * cv::norm(Vt) << endl;

        if (mWindow.size() != mWeights.size())
        {
            return 1.;
        }

        float wSum = std::accumulate(mWeights.begin(), mWeights.end(), 0.);
        float wAvg = 0.;
        for (int i = 0; i < mWindow.size(); i++)
        {
            wAvg += mWindow[i] * mWeights[i];
        }

        wAvg /= wSum;

        // if (consecutive >= 2)
            return wAvg;
        // else
            // return 1.;
        // Since the scale is good, we accept the scale and perform the rescaling
        // cout << "Scale accepted" << endl;
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

        // cout << mCurrentFrame.mTcw << endl << "Reference:" << mCurrentFrame.mpReferenceKF->mnFrameId << endl;
        // cout << "compare " << mpLastKeyFrame->GetPose() << mCurrentFrame.mTcw << endl;

        // float scaledTx = cv::norm(mVelocity.col(3).rowRange(0, 3) * scale);
        // mTxFile << mCurrentFrame.mnId << "," << scaledTx << endl;

        if (mCurrentFrame.mTimeStamp - mTimeStampLastUpdate > SCALE_UPDATE_PERIOD)
        {
        }
    }
    else
    {
        consecutive = 0; //reset consecutive
        return 1.;
    }

    // bNeedRescale = false;
    // bool test = false;
    // Optimizer::LocalBundleAdjustment(mpLastKeyFrame, &test, mpMap);
}
} // namespace ORB_SLAM2

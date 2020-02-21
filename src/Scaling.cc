#include "Tracking.h"
#include "Converter.h"

namespace ORB_SLAM2
{

bool CompareKeyFrameId(KeyFrame *kf1, KeyFrame *kf2)
{
    return (kf1->mnFrameId < kf2->mnFrameId);
}

void Tracking::FilterSrcAndDstPointsBasedOnMask(vector<cv::Point2f> &srcPoints, vector<cv::Point2f> &dstPoints, cv::Mat mask)
{
    int idx = 0; // idx will be the first index in which the mask = 0, also corresponds to the number of points with mask = 1
    for (MatConstIterator_<double> it = mask.begin<double>(); it != mask.end<double>(); ++it)
    {
        if (*it > 0)
        {
            srcPoints[idx] = srcPoints[it - mask.begin<double>()];
            dstPoints[idx++] = dstPoints[it - mask.begin<double>()];
        }
    }
    srcPoints.resize(idx);
    dstPoints.resize(idx);
}

bool Tracking::SolveH(vector<Point2f> &srcPoint, vector<Point2f> &dstPoint, cv::Mat &H, Mat &mask)
{
    // This ROI defines the ground plane in front of the vehicle. The paper suggests that a basic road detector is used
    // to pre-determine the ROI needed, but for now, we just use the ROI defined in the author's code.
    vector<Point2f> ROI = {Point2f(500, 230), Point2f(700, 230), Point2f(800, 380), Point2f(400, 380)};

    vector<Point2f> srcPointROI, dstPointROI;

    for (int i = 0; i < (int)srcPoint.size(); ++i)
    {
        // ISSUES: We only check if the source point is in ROI or not
        //         upon investigation, it is found that some of the point is match to the cloud etc.
        //         this might or might not affect findHomography badly
        // If srcPoint[i] is in / on the polygon defined in ROI, push both src and dst Point.
        if (cv::pointPolygonTest(ROI, srcPoint[i], false) >= 0)
        {
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

    if (isfinite(H.at<float>(0, 0)))
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

void Tracking::SolveRT(vector<cv::Point2f> &srcPoints, vector<cv::Point2f> &dstPoints, cv::Mat &R, cv::Mat &t, Mat &mask)
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
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
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
    n = (cv::Mat_<float>(3, 1) << X(1, 0),
         X(2, 0),
         X(3, 0));
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
    vector<vector<cv::DMatch>> nn_matches;
    matcher.knnMatch(mLastFrame.mDescriptors, mCurrentFrame.mDescriptors, nn_matches, 2);

    vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < nn_matches.size(); i++)
    {
        DMatch first = nn_matches[i][0];
        float dist1 = nn_matches[i][0].distance;
        float dist2 = nn_matches[i][1].distance;
        if (dist1 < 0.80 * dist2)
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
    bool retval = SolveH(srcPoint, dstPoint, H, mask);

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
void Tracking::Rescale()
{
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
    while (!mpLocalMapper->isStopped())
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
    cv::Mat VR = V.rowRange(0, 3).colRange(0, 3);
    cv::Mat Vt = V.rowRange(0, 3).col(3);
    float scale = EstimateScale(d, n, VR, Vt);
    //float scale = EstimateScale(d,n);
    // cout << "Vt:" << endl << Vt << endl;
    cout << "Scale, previous scale, angle: " << scale << " " << oldScale << " " << acos(n.dot(prevNormal) / (norm(n) * norm(prevNormal))) * 180 / M_PI << endl;
    cout << "Current NOrmal: " << n << endl;

    //cout << "current frame pose: \n" << mCurrentFrame.mTcw<< endl;
    float angle = CosineAngle(mPriorNormal, n);
    cout << "new normal angle is " << angle * 180 / M_PI << endl;

    // If the angle between normal from estimation and the last normal is greater that 5 degrees, do not rescale
    if (angle < NORMAL_ANGLE_THRESHOLD)
    //  if(acos(n.dot(prevNormal)/(norm(n)*norm(prevNormal))) < NORMAL_ANGLE_THRESHOLD)
    //  if(acos(n.dot(prevNormal)/(norm(n)*norm(prevNormal))) < NORMAL_ANGLE_THRESHOLD && fabs(oldScale/scale - 1) > 0.075)
    {
        // Since the scale is good, we accept the scale and perform the rescaling
        cout << "Scale accepted" << endl;
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

        std::vector<KeyFrame *> sortedLocalKeyFrames(mvpLocalKeyFrames);
        sort(sortedLocalKeyFrames.begin(), sortedLocalKeyFrames.end(), CompareKeyFrameId);

        ofstream beforeFile("before.txt"), afterFile("after.txt");
        std::vector<KeyFrame *> mykfs = mpMap->GetAllKeyFrames();
        for (int i = 0; i < mykfs.size(); i++)
        {
            KeyFrame *pKF = mykfs[i];
            if (pKF->isBad())
                continue;
            cv::Mat t = pKF->GetCameraCenter();
            beforeFile << t.at<float>(0) << "," << t.at<float>(1) << "," << t.at<float>(2) << endl;
        }

        // This is for the KeyFrames
        KeyFrame *firstFrame = sortedLocalKeyFrames[0];
        cv::Mat Twf = firstFrame->GetPoseInverse();
        cv::Mat Tfw = firstFrame->GetPose();
        cv::Mat Rfw = Tfw.rowRange(0, 3).colRange(0, 3);
        cv::Mat tfw = Tfw.rowRange(0, 3).col(3);
        g2o::Sim3 g2oSfw(Converter::toMatrix3d(Rfw), Converter::toVector3d(tfw), 1.);

        // for(int i = 0 ; i < (int)mvpLocalKeyFrames.size() ; i++)
        for (int i = 1; i < sortedLocalKeyFrames.size(); i++)
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
            g2o::Sim3 g2oSif(Converter::toMatrix3d(Rif), Converter::toVector3d(tif), 1 / scale);

            g2o::Sim3 g2oSiwCorrected = g2oSif * g2oSfw;

            //Tif.rowRange(0, 3).col(3) = Tif.rowRange(0,3).col(3) * scale;//  / oldScale;
            //cv::Mat TiwCorrected = Tif * firstFrame->GetPose();
            Eigen::Matrix3d eigR = g2oSiwCorrected.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oSiwCorrected.translation();
            double s = g2oSiwCorrected.scale();
            eigt *= 1. / s;
            cv::Mat TiwCorrected = Converter::toCvSE3(eigR, eigt);
            // cout << "BEFORE:" << Tiw << endl << "AFTER:" << TiwCorrected << endl;
            sortedLocalKeyFrames[i]->SetPose(TiwCorrected);

            cv::Mat t = sortedLocalKeyFrames[i]->GetCameraCenter();
            afterFile << t.at<float>(0) << "," << t.at<float>(1) << "," << t.at<float>(2) << endl;

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

            vector<MapPoint *> mapPointsMatches = sortedLocalKeyFrames[i]->GetMapPointMatches();
            for (size_t indexMP = 0; indexMP < mapPointsMatches.size(); indexMP++)
            {
                MapPoint *mapPointi = mapPointsMatches[indexMP];
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
        Tcf.rowRange(0, 3).col(3) = Tcf.rowRange(0, 3).col(3) * scale; //  / oldScale;
        cv::Mat TcwCorrected = Tcf * firstFrame->GetPose();
        mpLastKeyFrame->SetPose(TcwCorrected);
        mCurrentFrame.SetPose(TcwCorrected);

        mpLastKeyFrame->UpdateConnections();

        //        cout << endl;

        beforeFile.close();
        afterFile.close();
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
        mVelocity.col(3).rowRange(0, 3) = mVelocity.col(3).rowRange(0, 3) * scale; // / oldScale;

        float cal_tx = cv::norm(mVelocity.col(3).rowRange(0, 3));
        mTxFile << mCurrentFrame.mnId << "," << cal_tx << endl;

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
} // namespace ORB_SLAM2
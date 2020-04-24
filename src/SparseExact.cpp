#include "SparseExact.h"

SparseExact::SparseExact()
{
    using namespace cv;
    roi_vertices = {Point2f(500, 230), Point2f(700, 230), Point2f(800 + 25, 380), Point2f(400 - 25, 380)};
}

cv::Mat SparseExact::solveExactNorm(ORB_SLAM2::Frame &f, ORB_SLAM2::Frame &f2, cv::Mat &K, cv::Mat &R, cv::Mat &t)
{
    using namespace cv;

    vector<Point2f> srcPt, dstPt;

    for (int i = 0; i < f.mvKeys.size(); i++)
    {
        if (cv::pointPolygonTest(roi_vertices, f.mvKeys[i].pt, false) >= 0)
        {
            if (f.mvpMapPoints[i] != NULL)
            {
                for (int j = 0; j < f2.mvKeys.size(); j++)
                {
                    if (f2.mvpMapPoints[j] != NULL && f.mvpMapPoints[i]->mnId == f2.mvpMapPoints[j]->mnId)
                    {
                        srcPt.push_back(f.mvKeys[i].pt);
                        dstPt.push_back(f2.mvKeys[j].pt);
                        cout << "frame " << f.mnId << " x: " << f.mvKeys[i].pt.x << " y: " << f.mvKeys[i].pt.y << " to x: " << f2.mvKeys[j].pt.x << " y: " << f2.mvKeys[j].pt.y << endl;
                    }
                }
            }
        }
    }

    Mat ret_points;
    {

        Mat I = Mat::eye(3, 3, CV_32F);
        //P0
        Mat ext0 = Mat::zeros(3, 4, CV_32F);
        I.copyTo(ext0.rowRange(0, 3).colRange(0, 3));
        Mat P0 = K * ext0;

        //P1
        Mat ext1 = Mat::zeros(3, 4, CV_32F);
        R.copyTo(ext1.rowRange(0, 3).colRange(0, 3));
        t.copyTo(ext1.rowRange(0, 3).col(3));
        Mat P1 = K * ext1;

        triangulatePoints(P0, P1, srcPt, dstPt, ret_points);
    }

    Mat ground_X(srcPt.size(), 3, CV_32F);
    for (int i = 0; i < srcPt.size(); ++i)
    {
        Mat pt = ret_points.col(i);
        pt /= pt.at<float>(3, 0); 
        Mat tmp = pt.rowRange(0, 3).col(0).t();
        tmp.copyTo(ground_X.row(i).colRange(0,3));        
    }

    cv::Mat all_model;
    cv::Mat all_b = Mat::ones(srcPt.size(), 1, CV_32F);

    if (srcPt.size() >= 3)
    {
        cout << "solving exact for " << srcPt.size() << " points" << endl;
        cv::solve(ground_X, all_b, all_model, DECOMP_SVD);
    }

    return all_model;
}
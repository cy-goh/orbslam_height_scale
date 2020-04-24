#include <opencv2/opencv.hpp>
#include "Frame.h"


class SparseExact
{
public:
    SparseExact();

    cv::Mat solveExactNorm(ORB_SLAM2::Frame &f1, ORB_SLAM2::Frame &f2, cv::Mat &K, cv::Mat &R, cv::Mat &t);

private:
    std::vector<cv::Point2f> roi_vertices;

};
#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>

#include "SparseEstimator.h"
#include "DenseEstimator.h"
#include "Frame.h"

class ScaleManager
{
public:
    ScaleManager(float ref_height, cv::Mat &K);

    void manageScale(ORB_SLAM2::Frame &f, ORB_SLAM2::Frame &f2, cv::Mat &img1, cv::Mat &img2, int frame_id, cv::Mat &refR, cv::Mat &t, cv::Mat &desc1, cv::Mat &desc2, std::vector<cv::KeyPoint> &kp1, std::vector<cv::KeyPoint> &kp2, bool &rescale, float &estimated_scale);

    void sanityCheck(ORB_SLAM2::Frame &f, ORB_SLAM2::Frame &f2);

    float GetScaleGt(int frame_id);
    // inline void setK(cv::Mat &K) {mK = K.clone(); }

private:
    void qAppend(float s);
    void qDebugAppend(float s);

    void normQAppend(Mat &n, int timestamp);

    float actual_height;
    cv::Mat mK;
    cv::Mat prior_normal;
    SparseScaleEstimator *sparse_est;
    DenseScaleEstimator *dense_est;

    std::deque<float> q, qDebug;
    std::deque<float> timestamps;
    std::deque<Mat> norm_history;
    const int q_size = 5;

    std::vector<cv::Point2f> sparseROI;
    std::vector<cv::Point2f> denseROI;
    vector<float> scales_gt;


    int last_rescaled;
};
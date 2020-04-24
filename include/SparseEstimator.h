#ifndef EB188B38_8F8D_47EA_9FA8_91D2A05A7AC8
#define EB188B38_8F8D_47EA_9FA8_91D2A05A7AC8

#include <vector>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;

class SparseScaleEstimator
{

public:
    SparseScaleEstimator(Mat &cam_mat, std::vector<Point2f> &ROI, float height);

    // void get_matches(Mat &img1, Mat &img2, std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2, Mat &desc1, Mat &desc2, std::vector<DMatch> &good);

    void get_matches(std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2, Mat &desc1, Mat &desc2, std::vector<DMatch> &good);

    void get_RT(std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2, std::vector<DMatch> &matches, int &inliers, Mat &R, Mat &t, Mat &mask, std::vector<Point2f> &srcPts, std::vector<Point2f> &dstPts);

    Mat triangulate_pairs(Mat &R, Mat &t, std::vector<Point2f> &srcPtFiltered, std::vector<Point2f> &dstPtFiltered);

    std::vector<int> filter_ground_ROI(std::vector<Point2f> &pts);

    float calculate_scale(Mat &img1, Mat &img2, int iterations, float accepted_ratio, Mat &op_norm, float &inlier_ratio);
    float calculate_scale(Mat &desc1, Mat &desc2, vector<KeyPoint> kp1, vector<KeyPoint> kp2, int iterations, float accepted_ratio, Mat &op_norm, float &inlier_ratio);
    float calculate_scale(Mat& refR, Mat &refT, Mat &desc1, Mat &desc2, vector<KeyPoint> kp1, vector<KeyPoint> kp2, int iterations, float accepted_ratio, Mat &op_norm, float &inlier_ratio);


    Mat run_ransac(Mat &ground_pts, int n_iterations, float inlier_threshold, float inlier_ratio, int &ransac_count, float &fitted_ratio);

private:
    float actual_height;
    std::vector<Point2f> roi_vertices;
    Mat K;
    Ptr<AKAZE> fdetector;
    BFMatcher matcher;
};

#endif /* EB188B38_8F8D_47EA_9FA8_91D2A05A7AC8 */

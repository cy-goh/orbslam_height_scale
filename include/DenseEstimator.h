#ifndef D28C4BC8_1C45_4635_A00D_B3BFA2F1CE7A
#define D28C4BC8_1C45_4635_A00D_B3BFA2F1CE7A

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;

class DenseScaleEstimator
{
public:
    DenseScaleEstimator(Mat &cam_mat, vector<Point2f> &ROI, float height);
    void get_image_gradient(Mat &img, Mat &gx, Mat &gy);

    float optimize_scale(Mat &R, Mat &t, float n1, float n2, float n3, float lamb, int max_iters, Mat &src_img, Mat &dst_img, Mat &op_norm,  bool verbose = false);

private:
    Mat homography(Mat &R, Mat &t, Mat &n);
    Eigen::SparseMatrix<float> eval_jacobian_I(cv::Mat &grad_x, cv::Mat &grad_y, vector<Point2f> &indices);

    Eigen::MatrixXf eval_jacobian_f(float H[9], vector<Point2f> &point_indices);

    Eigen::Matrix<float, 9, 3> eval_jacobian_g(Mat &R, Mat &t, float n1, float n2, float n3);

    Eigen::MatrixXf eval_jacobian(Mat &R, Mat &t, float n1, float n2, float n3, vector<Point2f> &point_indices, Mat &gx, Mat &gy, vector<Point2f> &transformed_indices);

    vector<Point2f> get_point_indices(cv::Size img_dim);
    void get_valid_indices(Mat &R, Mat &t, float n1, float n2, float n3, vector<Point2f> &point_indices, cv::Size img_dims, vector<bool> &valid_point_indices, vector<Point2f> & transformed_xys);

    // vector<float> get_observations(Mat &img, vector<Point2f> &indices);
    Eigen::VectorXf get_observations(Mat &img, vector<Point2f> &indices);

    Mat K;
    Mat K_inv;
    vector<Point2f> roi_vertices;
    float ref_height;
};

#endif

#include "DenseEstimator.h"
#include <algorithm>
#include <chrono>

DenseScaleEstimator::DenseScaleEstimator(Mat &cam_mat, vector<Point2f> &ROI, float height) : K(cam_mat), roi_vertices(ROI), ref_height(height)
{
    K_inv = cam_mat.inv();
}

void DenseScaleEstimator::get_image_gradient(Mat &img, Mat &gx, Mat &gy)
{
    cv::Scharr(img, gx, CV_32F, 1, 0);
    cv::Scharr(img, gy, CV_32F, 0, 1);
}

Mat DenseScaleEstimator::homography(Mat &R, Mat &t, Mat &n)
{
    // Mat test = t * n.t();
    // cout << "size fo test: " << test.rows <<  " " << test.cols << endl;
    Mat H = K * (R + t * n.t()) * K_inv;
    return H;
}

Eigen::SparseMatrix<float> DenseScaleEstimator::eval_jacobian_I(Mat &grad_x, Mat &grad_y, vector<Point2f> &indices)
{
    size_t n_points = indices.size();
    Eigen::SparseMatrix<float> JI(n_points, n_points * 2);
    //JI.reserve(n_points * 2);

    // vector<Eigen::Triplet<float> > tripletList;

    for (size_t i = 0; i < n_points; i++)
    {
        //round?
        Point2i pt = indices[i];
        int c = pt.x;
        int r = pt.y;
        // cout << r << " " << c << endl;
        
        // tripletList.push_back(Eigen::Triplet<float>(i, 2 * i, grad_x.at<float>(pt) ));
        // tripletList.push_back(Eigen::Triplet<float>(i, 2 * i + 1, grad_y.at<float>(pt) ));
        
        JI.coeffRef(i, 2 * i) += grad_x.at<float>(pt);
        JI.coeffRef(i, 2 * i + 1) += grad_y.at<float>(pt);
        // JI.insert(i, 2 * i) = grad_x.at<float>(pt);
        // JI.insert(i, 2 * i + 1) = grad_y.at<float>(pt);
    }
    
    // JI.setFromTriplets(tripletList.begin(), tripletList.end());
    JI.makeCompressed();
    return JI;
}

Eigen::MatrixXf DenseScaleEstimator::eval_jacobian_f(float h[9], vector<Point2f> &point_indices)
{
    
    Eigen::MatrixXf Jf = Eigen::MatrixXf::Zero(point_indices.size() * 2, 9);

    for (int row = 0; row < point_indices.size(); row++)
    {
        Point2f &pt = point_indices[row];
        float &x = pt.x;
        float &y = pt.y;

        float dbase  = (h[6] * x + h[7] * y + h[8]);
        Jf(2 * row, 0) = x / dbase; //(h[6] * x + h[7] * y + h[8]);
        Jf(2 * row, 1) = y / dbase; //(h[6] * x + h[7] * y + h[8]);
        Jf(2 * row, 2) = 1. / dbase; //(h[6] * x + h[7] * y + h[8]);

        float n = h[0] * x + h[1] * y + h[2];
        float d = pow(dbase, 2); //pow((h[6] * x + h[7] * y + h[8]), 2);
        float nd = n / d;

        Jf(2 * row, 6) = -nd * x;
        Jf(2 * row, 7) = -nd * y;
        Jf(2 * row, 8) = -nd;

        Jf(2 * row + 1, 3) = x / dbase; //(h[6] * x + h[7] * y + h[8]);
        Jf(2 * row + 1, 4) = y / dbase; //(h[6] * x + h[7] * y + h[8]);
        Jf(2 * row + 1, 5) = 1 / dbase; //(h[6] * x + h[7] * y + h[8]);
        float n2 = h[3] * x + h[4] * y + h[5];
        float n2d = n2 / d;
        Jf(2 * row + 1, 6) = -(n2d) * x;
        Jf(2 * row + 1, 7) = -(n2d) * y;
        Jf(2 * row + 1, 8) = -n2d;
    }

    return Jf;
}

Eigen::Matrix<float, 9, 3> DenseScaleEstimator::eval_jacobian_g(Mat &R, Mat &t, float n1, float n2, float n3)
{
    float f1 = K.at<float>(0, 0);
    float f2 = K.at<float>(1, 1);
    float c1 = K.at<float>(0, 2);
    float c2 = K.at<float>(1, 2);

    float fi1 = K_inv.at<float>(0, 0);
    float fi2 = K_inv.at<float>(1, 1);
    float ci1 = K_inv.at<float>(0, 2);
    float ci2 = K_inv.at<float>(1, 2);

    // Eigen::MatrixXf J = Eigen::MatrixXf::Zero(9, 3);
    Eigen::Matrix<float, 9, 3> J = Eigen::Matrix<float, 9, 3>::Zero();

    float x = t.at<float>(0, 0);
    float y = t.at<float>(1, 0);
    float z = t.at<float>(2, 0);

    float f00 = f1 * fi1 * (x) + c1 * fi1 * (z);
    float f01 = 0;
    float f02 = 0;

    float f10 = 0;
    float f11 = f1 * fi2 * (x) + c1 * fi2 * (z);
    float f12 = 0;

    float f20 = f1 * (ci1 * x) + c1 * (ci1 * z);
    float f21 = f1 * (ci2 * x) + c1 * (ci2 * z);
    float f22 = f1 * (x) + c1 * (z);

    float f30 = f2 * fi1 * (y) + c2 * fi1 * (z);
    float f31 = 0;
    float f32 = 0;

    float f40 = 0;
    float f41 = f2 * fi2 * (y) + c2 * fi2 * (z);
    float f42 = 0;

    float f50 = f2 * ci1 * (y) + c2 * ci1 * (z);
    float f51 = f2 * ci2 * (y) + c2 * ci2 * (z);
    float f52 = f2 * (y) + c2 * (z);

    float f60 = fi1 * (z);
    float f61 = 0;
    float f62 = 0;

    float f70 = 0;
    float f71 = fi2 * z;
    float f72 = 0;

    float f80 = ci1 * (z);
    float f81 = ci2 * (z);
    float f82 = z;

    J << f00, f01, f02, 
        f10, f11, f12,
        f20, f21, f22,
        f30, f31, f32,
        f40, f41, f42,
        f50, f51, f52,
        f60, f61, f62,
        f70, f71, f72,
        f80, f81, f82; 


    // J(0, 0) = f1 * fi1 * (x) + c1 * fi1 * (z);
    // J(0, 1) = 0;
    // J(0, 2) = 0;

    // J(1, 0) = 0;
    // J(1, 1) = f1 * fi2 * (x) + c1 * fi2 * (z);
    // J(1, 2) = 0;

    // J(2, 0) = f1 * (ci1 * x) + c1 * (ci1 * z);
    // J(2, 1) = f1 * (ci2 * x) + c1 * (ci2 * z);
    // J(2, 2) = f1 * (x) + c1 * (z);

    // J(3, 0) = f2 * fi1 * (y) + c2 * fi1 * (z);
    // J(3, 1) = 0;
    // J(3, 2) = 0;

    // J(4, 0) = 0;
    // J(4, 1) = f2 * fi2 * (y) + c2 * fi2 * (z);
    // J(4, 2) = 0;

    // J(5, 0) = f2 * ci1 * (y) + c2 * ci1 * (z);
    // J(5, 1) = f2 * ci2 * (y) + c2 * ci2 * (z);
    // J(5, 2) = f2 * (y) + c2 * (z);

    // J(6, 0) = fi1 * (z);
    // J(6, 1) = 0;
    // J(6, 2) = 0;

    // J(7, 0) = 0;
    // J(7, 1) = fi2 * z;
    // J(7, 2) = 0;

    // J(8, 0) = ci1 * (z);
    // J(8, 1) = ci2 * (z);
    // J(8, 2) = z;

    return J;
}

vector<Point2f> DenseScaleEstimator::get_point_indices(cv::Size img_dim)
{
    vector<Point2f> ret_indices;

    for (int r = 0; r < img_dim.height; ++r)
    {
        // for (int c = 0; c < img_dim.width; ++c)
        for (int c = 0; c < img_dim.width; c+=2)
        {
            Point2f p(c, r);
            if (pointPolygonTest(roi_vertices, p, false) >= 0)
                ret_indices.push_back(p);
        }
    }
    return ret_indices;
}

Eigen::VectorXf DenseScaleEstimator::get_observations(Mat &img, vector<Point2f> &indices)
{
    //vector<float> ret_pixels;
    Eigen::VectorXf ret_pixels(indices.size());

    for (int i = 0; i < indices.size(); ++i)
    {
        Point2i pi = indices[i];
        ret_pixels(i) = img.at<float>(pi);
        // ret_pixels.push_back(img.at<float>(pi));
    }

    return ret_pixels;
}

Eigen::MatrixXf DenseScaleEstimator::eval_jacobian(Mat &R, Mat &t, float n1, float n2, float n3, vector<Point2f> &point_indices, Mat &gx, Mat &gy, vector<Point2f> &transformed_indices)
{
    Mat n = (Mat_<float>(3, 1) << n1, n2, n3);
    Mat H = homography(R, t, n);

    auto start5 = chrono::steady_clock::now();
    Eigen::MatrixXf Jf = eval_jacobian_f((float *)H.data, point_indices);
    auto end5 = chrono::steady_clock::now();
    // cout << "Elapsed time in JF : "  << chrono::duration_cast<chrono::microseconds>(end5 - start5).count() << endl;
    
    start5 = chrono::steady_clock::now();
    Eigen::Matrix<float, 9, 3> Jg = eval_jacobian_g(R, t, n1, n2, n3);
    end5 = chrono::steady_clock::now();
    // cout << "Elapsed time in JG : "  << chrono::duration_cast<chrono::microseconds>(end5 - start5).count() << endl;
    
    start5 = chrono::steady_clock::now();
    Eigen::SparseMatrix<float> JI = eval_jacobian_I(gx, gy, transformed_indices);
    end5 = chrono::steady_clock::now();
    // cout << "Elapsed time in JI : "  << chrono::duration_cast<chrono::microseconds>(end5 - start5).count() << endl;

    start5 = chrono::steady_clock::now();
    Eigen::MatrixXf J = JI * (Jf * Jg);
        end5 = chrono::steady_clock::now();

    // cout << "Elapsed time in jacobian mult : "  << chrono::duration_cast<chrono::microseconds>(end5 - start5).count() << endl;

    return J;
}

float DenseScaleEstimator::optimize_scale(Mat &R, Mat &t, float n1, float n2, float n3, float lamb, int max_iters, Mat &src_img, Mat &dst_img, Mat &op_norm, bool verbose)
{
    vector<Point2f> xy = get_point_indices(src_img.size());

    Mat gx, gy;
    get_image_gradient(dst_img, gx, gy);

    float last_sse = 1000000.;

    Eigen::VectorXf observations = get_observations(src_img, xy);

    for (int i = 0; i < max_iters; ++i)
    {

        auto start1 = chrono::steady_clock::now();

        vector<Point2f> transformed_xy; 
        vector<bool> valid_xy_indices;

        auto start5 = chrono::steady_clock::now();
        get_valid_indices(R, t, n1, n2, n3, xy, src_img.size(), valid_xy_indices, transformed_xy);
        auto end5 = chrono::steady_clock::now();
        // cout << "Elapsed time in valid_indices : "  << chrono::duration_cast<chrono::milliseconds>(end5 - start5).count() << endl;

        auto star = chrono::steady_clock::now();
        Eigen::VectorXf expected_observations = get_observations(dst_img, transformed_xy);
        //TODO: by right should filter by valid_xy_indices
        // Eigen::VectorXf observations = get_observations(src_img, valid_xy);
        // Eigen::VectorXf observations = get_observations(src_img, xy);
        auto en = chrono::steady_clock::now();
        // cout << "Elapsed time in getobservations : "  << chrono::duration_cast<chrono::milliseconds>(en - star).count() << endl;


        // Eigen::MatrixXf J = eval_jacobian(R, t, n1, n2, n3, valid_xy, gx, gy, transformed_xy);
        auto start2 = chrono::steady_clock::now();


        Eigen::MatrixXf J = eval_jacobian(R, t, n1, n2, n3, xy, gx, gy, transformed_xy);
        auto end2 = chrono::steady_clock::now();
        // cout << "Elapsed time in jacobian : "  << chrono::duration_cast<chrono::milliseconds>(end2 - start2).count() << endl;

        auto start3 = chrono::steady_clock::now();

        Eigen::MatrixXf JT = J.transpose();
        Eigen::MatrixXf JJ = JT * J;

        Eigen::VectorXf err = observations - expected_observations;

        Eigen::VectorXf dv = JJ.diagonal();
        Eigen::MatrixXf d = dv.asDiagonal();
        Eigen::VectorXf delta = (JJ + lamb * d).inverse() * (JT * err);

        auto end3 = chrono::steady_clock::now();
        // cout << "Elapsed time in calculation : "  << chrono::duration_cast<chrono::milliseconds>(end3 - start3).count() << endl;

        n1 += delta(0);
        n2 += delta(1);
        n3 += delta(2);

        float sse = err.dot(err);

        if (sse <= last_sse)
        {
            lamb /= 3.;
            lamb = max(0.00001f, lamb);
        }
        else
        {
            lamb *= 2.;
            lamb = min(50.f, lamb);
        }

        last_sse = sse;

        if (verbose)
        {
            Mat normal = (Mat_<float>(3, 1) << n1, n2, n3);
            cout << "iter " << i;
            cout << " err: " << sse;
            cout << " current scale: " << ref_height * cv::norm(normal);
            cout << " lambda: " << lamb;
            cout << "  points: " << xy.size();
            cout << endl;
        }

        auto end1 = chrono::steady_clock::now();
        // cout << "Elapsed time in optimizeScale : "  << chrono::duration_cast<chrono::milliseconds>(end1 - start1).count() << endl;
        // cout << "================================================" << endl;

    }

    op_norm = (Mat_<float>(3, 1) << n1, n2, n3);
    float scale = ref_height * cv::norm(op_norm);

    return scale;
}

void DenseScaleEstimator::get_valid_indices(Mat &R, Mat &t, float n1, float n2, float n3, vector<Point2f> &point_indices, cv::Size img_dims, vector<bool> &valid_xy_indices, vector<Point2f> &transformed_xys)
{
     Mat normal = (Mat_<float>(3, 1) << n1, n2, n3);
     Mat H = homography(R, t, normal);

    valid_xy_indices.clear();
    transformed_xys.clear();

    auto start1 = chrono::steady_clock::now();

    Mat uv1(3, point_indices.size(), CV_32F);
    for (int i = 0; i < point_indices.size(); i++)
    {
        auto p = point_indices[i];
        // Mat cp = (Mat_<float>(3, 1) << p.x, p.y, 1);
        // cp.copyTo(uv1.col(i));

        uv1.at<float>(0, i) = p.x;
        uv1.at<float>(1, i) = p.y;
        uv1.at<float>(2, i) = 1;
    }

    auto end1 = chrono::steady_clock::now();
    // cout << "Elapsed time of validIndices (1): "  << chrono::duration_cast<chrono::microseconds>(end1 - start1).count() << endl;   


    // Mat ones = Mat::ones(1, point_indices.size(), CV_32F);
    // Mat uv1;
    // cv::vconcat(uv.t(), ones, uv1);
    auto start2 = chrono::steady_clock::now();
    Mat transformed_ = H * uv1;
    auto end2 = chrono::steady_clock::now();
    // cout << "Elapsed time of validIndices (2): "  << chrono::duration_cast<chrono::microseconds>(end2 - start2).count() << endl;   

    auto start3 = chrono::steady_clock::now();

    transformed_.row(0) /= transformed_.row(2);
    transformed_.row(1) /= transformed_.row(2);
    // transformed_.row(2) /= transformed_.row(2);

    for (int i = 0; i < point_indices.size(); i++)
    {
        //Mat col = transformed_.col(i);
        // float s = col.at<float>(2, 0);
        float x = transformed_.at<float>(0, i);// / s;
        float y = transformed_.at<float>(1, i);// / s;

        Point2f pt(x,  y);
        transformed_xys.push_back(pt);

        bool within_img = y < img_dims.height;
        valid_xy_indices.push_back(within_img);

        if (!within_img)
        {
            string err = "Out of range: " + to_string(y);
            throw err;
        }
    }

    auto end3 = chrono::steady_clock::now();
    // cout << "Elapsed time of validIndices (3): "  << chrono::duration_cast<chrono::microseconds>(end3 - start3).count() << endl;   
}

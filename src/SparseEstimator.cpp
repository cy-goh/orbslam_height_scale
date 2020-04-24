#include "SparseEstimator.h"
#include "Utils.h"
#include <random>
#include <algorithm>
#include <chrono>

SparseScaleEstimator::SparseScaleEstimator(cv::Mat &cam_mat, std::vector<Point2f> &ROI, float height) : K(cam_mat), roi_vertices(ROI), actual_height(height)
{
    fdetector = AKAZE::create();
    fdetector->setThreshold(0.0001);
    matcher = BFMatcher(cv::NORM_HAMMING);
}

void SparseScaleEstimator::get_matches(std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2, Mat &desc1, Mat &desc2, std::vector<DMatch> &good)
{
    using namespace std;
    vector<vector<DMatch>> nn_matches;

    // fdetector->detectAndCompute(img1, Mat(), kp1, desc1);
    // fdetector->detectAndCompute(img2, Mat(), kp2, desc2);

    matcher.knnMatch(desc1, desc2, nn_matches, 2);

    for (size_t i = 0; i < nn_matches.size(); ++i)
    {
        if (nn_matches[i][0].distance < 0.8 * nn_matches[i][1].distance)
        {
            good.push_back(nn_matches[i][0]);
        }
    }
}

void SparseScaleEstimator::get_RT(std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2, std::vector<DMatch> &matches, int &inliers, Mat &R, Mat &t, Mat &mask, std::vector<Point2f> &srcPts, std::vector<Point2f> &dstPts)
{
    for (auto m : matches)
    {
        srcPts.push_back(kp1[m.queryIdx].pt);
        dstPts.push_back(kp2[m.trainIdx].pt);
    }

    Mat E = cv::findEssentialMat(srcPts, dstPts, K, cv::RANSAC, 0.999, 1, mask);
    inliers = cv::recoverPose(E, srcPts, dstPts, K, R, t, mask);
}

Mat SparseScaleEstimator::triangulate_pairs(Mat &R, Mat &t, std::vector<Point2f> &srcPtFiltered, std::vector<Point2f> &dstPtFiltered)
{
    Mat ret_points;

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

    triangulatePoints(P0, P1, srcPtFiltered, dstPtFiltered, ret_points);
    return ret_points;
}

std::vector<int> SparseScaleEstimator::filter_ground_ROI(std::vector<Point2f> &pts)
{
    vector<int> ret_indices;
    for (int i = 0; i < pts.size(); ++i)
    {
        if (cv::pointPolygonTest(roi_vertices, pts[i], false) >= 0)
            ret_indices.push_back(i);
    }

    return ret_indices;
}

float SparseScaleEstimator::calculate_scale(Mat &img1, Mat &img2, int iterations, float accepted_ratio, Mat &op_norm, float &inlier_ratio)
{
    std::vector<KeyPoint> kp1, kp2;
    std::vector<DMatch> matches;
    Mat desc1, desc2;

    auto start1 = chrono::steady_clock::now();

    fdetector->detectAndCompute(img1, Mat(), kp1, desc1);
    fdetector->detectAndCompute(img2, Mat(), kp2, desc2);

    float ret_scale = calculate_scale(desc1, desc2, kp1, kp2, iterations, accepted_ratio, op_norm, inlier_ratio);

    return ret_scale;
}

float SparseScaleEstimator::calculate_scale(Mat &desc1, Mat &desc2, vector<KeyPoint> kp1, vector<KeyPoint> kp2, int iterations, float accepted_ratio, Mat &op_norm, float &inlier_ratio)
{
    std::vector<DMatch> matches;
    get_matches(kp1, kp2, desc1, desc2, matches);

    int inliers;
    Mat R, t, mask;
    vector<Point2f> src_pts, dst_pts;
    get_RT(kp1, kp2, matches, inliers, R, t, mask, src_pts, dst_pts);

    {
        int idx = 0; // idx will be the first index in which the mask = 0, also corresponds to the number of points with mask = 1
        for (MatConstIterator_<uchar> it = mask.begin<uchar>(); it != mask.end<uchar>(); ++it)
        {
            if (*it > 0)
            {
                src_pts[idx] = src_pts[it - mask.begin<uchar>()];
                dst_pts[idx++] = dst_pts[it - mask.begin<uchar>()];
            }
        }
        src_pts.resize(idx);
        dst_pts.resize(idx);
    }

    //TODO: this is not yet transpsoe
    Mat pts_3D;
    try
    {
        pts_3D = triangulate_pairs(R, t, src_pts, dst_pts);
    }
    catch (Exception e)
    {
        return -1;
    }
    vector<int> ground_idx = filter_ground_ROI(src_pts);
    size_t n_points = ground_idx.size();

    // cout << "ground points: " << n_points << endl;
    if (n_points < 8)
    {
        return -1;
    }

    Mat ground_X(n_points, 3, CV_32F);
    for (int i = 0; i < n_points; ++i)
    {
        Mat pt = pts_3D.col(ground_idx[i]);
        pt /= pt.at<float>(3, 0);
        Mat tmp = pt.rowRange(0, 3).col(0).t();
        tmp.copyTo(ground_X.row(i).colRange(0, 3));
    }

    // cout << "There are " << n_points << " points" << endl;

    //run ransac
    std::random_device rd;
    std::mt19937 g(rd());

    cv::Mat b = cv::Mat::ones(3, 1, CV_32F);

    float err_threshold = .2;
    // float best_error = 10000000;
    float ret_scale = -1.;
    int times_compared = 0;

    auto start2 = chrono::steady_clock::now();

    int ransac_count = 0;
    op_norm = run_ransac(ground_X, iterations, err_threshold, accepted_ratio, ransac_count, inlier_ratio);

    if (!op_norm.empty())
        ret_scale = actual_height * cv::norm(op_norm);
    else
    {
        float small_angle_thres = .5;
        vector<int> valid_indices;

        Mat t2;
        t.convertTo(t2, CV_32F);

        for (int i = 0; i < n_points; ++i)
        {
            Mat p3 = ground_X.row(i);

            // cout << t << endl;
            // cout << p3 << endl;

            Mat p31 = -p3;
            Mat p32 = t2.t() - p3;

            float tri_angle = CosineAngle(p31, p32) * 180. / M_PI;
            if (tri_angle > small_angle_thres)
            {
                valid_indices.push_back(i);
            }
        }

        cout << "RANSAC retry: Reducing points from " << n_points << " to " << valid_indices.size() << endl;
        if (valid_indices.size() >= 8)
        {
            n_points = valid_indices.size();

            cv::Mat reduced_X(valid_indices.size(), 3, CV_32F);
            for (int i = 0; i < valid_indices.size(); ++i)
            {
                ground_X.row(valid_indices[i]).copyTo(reduced_X.row(i));
            }

            op_norm = run_ransac(reduced_X, iterations, err_threshold, accepted_ratio, ransac_count, inlier_ratio);
            if (!op_norm.empty())
                ret_scale = actual_height * cv::norm(op_norm);
        }
    }
    /*for (int i = 0; i < iterations; ++i)
    {
        //random sample 3 numbers
        vector<int> sample_indices;
        for (int j = 0; j < n_points; ++j)
            sample_indices.push_back(j);

        shuffle(sample_indices.begin(), sample_indices.end(), g);

        Mat p(3, 3, CV_32F);

        ground_X.row(sample_indices[0]).copyTo(p.row(0));
        ground_X.row(sample_indices[1]).copyTo(p.row(1));
        ground_X.row(sample_indices[2]).copyTo(p.row(2));

        //calculate model
        Mat model = p.inv() * b;
        float model_norm = cv::norm(model);
        float height = 1 / model_norm;
        Mat unit_model = model / model_norm;

        int inliers_count = 0;
        vector<int> inlier_indices;

        for (int j = 3; j < sample_indices.size(); ++j)
        {
            Mat other_pt = ground_X.row(sample_indices[j]);

            float pt_height = other_pt.t().dot(unit_model);
            if (fabs(pt_height - height) < err_threshold)
            {
                inliers_count++;
                inlier_indices.push_back(sample_indices[j]);
            }
        }

        // cout << "inliers: " << inliers_count << "(" <<  inliers_count * 1. / n_points << ")" <<  endl;

        //refit with all inlier points
        if ((inliers_count * 1. / (n_points - 3)) > accepted_ratio)
        {
            inlier_indices.push_back(sample_indices[0]);
            inlier_indices.push_back(sample_indices[1]);
            inlier_indices.push_back(sample_indices[2]);
            inliers_count += 3;

            cv::Mat all_P = cv::Mat::zeros(inliers_count, 3, CV_32F);
            for (int j = 0; j < inliers_count; ++j)
            {
                ground_X.row(inlier_indices[j]).copyTo(all_P.row(j));
            }

            cv::Mat all_model;
            cv::Mat all_b = Mat::ones(inliers_count, 1, CV_32F);
            cv::solve(all_P, all_b, all_model, DECOMP_SVD);

            float all_model_norm = cv::norm(all_model);
            float all_height = 1. / all_model_norm;
            Mat unit_all_model = all_model / all_model_norm;

            Mat ys = all_P * (unit_all_model);
            ys -= all_height;
            float avg_hdiff = ys.dot(ys) / inliers_count;
            avg_hdiff = sqrt(avg_hdiff);

            if (avg_hdiff < best_error)
            {
                best_error = avg_hdiff;
                ret_scale = actual_height / all_height;
                op_norm = all_model.clone();
                inlier_ratio = inliers_count * 1. / n_points;

                times_compared++;
            }
        }
    }*/

    //auto end2 = chrono::steady_clock::now();
    // cout << "Elapsed time in ransac : "
    // 	<< chrono::duration_cast<chrono::milliseconds>(end2 - start2).count()
    // 	<< " ms" << endl;

    cout << "ransac compared:" << ransac_count << " points: " << n_points << endl;
    return ret_scale;
}

Mat SparseScaleEstimator::run_ransac(Mat &ground_pts, int n_iterations, float inlier_threshold, float accepted_ratio, int &ransac_count, float &fitted_ratio)
{
    //output norm
    Mat op_norm;

    //ransac related
    std::random_device rd;
    std::mt19937 g(rd());
    float best_error = 10000000;
    cv::Mat b = cv::Mat::ones(3, 1, CV_32F);

    int n_points = ground_pts.rows;

    for (int i = 0; i < n_iterations; ++i)
    {
        //random sample 3 numbers
        vector<int> sample_indices;
        for (int j = 0; j < n_points; ++j)
            sample_indices.push_back(j);

        shuffle(sample_indices.begin(), sample_indices.end(), g);

        Mat p(3, 3, CV_32F);

        ground_pts.row(sample_indices[0]).copyTo(p.row(0));
        ground_pts.row(sample_indices[1]).copyTo(p.row(1));
        ground_pts.row(sample_indices[2]).copyTo(p.row(2));

        //calculate model
        Mat model = p.inv() * b;
        float model_norm = cv::norm(model);
        float height = 1 / model_norm;
        Mat unit_model = model / model_norm;

        int inliers_count = 0;
        vector<int> inlier_indices;

        for (int j = 3; j < sample_indices.size(); ++j)
        {
            Mat other_pt = ground_pts.row(sample_indices[j]);

            float pt_height = other_pt.t().dot(unit_model);
            if (fabs(pt_height - height) < inlier_threshold)
            {
                inliers_count++;
                inlier_indices.push_back(sample_indices[j]);
            }
        }

        // cout << "inliers: " << inliers_count << "(" <<  inliers_count * 1. / n_points << ")" <<  endl;

        //refit with all inlier points
        if ((inliers_count * 1. / (n_points - 3)) > accepted_ratio)
        {
            inlier_indices.push_back(sample_indices[0]);
            inlier_indices.push_back(sample_indices[1]);
            inlier_indices.push_back(sample_indices[2]);
            inliers_count += 3;

            cv::Mat all_P = cv::Mat::zeros(inliers_count, 3, CV_32F);
            for (int j = 0; j < inliers_count; ++j)
            {
                ground_pts.row(inlier_indices[j]).copyTo(all_P.row(j));
            }

            cv::Mat all_model;
            cv::Mat all_b = Mat::ones(inliers_count, 1, CV_32F);
            cv::solve(all_P, all_b, all_model, DECOMP_SVD);

            float all_model_norm = cv::norm(all_model);
            float all_height = 1. / all_model_norm;
            Mat unit_all_model = all_model / all_model_norm;

            Mat ys = all_P * (unit_all_model);
            ys -= all_height;
            float avg_hdiff = ys.dot(ys) / inliers_count;
            avg_hdiff = sqrt(avg_hdiff);

            if (avg_hdiff < best_error)
            {
                best_error = avg_hdiff;
                // ret_scale = actual_height / all_height;
                op_norm = all_model.clone();
                fitted_ratio = inliers_count * 1. / n_points;
            }

            ransac_count++;
        }
    }

    return op_norm;
}
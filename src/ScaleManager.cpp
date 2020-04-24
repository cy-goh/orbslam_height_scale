#include "ScaleManager.h"
#include "Utils.h"
#include "SparseExact.h"
#include <iostream>
#include <fstream>

ScaleManager::ScaleManager(float ref_height, cv::Mat &K) : actual_height(ref_height), mK(K), last_rescaled(0)
{
    sparseROI = {Point2f(500, 230), Point2f(700, 230), Point2f(800 + 40, 380), Point2f(400 - 40, 380)};

    denseROI = {Point2f(500, 230), Point2f(700, 230), Point2f(800, 330), Point2f(400, 330)};

    dense_est = new DenseScaleEstimator(mK, denseROI, actual_height);
    sparse_est = new SparseScaleEstimator(mK, sparseROI, actual_height);

    const float pitch_rot = 4. * M_PI / 180.;
    Eigen::Vector4f v(0, 1, 0, 1);
    Eigen::Matrix4f eigRot = Eigen::Matrix4f::Identity();
    // eigRot(1, 1) = cos(pitch_rot);
    // eigRot(1, 2) = -sin(pitch_rot);
    // eigRot(2, 1) = sin(pitch_rot);
    // eigRot(2, 2) = cos(pitch_rot);
    Eigen::Vector4f eigN = eigRot * v;
    prior_normal = (cv::Mat_<float>(3, 1) << eigN(0), eigN(1), eigN(2));

    ifstream gt;
    gt.open(("/home/cy/Documents/data/kitti/data_odometry_poses/06.txt"));
    if (!gt)
    {
        cout << "unable to open gt file" << endl;
        exit(1);
    }

    float x = -1, y = -1, z = -1;
    while (true)
    {
        float _, cx, cy, cz;

        gt >> _;
        gt >> _;
        gt >> _;
        gt >> cx;
        gt >> _;
        gt >> _;
        gt >> _;
        gt >> cy;
        gt >> _;
        gt >> _;
        gt >> _;
        gt >> cz;

        if (z == -1 && y == -1 && z == -1)
        {
            x = cx;
            y = cy;
            z = cz;
        }
        else
        {
            float scale = sqrt(pow(cx - x, 2) + pow(cy - y, 2) + pow(cz - z, 2));
            scales_gt.push_back(scale);
            x = cx;
            y = cy;
            z = cz;
        }

        if (gt.eof())
            break;
    }
}

float ScaleManager::GetScaleGt(int frame_id)
{
    return scales_gt[frame_id];
}

void ScaleManager::qAppend(float s)
{
    q.push_back(s);

    // q.push(s);

    if (q.size() > q_size)
    {
        q.pop_front();
    }
}

void ScaleManager::normQAppend(Mat &n, int timestamp)
{
    norm_history.push_back(n);
    timestamps.push_back(timestamp);

    if (norm_history.size() > q_size)
    {
        norm_history.pop_front();
        timestamps.pop_front();
    }
}

void ScaleManager::qDebugAppend(float s)
{
    qDebug.push_back(s);

    // q.push(s);

    if (qDebug.size() > q_size)
    {
        qDebug.pop_front();
    }
}

void ScaleManager::manageScale(ORB_SLAM2::Frame &f1, ORB_SLAM2::Frame &f2, cv::Mat &img1, cv::Mat &img2, int frame_id, cv::Mat &refR, cv::Mat &t, cv::Mat &desc1, cv::Mat &desc2, std::vector<cv::KeyPoint> &kp1, std::vector<cv::KeyPoint> &kp2, bool &rescale, float &estimated_scale)
{
    using namespace cv;
    using namespace std;

    float sparse_ang_thres = 10.;
    float dense_ang_thres = 20.;
    float sparse_inlier_thres = 0.60;
    int rescale_freq = 1;

    Mat est_norm;
    float inlier_ratio;

    // float scale = sparse_est->calculate_scale(img1, img2, 100, 0.60, est_norm, inlier_ratio);
    cout << "ORB R: \n"
         << refR << endl;
    cout << "ORB t: \n"
         << t << endl;

    float refT_scale = cv::norm(t);
    float scale = sparse_est->calculate_scale(refR, t, desc1, desc2, kp1, kp2, 3 * 150, 0.70, est_norm, inlier_ratio);
    float scale_unit = scale * refT_scale;

    // SparseExact sparse_est;
    // cv::Mat est_norm = sparse_est.solveExactNorm(f1, f2, mK, refR, t);

    if (scale == -1)
    // if (est_norm.empty())
    {
        cout << "Unsuccessful sparse estimation" << endl;
        rescale = false;
        estimated_scale = -1;
        qAppend(estimated_scale);
        return;
    }
    else
    {
        cout << "Sparse scale: " << scale_unit << endl;
        // cout << "Sparse scale: " << scale << endl;
    }
    float angle = CosineAngle(prior_normal, est_norm) * 180. / M_PI;

    // cout << "Angle is " << angle << " inliers: " << inlier_ratio << endl;

    // if (angle > sparse_ang_thres || inlier_ratio < sparse_inlier_thres || scale > 2.)
    // if (angle > sparse_ang_thres )
    if (angle > sparse_ang_thres || inlier_ratio < sparse_inlier_thres || scale_unit > 2.)
    {
        cout << "Bad sparse output: (angle)" << angle << " (scale): " << scale_unit << endl;
        rescale = false;
        estimated_scale = -1;
        qAppend(estimated_scale);
        return;
    }

    int m_count = 1;
    Mat avg_norm = est_norm.clone(); //  / cv::norm(t);
    for (int widx = 0; widx < norm_history.size(); widx++)
    {
        if (frame_id - timestamps[widx] <= 5)
        {
            m_count += 1;
            avg_norm += norm_history[widx];
        }
    }

    avg_norm /= m_count;
    float avg_angle = CosineAngle(prior_normal, avg_norm) * 180 / M_PI;

    float n1, n2, n3;
    if (avg_angle < 7.5)
    {
        n1 = avg_norm.at<float>(0, 0);
        n2 = avg_norm.at<float>(1, 0);
        n3 = avg_norm.at<float>(2, 0);
        cout << "running average of " << m_count << " with scale of " << actual_height * cv::norm(avg_norm) * refT_scale << "org:" << scale_unit << endl;
    }
    else
    {
        // est_norm /= cv::norm(t);
        n1 = est_norm.at<float>(0, 0);
        n2 = est_norm.at<float>(1, 0);
        n3 = est_norm.at<float>(2, 0);
    }

    // n1 /= cv::norm(t);
    // n2 /= cv::norm(t);
    // n3 /= cv::norm(t);

    Mat final_norm;
    float final_scale;

    try
    {
        Mat img1f = img1.clone();
        Mat img2f = img2.clone();
        img1f.convertTo(img1f, CV_32F);
        img2f.convertTo(img2f, CV_32F);
        img1f /= 255.;
        img2f /= 255.;

        final_scale = dense_est->optimize_scale(refR, t, n1, n2, n3, 0, 35, img1f, img2f, final_norm, false);
    }
    catch (std::string &err)
    {
        cout << "homography failed. " << err << endl;
        rescale = false;
        estimated_scale = -1;
        qAppend(estimated_scale);
        return;
    }

    float dense_angle = CosineAngle(prior_normal, final_norm) * 180 / M_PI;
    if (dense_angle < 20)
    {
        cout << "Good dense scale at " << final_scale * refT_scale << endl;
        estimated_scale = final_scale;
        qAppend(estimated_scale * refT_scale);

        int invalid = 0;
        int valid = 0;
        float sum = 0.;
        for (auto it = q.begin(); it != q.end(); ++it)
        {
            if (*it == -1)
                invalid++;
            else
            {
                valid++;
                sum += *it;
            }
        }

        if (invalid <= 1 && q.size() == q_size && frame_id - last_rescaled > rescale_freq)
        {
            estimated_scale = sum / valid;
            // rescale = true;
            cout << "average scale over " << valid << "frame: " << estimated_scale /** cv::norm(t)*/ << endl;

            // q.clear();

            last_rescaled = frame_id;
            estimated_scale /= refT_scale;

            if (fabs(estimated_scale - 1) > 0.075)
            {
                rescale = true;
            }

            // estimated_scale = GetScaleGt(frame_id) / cv::norm(t);
        }
        // else
        // cout << "invalid:" << invalid <<  " q: " << q.size() << " time pase: " << frame_id - last_rescaled << endl;

        // final_norm *= cv::norm(t);
        normQAppend(final_norm, frame_id);
    }
    else
    {
        cout << "bad dense angle: " << dense_angle << endl;
        estimated_scale = -1;
        rescale = false;
        qAppend(estimated_scale);
    }
}

void ScaleManager::sanityCheck(ORB_SLAM2::Frame &f, ORB_SLAM2::Frame &f2)
{
    int count = 0;
    for (auto kp : f.mvKeys)
    {
        if (cv::pointPolygonTest(sparseROI, kp.pt, false) >= 0)
            count++;
    }

    cout << "Frame: " << f.mnId << " points inside ROI :" << count << " total: " << f.mvKeys.size() << endl;

    int associated = 0;

    for (int i = 0; i < f.mvKeys.size(); i++)
    {
        if (cv::pointPolygonTest(sparseROI, f.mvKeys[i].pt, false) >= 0)
        {
            if (f.mvpMapPoints[i] != NULL)
            {
                for (int j = 0; j < f2.mvKeys.size(); j++)
                {
                    if (f2.mvpMapPoints[j] != NULL && f.mvpMapPoints[i]->mnId == f2.mvpMapPoints[j]->mnId)
                    {
                        associated++;
                    }
                }
            }
        }
    }

    cout << "Frame: " << f.mnId << " points inside ROI :" << count << " associated:" << associated << " total: " << f.mvKeys.size() << endl;
}
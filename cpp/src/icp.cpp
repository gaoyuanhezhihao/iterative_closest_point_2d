#include <string>
#include <iostream>
#include "icp.h"
#include "cv_utils.h"

using namespace std;
void to_CV32F(const PointMat & e, cv::Mat & c) {
    assert(e.cols() == c.rows);
    const size_t N = e.cols();
    for(size_t i = 0; i < N; ++i) {
        c.at<float>(i,0) = e(0, i);
        c.at<float>(i,1) = e(1, i);
    }
}

cv::Mat range(const int n) {
    cv::Mat r(n, 1,  CV_32FC1);
    for(int i = 0; i < n; ++i) {
        r.at<float>(i, 0) = i;
    }
    return r;
}

cv::Mat extract_matches(const cv::Mat & pts, const cv::Mat & m_ids) {
    const size_t N = m_ids.rows;
    cv::Mat extracted_pts(N, 2, CV_32FC1);
    for(size_t r = 0; r < N; ++r) {
        int i = m_ids.at<int>(r, 0);
        extracted_pts.at<float>(r, 0) = pts.at<float>(i, 0);
        extracted_pts.at<float>(r, 1) = pts.at<float>(i, 1);
    }
    return extracted_pts;
}

void transform(cv::Mat & m, const cv::Mat & t)   {
    cv::Mat a = t(cv::Rect(0, 0, 2, 2));
    cv::Mat b = t(cv::Rect(2, 0, 1, 2));
    //a.convertTo(a, CV_32FC1);
    //b.convertTo(b, CV_32FC1);
    //cout << "t type=" << type2str(t.type());
    cout << "a=\n" << a << "\nb=\n" << b <<"\n";
    for(size_t r = 0; r < m.rows; ++r) {
        m.row(r) = m.row(r) * a.t() + b.t();
    }
}


double OpencvPoseOptimizer::optimize(const PointMat & src_pts, Pose2D init_pose) {
    cv::Mat src_mat(src_pts.cols(), 2, CV_32FC1);
    to_CV32F(src_pts, src_mat);

    for(int i = 0; i < 50; ++i) {
        cv::Mat result = _matcher->search(src_mat);
        plot_figure_size(1200, 780);
        //plot_xlim(9, 15);
        //plot_ylim(0, 5);
        plot_pts(src_mat, "ro-");
        plot_pts(_target_mat, "bo-");
        plot_match(_target_mat, src_mat, result);
        plot_save(string("./iterate_") + to_string(i) + ".png");
        src_mat = _estimator->est(extract_matches(_target_mat, result), src_mat);
        //cout << "transform=" << t << "\n";
        //transform(src_mat, t);
    }
}

Eigen::Vector2f minus_mean(cv::Mat & m) {
    double x_mean = 0;
    double y_mean = 0;
    const int N = m.rows;
    for(int i = 0; i < N; ++i) {
        x_mean += m.at<float>(i, 0);
        y_mean += m.at<float>(i, 1);
    }
    x_mean /= N;
    y_mean /= N;
    std::cout << "x mean=" <<  x_mean << "\n";
    std::cout << "y mean=" <<  y_mean << "\n";
    for(int i = 0; i < N; ++i) {
        m.at<float>(i, 0) -= x_mean;
        m.at<float>(i, 1) -= y_mean;
    }
    Eigen::Vector2f mean;
    mean << x_mean, y_mean;
    return mean;
}

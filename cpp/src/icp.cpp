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


double OpencvPoseOptimizer::optimize(const PointMat & src_pts, Pose2D init_pose) {
    cv::Mat src_mat(src_pts.cols(), 2, CV_32FC1);
    to_CV32F(src_pts, src_mat);
    const int N = src_pts.cols();

    cv::Mat result = _matcher->search(src_mat);
    cout << "result=\n" << result << "\n";
    cout << "result type=" << type2str(result.type()) << endl;
    plot_match(_target_mat, src_mat, result);
}

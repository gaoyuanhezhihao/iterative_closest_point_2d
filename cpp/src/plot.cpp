#include <vector>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/ml.hpp"
#include "matplotlibcpp.h"
#include "plot.h"

using namespace std;

namespace plt = matplotlibcpp;

void plot_match(const cv::Mat & target, const cv::Mat & src, const cv::Mat & match_id) {
    assert(match_id.rows == src.rows);
    const size_t N = src.rows;
    vector<double> x;
    vector<double> y;
    for(int i = 0; i < N; ++i) {
        x.push_back(src.at<float>(i, 0));
        y.push_back(src.at<float>(i, 1));

        const int mi = match_id.at<int>(i, 0);
        x.push_back(target.at<float>(mi, 0));
        y.push_back(target.at<float>(mi, 1));
    }
    
    plt::plot(x, y, "ro-");
}

void plot_pts(PointMat & pts, string style) {
    const int N = pts.cols();
    vector<double> x(N), y(N);
    for(int i = 0; i < N; ++i) {
        x[i] = pts(0, i);
        y[i] = pts(1, i);
    }

    plt::plot(x, y, style);
}

void plot_save(const string & fn) {
    plt::save(fn);
}
void plot_figure_size(const int r, const int c) {
    plt::figure_size(r, c);
}

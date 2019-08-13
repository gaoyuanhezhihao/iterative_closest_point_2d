#ifndef PLOT_H
#define PLOT_H

#include <string>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/ml.hpp"

typedef Eigen::Matrix<double, 2, Eigen::Dynamic> PointMat;
void plot_save(const std::string & fn);
void plot_match(const cv::Mat & target, const cv::Mat & src, const cv::Mat & match_id);
void plot_pts(PointMat & pts, std::string style);
void plot_figure_size(const int r, const int c);
#endif //PLOT_H

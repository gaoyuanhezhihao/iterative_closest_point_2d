#ifndef ICP_H
#define ICP_H

#include <memory>
#include <iostream>
#include <string>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/ml.hpp"
#include <opencv2/flann.hpp>
#include "cv_utils.h"
#include "plot.h"

    typedef Eigen::Translation<double,2> Translation2D;
    typedef Eigen::Translation<double,3> Translation3D;
    typedef Eigen::Transform<double, 2, Eigen::Affine> Transform2D;
    typedef Eigen::Transform<double, 3, Eigen::Affine> Transform3D;
    typedef Eigen::Rotation2D<double> Rot2D;
    typedef Eigen::Matrix<double, 3, 1> Pose2D;
    typedef Eigen::Matrix<double, 6, 1> Pose3D;
    typedef Eigen::Matrix<double, 3, 3> Cov2D;
    typedef Eigen::Matrix<double, 6, 6> Cov3D;
    typedef Eigen::Matrix<double, 2, Eigen::Dynamic> PointMat;



void to_CV32F(const PointMat & e, cv::Mat & c);

cv::Mat range(const int n);

class FlannMatcher{
    public:
        FlannMatcher(const cv::Mat & targets):flann_index(targets, cv::flann::KDTreeIndexParams(4)){

        }

        cv::Mat search(const cv::Mat & src_pts) {
            cv::Mat match_ids;
            cv::Mat dists;
            flann_index.knnSearch(src_pts, match_ids, dists, 1, cv::flann::SearchParams(-1));
            return match_ids;
        }
    private:

          cv::flann::Index flann_index;
};

class MotionEstimator{

};

class OpencvPoseOptimizer{
    public:
        OpencvPoseOptimizer(const PointMat& target_pts):_target_mat(target_pts.cols(), 2, CV_32FC1){
            to_CV32F(target_pts, _target_mat);
            std::cout << "eigen mat:\n" << target_pts << "\n";
            std::cout << "cv mat:\n" << _target_mat << "\n";
            std::cout << "cv mat type=" << type2str(_target_mat.type()) << std::endl;
            _matcher =std::make_shared<FlannMatcher>(_target_mat);
        }

        double optimize(const PointMat & src_pts, Pose2D init_pose);
    private:
        cv::Mat _target_mat;
        std::shared_ptr<FlannMatcher> _matcher;
};
#endif //ICP_H

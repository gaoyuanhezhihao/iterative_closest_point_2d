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
#include "opencv2/video/tracking.hpp"
#include "opencv2/flann.hpp"
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
            flann_index.knnSearch(src_pts, match_ids, dists, 1, cv::flann::SearchParams(32));
            return match_ids;
        }
    private:

          cv::flann::Index flann_index;
};

class MotionEstimator{
    public:
        cv::Mat est(const cv::Mat & target, const cv::Mat & src) {
            return cv::estimateRigidTransform(src, target, true);
        }
};

Eigen::Vector2f minus_mean(cv::Mat & m);

class MyEstimator{
    public:
        cv::Mat est(cv::Mat target, cv::Mat src) {
            assert(target.rows == src.rows);
            std::cout << "before:\n" << target << "\n";
            Eigen::Vector2f t_mean = minus_mean(target);
            std::cout << "after:\n" << target << "\n";
            Eigen::Vector2f s_mean = minus_mean(src);

            cv::Mat w = cv::Mat::zeros(2, 2, CV_32FC1);
            const int N = target.rows;
            for(int i = 0; i < N; ++i) {
                w += target.row(i).t() * src.row(i);
            }
            std::cout << "w=" << w << std::endl;

            Eigen::Matrix2f W = Eigen::Matrix2f::Zero();
            cv::cv2eigen(w, W);

            std::cout << "W=" << W << std::endl;
            Eigen::JacobiSVD<Eigen::Matrix2f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix2f U = svd.matrixU();
            Eigen::Matrix2f V = svd.matrixV();
            Eigen::Matrix2f R = U*(V.transpose());
            Eigen::Vector2f t = t_mean - R * s_mean;
            std::cout << "R=\n" << R << "\nt=\n" << t << "\n";
            cv::Mat RR = (cv::Mat_<float>(2, 2) <<
                    R(0, 0), R(0, 1),
                    R(1, 0), R(1, 1));
            cv::Mat tt = (cv::Mat_<float>(2, 1) <<
                   t(0), 
                    t(1));

            cv::Mat mean = (cv::Mat_<float>(2, 1) << s_mean(0), s_mean(1));


            cv::Mat rst(src.size(), src.type());
            for(int i = 0; i < N; ++i) {
                rst.row(i) = (RR*src.row(i).t() + mean  +tt).t();
            }
            return rst;
        }
};

class OpencvPoseOptimizer{
    public:
        OpencvPoseOptimizer(const PointMat& target_pts):_target_mat(target_pts.cols(), 2, CV_32FC1){
            to_CV32F(target_pts, _target_mat);
            std::cout << "eigen mat:\n" << target_pts << "\n";
            std::cout << "cv mat:\n" << _target_mat << "\n";
            std::cout << "cv mat type=" << type2str(_target_mat.type()) << std::endl;
            _matcher =std::make_shared<FlannMatcher>(_target_mat);
            _estimator = std::make_shared<MyEstimator>();
        }

        double optimize(const PointMat & src_pts, Pose2D init_pose);
    private:
        cv::Mat _target_mat;
        std::shared_ptr<FlannMatcher> _matcher;
        std::shared_ptr<MyEstimator> _estimator;
};
#endif //ICP_H

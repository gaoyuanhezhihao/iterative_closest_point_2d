#include <iostream>
#include <string>
#include "Eigen/Core"
#include "icp.h"
#include "plot.h"
#include <random>

using namespace std;

const int SRC_N = 20;
PointMat target_demo() {
    PointMat src_pts(2, SRC_N);
    for(int i = 0; i < SRC_N; ++i) {
        src_pts(0, i) = i*CV_PI/SRC_N +10;
    }

    for(int i = 0; i < SRC_N; ++i) {
        src_pts(1, i) = sin(i*CV_PI/SRC_N);
    }
    return src_pts;
}

PointMat src_demo(double x, double y, double yaw) {
    PointMat src = target_demo();

    Transform2D t = Translation2D(x, y) * Rot2D(yaw);
    std::random_device rd;
    std::mt19937 gen(rd());  //here you could also set a seed
    std::uniform_real_distribution<double> dis(-0.05, 0.05);

    //generate a 3x3 matrix expression

    //store the random_number in a matrix M
    Eigen::MatrixXd M = Eigen::MatrixXf::Zero(src.rows(),src.cols()).unaryExpr([&](float dummy){return dis(gen);});
    return t*src + M;
}


int main(int argc, const char ** argv){
    PointMat target_pts = target_demo();
    OpencvPoseOptimizer opt(target_pts);

    Pose2D useless_pose;
    PointMat src_pts = src_demo(0.5, 0.1, 0.2);
    cout << "src_pts=" << src_pts << endl;
    opt.optimize(src_pts, useless_pose);

    return 0;
}

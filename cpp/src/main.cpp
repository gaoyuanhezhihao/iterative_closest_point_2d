#include <iostream>
#include <string>
#include "Eigen/Core"
#include "icp.h"
#include "plot.h"

using namespace std;

const int SRC_N = 50;
PointMat target_demo() {
    PointMat src_pts(2, SRC_N);
    for(int i = 0; i < SRC_N; ++i) {
        src_pts(0, i) = i*CV_PI/SRC_N ;
    }

    for(int i = 0; i < SRC_N; ++i) {
        src_pts(1, i) = sin(i*CV_PI/SRC_N);
    }
    return src_pts;
}

PointMat src_demo(double x, double y, double yaw) {
    PointMat src = target_demo();

    Transform2D t = Translation2D(x, y) * Rot2D(yaw);
    return t *src;
}


int main(int argc, const char ** argv){
    plot_figure_size(1200, 780);
    PointMat target_pts = target_demo();
    OpencvPoseOptimizer opt(target_pts);

    Pose2D useless_pose;
    PointMat src_pts = src_demo(1.2, 0.1, 0.2);
    cout << "src_pts=" << src_pts << endl;
    opt.optimize(src_pts, useless_pose);

    plot_pts(src_pts, "r--");
    plot_pts(target_pts, "b--");
    //plt::legend();
    plot_save("./basic.png");
    return 0;
}

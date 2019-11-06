#ifndef ESTIMATORS_SIM3_H_
#define ESTIMATORS_SIM3_H_

#include "Eigen/Core"
#include "Eigen/Dense"

namespace GraphSfM {

struct Sim3
{
    // Eigen::Matrix4d similarity;
    // std::string sfm_data_path;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    double s = 1.0;

    Sim3()
    {
        R = Eigen::Matrix3d::Identity();
        t = Eigen::Vector3d::Zero();
        s = 1.0;
    }

    Sim3(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation, const double& scale)
    {
        // sfm_data_path = path;
        R = rotation;
        t = translation;
        s = scale; 
    }

    Sim3(const Sim3& sim)
    {
        R = sim.R;
        t = sim.t;
        s = sim.s;
    }
};

}   // namespace GraphSfM

#endif
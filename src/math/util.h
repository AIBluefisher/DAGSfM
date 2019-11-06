#ifndef GRAPHSFM_MATH_UTIL_H_
#define GRAPHSFM_MATH_UTIL_H_

#include <algorithm>
#include <cmath>

#include <ceres/rotation.h>
#include <Eigen/Geometry>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

namespace GraphSfM {

static const double kRadToDeg = 180.0 / M_PI;
static const double kDegToRad = M_PI / 180.0;

inline double RadToDeg(double angle_radians) 
{
    return angle_radians * kRadToDeg;
}

inline double DegToRad(double angle_degrees) 
{
    return angle_degrees * kDegToRad;
}

inline double Clamp(const double val, const double min, const double max) 
{
    return std::max(min, std::min(val, max));
}

inline Eigen::Vector3d RelativeRotationFromTwoRotations(const Eigen::Vector3d& rotation1,
                                                 const Eigen::Vector3d& rotation2) 
{
    Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
    ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
    ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

    const Eigen::AngleAxisd relative_rotation(
        rotation_matrix2 * rotation_matrix1.transpose());
    return relative_rotation.angle() * relative_rotation.axis();
}

inline Eigen::Vector3d RelativeTranslationFromTwoPositions(const Eigen::Vector3d& position1,
                                                    const Eigen::Vector3d& position2,
                                                    const Eigen::Vector3d& rotation1) 
{
    Eigen::Matrix3d rotation_matrix1;
    ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
    const Eigen::Vector3d relative_translation =
          rotation_matrix1 * (position2 - position1).normalized();
    return relative_translation;
}

}  // namespace GraphSfM

#endif  // GRAPHSFM_MATH_UTIL_H_

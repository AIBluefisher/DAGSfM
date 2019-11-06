#ifndef GRAPHSFM_MATH_ROTATION_H_
#define GRAPHSFM_MATH_ROTATION_H_

#include <Eigen/Core>

namespace GraphSfM {

// Multiply two angle-axis rotations without having to form the rotation
// matrices. Given the corresponding rotation matrices R1 and R2, this method
// computes: R3 = R1 * R2.
Eigen::Vector3d MultiplyRotations(const Eigen::Vector3d& rotation1,
                                  const Eigen::Vector3d& rotation2);
}  // namespace GraphSfM

#endif  // GRAPHSFM_MATH_ROTATION_H_

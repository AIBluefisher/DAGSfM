#include <Eigen/Core>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>

#include "gtest/gtest.h"
#include "math/matrix/rq_decomposition.h"

namespace GraphSfM {

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Matrix4d;
using Eigen::Matrix4f;
using Eigen::MatrixXd;

const double kSmallTolerance = 1e-12;
const double kLargeTolerance = 1e-6;

template <typename MatrixType>
void TestRQDecomposition(const int rows, const int cols, const double tol) {
  for (int i = 0; i < 100; ++i) {
    const MatrixType& m = MatrixType::Random(rows, cols);
    RQDecomposition<MatrixType> rq(m);
    EXPECT_TRUE(rq.matrixR().bottomRows(std::min(rows, cols))
                    .isUpperTriangular());
    EXPECT_TRUE(rq.matrixQ().isUnitary());
    EXPECT_NEAR(rq.matrixQ().determinant(), 1.0, kLargeTolerance);

    const MatrixType& diff = rq.matrixR() * rq.matrixQ() - m;
    EXPECT_LE(diff.cwiseAbs().maxCoeff(), tol)
        << rq.matrixR() << "\n\n" << rq.matrixQ() << "\n\n"
        << rq.matrixR() * rq.matrixQ() << "\n\n" << m;
  }
}

TEST(RQTest, FixedSizeMatrices) {
  // Square matrices.
  TestRQDecomposition<Matrix3d>(3, 3, kSmallTolerance);
  TestRQDecomposition<Matrix4d>(4, 4, kSmallTolerance);
  TestRQDecomposition<Matrix<double, 5, 5> >(5, 5, kSmallTolerance);
  TestRQDecomposition<Matrix<double, 6, 6> >(6, 6, kSmallTolerance);

  // Non-square matrices.
  TestRQDecomposition<Matrix<double, 3, 4> >(3, 4, kSmallTolerance);
  TestRQDecomposition<Matrix<double, 4, 3> >(4, 3, kSmallTolerance);
}

TEST(RQTest, DynamicSizedMatrices) {
  // Square matrices.
  TestRQDecomposition<MatrixXd>(25, 25, kSmallTolerance);
  TestRQDecomposition<MatrixXd>(50, 50, kSmallTolerance);
  TestRQDecomposition<MatrixXd>(75, 75, kSmallTolerance);
  TestRQDecomposition<MatrixXd>(100, 100, kSmallTolerance);

  // Non-square matrices.
  TestRQDecomposition<MatrixXd>(30, 40, kSmallTolerance);
  TestRQDecomposition<MatrixXd>(40, 30, kSmallTolerance);
}

TEST(RQTest, FloatMatrices) {
  TestRQDecomposition<Matrix3f>(3, 3, kLargeTolerance);
  TestRQDecomposition<Matrix4f>(4, 4, kLargeTolerance);
}

TEST(RQTest, RowMajorMatrices) {
  TestRQDecomposition<Matrix<double, 3, 3, Eigen::RowMajor> >(3, 3,
                                                              kSmallTolerance);
  TestRQDecomposition<Matrix<float, 3, 3, Eigen::RowMajor> >(3, 3,
                                                             kLargeTolerance);
}

}  // namespace GraphSfM

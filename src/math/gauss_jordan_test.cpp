#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "math/matrix/gauss_jordan.h"

namespace GraphSfM {
using RowMajorMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

TEST(GaussJordan, FullDiagonalizationOnSquaredRowMajorMatrix) {
  const int kNumRows = 32;
  RowMajorMatrixXd mat = RowMajorMatrixXd::Random(kNumRows, kNumRows);
  GaussJordan(&mat);
  // Trace of matrix must be equals to the number of rows.
  EXPECT_NEAR(mat.trace(), static_cast<double>(kNumRows), 1e-6);
  // Verify that the lower triangular part sums to the trace.
  EXPECT_NEAR(mat.sum(), mat.trace(), 1e-6);
}

TEST(GaussJordan, FullDiagonalizationOnSquaredColumnMajorMatrix) {
  const int kNumRows = 32;
  Eigen::MatrixXd mat = Eigen::MatrixXd::Random(kNumRows, kNumRows);
  GaussJordan(&mat);
  // Trace of matrix must be equals to the number of rows.
  EXPECT_NEAR(mat.trace(), static_cast<double>(kNumRows), 1e-6);
  // Verify that the lower triangular part sums to the trace.
  EXPECT_NEAR(mat.sum(), mat.trace(), 1e-6);
}

TEST(GaussJordan, EliminationOnFatMatrix) {
  const int kNumRows = 32;
  const int kNumCols = kNumRows + 4;
  RowMajorMatrixXd mat = RowMajorMatrixXd::Random(kNumRows, kNumCols);
  GaussJordan(&mat);
  // Verify that the left-block (rows, rows) is diagonalized.
  EXPECT_NEAR(mat.block(0, 0, kNumRows, kNumRows).sum(),
              mat.block(0, 0, kNumRows, kNumRows).trace(), 1e-6);
}

TEST(GaussJordan, PartialEliminationOnFatMatrix) {
  const int kNumRows = 32;
  const int kNumCols = kNumRows + 4;
  const int kNumRowsToProcess = kNumRows - 4;
  const int kLastRowToProcess = kNumRowsToProcess - 1;
  RowMajorMatrixXd mat = RowMajorMatrixXd::Random(kNumRows, kNumCols);
  GaussJordan(kLastRowToProcess, &mat);
  // Verify that the left-block (rows, rows) is diagonalized.
  EXPECT_NEAR(mat.block(0, 0, kNumRowsToProcess, kNumRowsToProcess).trace(),
              static_cast<double>(kNumRowsToProcess),
              1e-6);
}

TEST(GaussJordan, PartialDiagonalizationOnFatMatrix) {
  const int kNumRows = 32;
  const int kNumCols = kNumRows + 4;
  const int kLastRowToProcess = 2;
  RowMajorMatrixXd mat = RowMajorMatrixXd::Random(kNumRows, kNumCols);
  GaussJordan(kNumRows - 1, kLastRowToProcess, &mat);
  // Verify that the left-block (rows, rows) is partially diagonalized.
  EXPECT_NEAR(mat.block(kLastRowToProcess, kLastRowToProcess,
                        kNumRows - kLastRowToProcess,
                        kNumRows - kLastRowToProcess).sum(),
              kNumRows - kLastRowToProcess, 1e-6);
  EXPECT_NEAR(mat.block(kLastRowToProcess, kLastRowToProcess,
                        kNumRows - kLastRowToProcess,
                        kNumRows - kLastRowToProcess).sum(),
              mat.block(kLastRowToProcess, kLastRowToProcess,
                        kNumRows - kLastRowToProcess,
                        kNumRows - kLastRowToProcess).trace(),
              1e-6);
  EXPECT_NE(mat.block(0, 0, kNumRows, kNumRows).sum(),
            mat.block(kLastRowToProcess, kLastRowToProcess,
                      kNumRows - kLastRowToProcess,
                      kNumRows - kLastRowToProcess).sum());
}

TEST(GaussJordan, FullDiagonalizationOnLargeSquaredMatrix) {
  const int kNumRows = 400;
  const int kNumCols = kNumRows;
  RowMajorMatrixXd mat = RowMajorMatrixXd::Random(kNumRows, kNumCols);
  GaussJordan(&mat);
  EXPECT_NEAR(mat.sum(), mat.trace(), 1e-6);
}

}  // namespace GraphSfM

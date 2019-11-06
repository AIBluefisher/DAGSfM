#ifndef GRAPHSFM_MATH_MATRIX_GAUSS_JORDAN_H_
#define GRAPHSFM_MATH_MATRIX_GAUSS_JORDAN_H_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <glog/logging.h>

namespace GraphSfM {
namespace internal {

// Finds the largest absolute value in column starting from a certain row.
template <typename Derived>
typename Derived::Scalar FindLargestAbsoluteValueInColumn(
    const Eigen::MatrixBase<Derived>& matrix,
    const int column,
    const int starting_row,
    const int ending_row,
    int* max_value_index) {
  using ScalarType = typename Derived::Scalar;
  ScalarType max_value = matrix(starting_row, column);
  *max_value_index = starting_row;
  for (int row = starting_row; row < ending_row; ++row) {
    const ScalarType candidate_max_value = matrix(row, column);
    if (std::abs(max_value) < std::abs(candidate_max_value)) {
      max_value = candidate_max_value;
      *max_value_index = row;
    }
  }
  return max_value;
}

// The implementation is based on Gauss Jordan Elimination algorithm explained
// in
// https://martin-thoma.com/solving-linear-equations-with-gaussian-elimination/
//
// for (row = 0; row < max_rows; ++row) {
//   Search for the maximum entry in column = row. Keep its value and row.
//   Swap row with maximum entry to be the current one.
//   Make all the entries in column zero.
// }
//
// This function eliminates the lower-triangular part of a matrix while leaving
// the main diagonal of the matrix set to ones. This function operates in a
// top-down fashion, i.e., starting from the first row and ending at the
// max_rows-th row of the matrix. The function returns the number of rows that
// were processed.
//
// Example:
//         | a b ... c |                            |1 x ... y |
// input = | d e ... f |  => TopDownGaussJordan =>  |0 1 ... z |
//         |     ...   |                            |0 0 ... w |
//         | g h ... i |                            |0 0 ... 1 |
//
// NOTE: Since the implementation handles rows, the implementation performs
// faster when using row-major matrices.
template <typename Derived>
int TopDownGaussJordan(const int last_row, Eigen::MatrixBase<Derived>* input) {
  using ScalarType = typename Derived::Scalar;
  const ScalarType kPrecisionThreshold = static_cast<ScalarType>(1e-9);
  CHECK_GE(last_row, 1) << "last_row must be larger than or equal to 1.";
  CHECK_GE(CHECK_NOTNULL(input)->cols(), input->rows())
      << "Expected a sqaured or fat matrix.";
  const int num_rows_to_process = std::min(
      static_cast<int>(input->rows()), last_row + 1);
  const int last_row_to_process = num_rows_to_process - 1;

  // This for loop eliminates entries in the lower-left triangular part, and it
  // operates from top to bottom of the matrix.
  for (int current_row = 0; current_row < num_rows_to_process; ++current_row) {
    // Search for the maxium entry in column with current_row index.
    const int tail_size = num_rows_to_process - current_row;
    int max_coeff_row_idx = 0;
    const ScalarType max_value =
        FindLargestAbsoluteValueInColumn(
            *input, current_row, current_row,
            last_row_to_process, &max_coeff_row_idx);

    // Swap rows.
    input->row(current_row).swap(input->row(max_coeff_row_idx));

    // Divide the current row by the leading coefficient or max value.
    input->row(current_row) /= max_value;
    (*input)(current_row, current_row) = static_cast<ScalarType>(1.0);

    // Set the remaining entries of the column to zero.
    for (int temp_row = current_row + 1;
         temp_row < num_rows_to_process; ++temp_row) {
      // If the number is too small, it is best to consider it as zero already.
      const ScalarType leading_coeff = (*input)(temp_row, current_row);
      if (std::abs(leading_coeff) < kPrecisionThreshold) {
        continue;
      }

      // Multiply current_row by the leading coeff and subtract it from temp_row
      // to eliminate or set the leading coefficient to zero.
      input->row(temp_row) -= leading_coeff * input->row(current_row);
    }
  }

  return num_rows_to_process;
}

// This function eliminates the upper-triangular part of the upper-left
// block of the input matrix. This function requires as input a matrix processed
// by the TopDownGaussJordan() function a priori.
//
// Example:
//
//         |1 x ... y |                           | 1 0 ... 0 |
// input = |0 1 ... z | => BottomUpGaussJordan => | 0 1 ... 0 |
//         |0 0 ... w |                           | 0 0 ... 0 |
//         |0 0 ... 1 |                           | 0 0 ... 1 |
template <typename Derived>
void BottomUpGaussJordan(const int start_row,
                         const int last_row_to_process,
                         Eigen::MatrixBase<Derived>* input) {
  using ScalarType = typename Derived::Scalar;
  const ScalarType kPrecisionThreshold = static_cast<ScalarType>(1e-9);
  const int min_dimension = std::min(
      static_cast<int>(CHECK_NOTNULL(input)->rows()),
      static_cast<int>(input->cols()));
  CHECK_GE(start_row, 0);
  CHECK_LE(start_row, min_dimension);
  CHECK_GE(last_row_to_process, 0);
  CHECK_LE(last_row_to_process, start_row);

  // Cannot process bottom up.
  if (start_row == last_row_to_process) {
    return;
  }

  // This for loop eliminates entries in the upper-left triangular part, and it
  // operates from bottom to top of the matrix.
  for (int current_row = start_row - 1;
       current_row >= last_row_to_process; --current_row) {
    // Column to process.
    const int col = current_row;
    // Eliminate coefficients in column.
    for (int row = current_row - 1; row >= last_row_to_process; --row) {
      // Get the leading coefficient to eliminate.
      const ScalarType leading_coeff = (*input)(row, col);
      if (std::abs(leading_coeff) < kPrecisionThreshold) {
        continue;
      }

      // Eliminate.
      input->row(row) -= leading_coeff * input->row(current_row);
    }
  }
}

}  // namespace internal

// This function performs Gauss-Jordan elimination on a squared or fat-matrix.
// The implementation operates only on the upper-left-squared block of size
// num. rows x num. rows, and can be used to perform the following operations:
//
// 1. Top-down elimination of the lower-triangular part up to a certain row; and
// 2. Bottom-up elimination of the upper-triangular part up to a certain row.
//
// Example:
//
// Case 1: Top-down elimination up to second row:
//
//         | a b c d |                            | 1 x y z |
// input = | e f g h |  => TopDownGaussJordan =>  | 0 1 u v |
//         | i j k l |                            | 0 0 w q |
template <typename Derived>
void GaussJordan(const int top_down_last_row,
                 const int bottom_up_last_row,
                 Eigen::MatrixBase<Derived>* input) {
  using internal::TopDownGaussJordan;
  using internal::BottomUpGaussJordan;
  // Eliminate lower-triangular part of the upper-left squared block of the
  // input matrix.
  const int num_rows_processed = TopDownGaussJordan(top_down_last_row, input);

  // Eliminate upper-triangular part of the upper-left squared block of the
  // input matrix.
  BottomUpGaussJordan(num_rows_processed, bottom_up_last_row, input);
}

// TODO(vfragoso): Document me!
template <typename Derived>
void GaussJordan(const int top_down_last_row,
                 Eigen::MatrixBase<Derived>* input) {
  using internal::TopDownGaussJordan;
  TopDownGaussJordan(top_down_last_row, input);
}

template <typename Derived>
void GaussJordan(Eigen::MatrixBase<Derived>* input) {
  GaussJordan(CHECK_NOTNULL(input)->rows() - 1, 0, input);
}

}  // namespace GraphSfM

#endif  // GRAPHSFM_MATH_MATRIX_GAUSS_JORDAN_H_

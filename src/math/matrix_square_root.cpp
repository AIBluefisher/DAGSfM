#include "math/matrix_square_root.h"

#include "eigen3/Eigen/Cholesky"

namespace GraphSfM {
Eigen::MatrixXd MatrixSquareRoot(const Eigen::MatrixXd& mat)
{
    // only square matrix can apply matrix square root
    CHECK_EQ(mat.rows(), mat.cols());

    Eigen::EigenSolver<Eigen::MatrixXd> solver(mat);
    Eigen::MatrixXd V = solver.eigenvectors().real();
    Eigen::VectorXd Dv = solver.eigenvalues().real();
    Eigen::MatrixXd sqrt_D = Dv.cwiseSqrt().asDiagonal();

    return V * sqrt_D * V.inverse();
}

Eigen::MatrixXd MatrixSquareRootForSemidefinitePositiveMat(const Eigen::MatrixXd& mat)
{
    // only square matrix can apply matrix square root
    CHECK_EQ(mat.rows(), mat.cols());

    Eigen::MatrixXd result;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(mat);

    result = ldlt.matrixL();
    result = ldlt.transpositionsP().transpose() * result;
    result *= ldlt.vectorD().array().sqrt().matrix().asDiagonal();
    
    return result;
}

} // namespace GraphSfM
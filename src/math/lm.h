#ifndef MATH_LM_H_
#define MATH_LM_H_

// Levenberg Marquardt Non Linear Optimization
#include <Eigen/Core>

namespace DAGSfM {
using namespace Eigen;

/**
 * @brief Generic functor Levenberg-Marquardt minimization
 * @tparam _Scalar Type of internal computation
 * @tparam NX Number of values per sample at compile time
 * @tparam NY Number of samples at compile time
 */
template <typename _Scalar, int NX = Dynamic, int NY = Dynamic>
struct Functor {
  using Scalar = _Scalar;
  enum { InputsAtCompileTime = NX, ValuesAtCompileTime = NY };
  using InputType = Matrix<Scalar, InputsAtCompileTime, 1>;
  using ValueType = Matrix<Scalar, ValuesAtCompileTime, 1>;
  using JacobianType = Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime>;

  /// Number of values per sample
  const int m_inputs;

  // Number of sample
  const int m_values;

  /**
   * @brief Default constructor
   */
  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}

  /**
   * @brief Constructor
   * @param inputs Number of column per sample
   * @param values Number of sample
   */
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  /**
   * @brief Get number of samples
   * @return Number of samples
   */
  int inputs() const { return m_inputs; }

  /**
   * @brief Get number of samples
   * @return Number of samples
   */
  int values() const { return m_values; }

  // you should define that in the subclass :
  //  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0)
  //  const;
};

};  // namespace DAGSfM

#endif  // GRAPHSFM_NUMERIC_LM_HPP

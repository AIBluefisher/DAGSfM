// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#ifndef COLMAP_SRC_UTIL_ALIGNMENT_H_
#define COLMAP_SRC_UTIL_ALIGNMENT_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SparseCore>
#include <Eigen/StdVector>
#include <initializer_list>
#include <memory>
#include <vector>

#ifndef EIGEN_ALIGNED_ALLOCATOR
#define EIGEN_ALIGNED_ALLOCATOR Eigen::aligned_allocator
#endif

// Equivalent to EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION but with support for
// initializer lists, which is a C++11 feature and not supported by the Eigen.
// The initializer list extension is inspired by Theia and StackOverflow code.
#define EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(...)                 \
  namespace std {                                                          \
  template <>                                                              \
  class vector<__VA_ARGS__, std::allocator<__VA_ARGS__>>                   \
      : public vector<__VA_ARGS__, EIGEN_ALIGNED_ALLOCATOR<__VA_ARGS__>> { \
    typedef vector<__VA_ARGS__, EIGEN_ALIGNED_ALLOCATOR<__VA_ARGS__>>      \
        vector_base;                                                       \
                                                                           \
   public:                                                                 \
    typedef __VA_ARGS__ value_type;                                        \
    typedef vector_base::allocator_type allocator_type;                    \
    typedef vector_base::size_type size_type;                              \
    typedef vector_base::iterator iterator;                                \
    explicit vector(const allocator_type& a = allocator_type())            \
        : vector_base(a) {}                                                \
    template <typename InputIterator>                                      \
    vector(InputIterator first, InputIterator last,                        \
           const allocator_type& a = allocator_type())                     \
        : vector_base(first, last, a) {}                                   \
    vector(const vector& c) : vector_base(c) {}                            \
    explicit vector(size_type num, const value_type& val = value_type())   \
        : vector_base(num, val) {}                                         \
    vector(iterator start, iterator end) : vector_base(start, end) {}      \
    vector& operator=(const vector& x) {                                   \
      vector_base::operator=(x);                                           \
      return *this;                                                        \
    }                                                                      \
    vector(initializer_list<__VA_ARGS__> list)                             \
        : vector_base(list.begin(), list.end()) {}                         \
  };                                                                       \
  }  // namespace std

namespace DAGSfM {
using Eigen::Map;

/// Trait used for double type
using EigenDoubleTraits = Eigen::NumTraits<double>;

/// 3d vector using double internal format
using Vec3 = Eigen::Vector3d;

/// 2d vector using int internal format
using Vec2i = Eigen::Vector2i;

/// 2d vector using float internal format
using Vec2f = Eigen::Vector2f;

/// 3d vector using float internal format
using Vec3f = Eigen::Vector3f;

/// 9d vector using double internal format
using Vec9 = Eigen::Matrix<double, 9, 1>;

/// Quaternion type
using Quaternion = Eigen::Quaternion<double>;

/// 3x3 matrix using double internal format
using Mat3 = Eigen::Matrix<double, 3, 3>;

/// 3x4 matrix using double internal format
using Mat34 = Eigen::Matrix<double, 3, 4>;

/// 2d vector using double internal format
using Vec2 = Eigen::Vector2d;

/// 4d vector using double internal format
using Vec4 = Eigen::Vector4d;

/// 6d vector using double internal format
using Vec6 = Eigen::Matrix<double, 6, 1>;

/// 4x4 matrix using double internal format
using Mat4 = Eigen::Matrix<double, 4, 4>;

/// generic matrix using unsigned int internal format
using Matu = Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>;

/// 3x3 matrix using double internal format with RowMajor storage
using RMat3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

//-- General purpose Matrix and Vector
/// Unconstrained matrix using double internal format
using Mat = Eigen::MatrixXd;

/// Unconstrained vector using double internal format
using Vec = Eigen::VectorXd;

/// Unconstrained vector using unsigned int internal format
using Vecu = Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>;

/// Unconstrained matrix using float internal format
using Matf = Eigen::MatrixXf;

/// Unconstrained vector using float internal format
using Vecf = Eigen::VectorXf;

/// 2xN matrix using double internal format
using Mat2X = Eigen::Matrix<double, 2, Eigen::Dynamic>;

/// 3xN matrix using double internal format
using Mat3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;

/// 4xN matrix using double internal format
using Mat4X = Eigen::Matrix<double, 4, Eigen::Dynamic>;

/// Nx9 matrix using double internal format
using MatX9 = Eigen::Matrix<double, Eigen::Dynamic, 9>;

//-- Sparse Matrix (Column major, and row major)
/// Sparse unconstrained matrix using double internal format
using sMat = Eigen::SparseMatrix<double>;

/// Sparse unconstrained matrix using double internal format and Row Major
/// storage
using sRMat = Eigen::SparseMatrix<double, Eigen::RowMajor>;

// // EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Vec2)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Vec3)
// // // EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Vec4)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Vec6)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Vec9)
// // EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Vec2i)
// // EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Vec2f)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Vec3f)
// // // EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Quaternion)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Mat3)
// // EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::RMat3)
// // // EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Mat4)
// // EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(DAGSfM::Mat34)
}  // namespace DAGSfM

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Vector2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Vector4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Vector4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Affine3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Affine3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Quaterniond)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Quaternionf)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix<float, 3, 4>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix<double, 3, 4>)

#define EIGEN_STL_UMAP(KEY, VALUE)                                   \
  std::unordered_map<KEY, VALUE, std::hash<KEY>, std::equal_to<KEY>, \
                     Eigen::aligned_allocator<std::pair<KEY const, VALUE>>>

#endif  // COLMAP_SRC_UTIL_ALIGNMENT_H_

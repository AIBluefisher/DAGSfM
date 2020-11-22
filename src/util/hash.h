#ifndef DAGSfM_UTIL_HASH_H_
#define DAGSfM_UTIL_HASH_H_

#include <Eigen/Core>
#include <utility>

// This file defines hash functions for stl containers.
namespace std {
namespace {

// Combines the hash of v with the current hash value seed. This is the
// recommended approach from Boost.
template <class T>
inline void HashCombine(const T& v, std::size_t* seed) {
  std::hash<T> hasher;
  *seed ^= hasher(v) + 0x9e3779b9 + (*seed << 6) + (*seed >> 2);
}

}  // namespace

// STL does not implement hashing for pairs, so a simple pair hash is done here.
template <typename T1, typename T2>
struct hash<std::pair<T1, T2> > {
 public:
  size_t operator()(const std::pair<T1, T2>& e) const {
    size_t seed = 0;
    HashCombine(e.first, &seed);
    HashCombine(e.second, &seed);
    return seed;
  }
};

// A generic hash function that hashes constant size Eigen matrices and vectors.
template <typename T, int N, int M>
struct hash<Eigen::Matrix<T, N, M> > {
  size_t operator()(const Eigen::Matrix<T, N, M>& matrix) const {
    size_t seed = 0;
    const T* data = matrix.data();
    for (int i = 0; i < matrix.size(); ++i) {
      seed ^= std::hash<T>()(data[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

}  // namespace std

#endif  // DAGSfM_UTIL_HASH_H_

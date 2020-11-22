#ifndef BASE_STRUCTURE_H_
#define BASE_STRUCTURE_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include "estimators/rotation_averaging.h"
#include "feature/types.h"

using namespace Eigen;
using namespace std;

class ImgFeature {
 public:
  ImgFeature() {}

  ImgFeature(double _x, double _y, double _scale, double _orientation)
      : x(_x), y(_y), scale(_scale), orien(_orientation) {}

  double x, y, scale, orien;
};

/* Data struct for matches */
class KeypointMatch {
 public:
  KeypointMatch() {}

  KeypointMatch(int idx1, int idx2) : m_idx1(idx1), m_idx2(idx2) {}

  int m_idx1, m_idx2;
};

class AdjListElem {
 public:
  bool operator<(const AdjListElem& other) const {
    return m_index < other.m_index;
  }

  unsigned int m_index;
  std::vector<KeypointMatch> m_match_list;
};

typedef std::vector<AdjListElem> MatchAdjList;
typedef std::vector<ImgFeature> ImgFeatures;
typedef std::pair<int, int> ImageKey;
typedef std::vector<ImageKey> ImageKeyVector;

class TrackData {
 public:
  TrackData() {}

  TrackData(ImageKeyVector keys) : m_views(keys) {}

  ImageKeyVector m_views;
};

struct IndMatch {
  IndMatch(IndexT i = 0, IndexT j = 0) {
    i_ = i;
    j_ = j;
  }

  friend bool operator==(const IndMatch& m1, const IndMatch& m2) {
    return (m1.i_ == m2.i_ && m1.j_ == m2.j_);
  }

  friend bool operator!=(const IndMatch& m1, const IndMatch& m2) {
    return !(m1 == m2);
  }

  // Lexicographical ordering of matches. Used to remove duplicates.
  friend bool operator<(const IndMatch& m1, const IndMatch& m2) {
    return (m1.i_ < m2.i_ || (m1.i_ == m2.i_ && m1.j_ < m2.j_));
  }

  /// Remove duplicates ((i_, j_) that appears multiple times)
  static bool getDeduplicated(std::vector<IndMatch>& vec_match) {
    const size_t sizeBefore = vec_match.size();
    std::set<IndMatch> set_deduplicated(vec_match.begin(), vec_match.end());
    vec_match.assign(set_deduplicated.begin(), set_deduplicated.end());
    return sizeBefore != vec_match.size();
  }

  // Serialization
  template <class Archive>
  void serialize(Archive& ar) {
    ar(i_, j_);
  }

  IndexT i_, j_;  // Left, right index
};

typedef std::vector<IndMatch> IndMatches;
typedef std::pair<IndexT, IndexT> Pair;

/// Set of Pair
typedef std::set<Pair> Pair_Set;
//--
/// Pairwise matches (indexed matches for a pair <I,J>)
/// The structure used to store corresponding point indexes per images pairs
typedef std::map<Pair, IndMatches> PairWiseMatches;
#endif  // STRUCTURE_H_INCLUDED

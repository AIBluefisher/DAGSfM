// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef I23DSFM_TYPES_H_
#define I23DSFM_TYPES_H_

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <vector>

#if defined I23DSFM_STD_UNORDERED_MAP
#include <unordered_map>
#endif

namespace i23dSFM{

typedef uint32_t IndexT;
static const IndexT UndefinedIndexT = std::numeric_limits<IndexT>::max();

typedef std::pair<IndexT,IndexT> Pair;
typedef std::set<Pair> Pair_Set;
typedef std::vector<Pair> Pair_Vec;

#define I23DSFM_NO_UNORDERED_MAP 1

#if defined I23DSFM_NO_UNORDERED_MAP
template<typename K, typename V>
struct Hash_Map : std::map<K, V, std::less<K>,
 Eigen::aligned_allocator<std::pair<K,V> > > {};
#endif

#if defined I23DSFM_STD_UNORDERED_MAP
template<typename Key, typename Value>
struct Hash_Map : std::unordered_map<Key, Value> {};
#endif

} // namespace i23dSFM

#endif  // I23DSFM_TYPES_H_

// BSD 3-Clause License

// Copyright (c) 2020, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GRAPH_UNION_FIND_H
#define GRAPH_UNION_FIND_H

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace DAGSfM {
namespace graph {

class UnionFind {
 private:
  std::vector<size_t> ranks_;
  std::vector<size_t> parents_;
  std::vector<size_t> nodes_;
  std::unordered_map<size_t, size_t> nodes_mapper_;

 public:
  // constructor
  UnionFind() {}
  UnionFind(size_t n);

  // union find operations.
  void Init(size_t n);
  void InitWithNodes(const std::vector<size_t>& nodes);
  size_t FindRoot(size_t x);
  void Union(size_t x, size_t y);

  // Get functions.
  std::vector<size_t> GetRanks() const;
  std::vector<size_t> GetParents() const;
  // Components are the unique ids of parents.
  std::unordered_set<size_t> GetConnectedComponents() const;
};

}  // namespace graph
}  // namespace DAGSfM

#endif
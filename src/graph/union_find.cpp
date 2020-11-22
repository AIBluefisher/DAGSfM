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

#include "graph/union_find.h"

namespace DAGSfM {
namespace graph {

UnionFind::UnionFind(size_t n) { this->Init(n); }

void UnionFind::Init(size_t n) {
  // Reserve enough space.
  parents_.reserve(n);
  ranks_.reserve(n);
  nodes_.reserve(n);
  nodes_mapper_.reserve(n);

  for (size_t i = 0; i < n; i++) {
    parents_.push_back(i);
    ranks_.push_back(0);
    nodes_.push_back(i);
    nodes_mapper_[i] = i;
  }
}

void UnionFind::InitWithNodes(const std::vector<size_t>& nodes) {
  // Clear the space of nodes, if Init(size_t n) is called first.
  std::vector<size_t>{}.swap(nodes_);
  nodes_.reserve(nodes.size());
  nodes_.assign(nodes.begin(), nodes.end());

  // Clear the space of nodes mapper, if Init(size_t n) is called first.
  nodes_mapper_.clear();
  nodes_mapper_.reserve(nodes.size());

  for (uint i = 0; i < nodes.size(); i++) {
    nodes_mapper_[nodes[i]] = i;
  }
}

size_t UnionFind::FindRoot(size_t x) {
  size_t idx = nodes_mapper_[x];
  return (parents_[idx] == idx)
             ? idx
             : (parents_[idx] = FindRoot(nodes_[parents_[idx]]));
}

void UnionFind::Union(size_t x, size_t y) {
  x = FindRoot(x);
  y = FindRoot(y);
  if (x == y) return;

  if (ranks_[x] < ranks_[y])
    parents_[x] = y;
  else {
    parents_[y] = x;
    if (ranks_[x] == ranks_[y]) ranks_[x]++;
  }
}

std::vector<size_t> UnionFind::GetRanks() const { return ranks_; }

std::vector<size_t> UnionFind::GetParents() const { return parents_; }

std::unordered_set<size_t> UnionFind::GetConnectedComponents() const {
  std::unordered_set<size_t> components(parents_.begin(), parents_.end());
  return components;
}

}  // namespace graph
}  // namespace DAGSfM
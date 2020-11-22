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

#ifndef SRC_CLUSTERING_CLUSTER_H_
#define SRC_CLUSTERING_CLUSTER_H_

#include <glog/logging.h>
#include <igraph/igraph.h>

#include <cstdio>
#include <unordered_map>

#include "util/types.h"

using namespace colmap;

namespace DAGSfM {

enum ClusterType { NCUT, KMEANS, SPECTRAL, COMMUNITY_DETECTION, HYBRID };

class Cluster {
 protected:
  std::unordered_map<int, int> labels_;

  igraph_t igraph_;

  igraph_vector_t i_edges_;

  igraph_vector_t i_weights_;

  std::vector<int> nodes_;

  int cluster_num_;

 public:
  Cluster();
  ~Cluster();

  virtual std::unordered_map<int, int> ComputeCluster(
      const std::vector<std::pair<int, int>>& edges,
      const std::vector<int>& weights, const int num_partitions) = 0;

  int ClusterNum() const;

  bool InitIGraph(const std::vector<std::pair<int, int>>& edges,
                  const std::vector<int>& weights);

  bool OutputIGraph(const std::string graph_dir,
                    const std::string image_path = "") const;
};

}  // namespace DAGSfM

#endif
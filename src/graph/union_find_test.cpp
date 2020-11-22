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

#include "union_find.h"

#include <utility>
#include <vector>

#include "gtest/gtest.h"

using namespace DAGSfM::graph;

TEST(UNION_FIND_TEST, TEST_INIT) {
  UnionFind union_find;
  const int size = 100;
  union_find.Init(size);

  for (int i = 0; i < size; i++) {
    ASSERT_EQ(union_find.GetRanks()[i], 0);
    ASSERT_EQ(union_find.GetParents()[i], i);
  }
}

TEST(UNION_FIND_TEST, TEST_FINDROOT) {
  const int size = 10;
  UnionFind union_find(size);

  std::vector<std::pair<int, int>> sets = {{0, 2}, {4, 5}, {3, 9},
                                           {5, 7}, {6, 7}, {1, 4}};

  for (auto set : sets) {
    union_find.Union(set.first, set.second);
  }

  EXPECT_EQ(union_find.FindRoot(0), 0);
  EXPECT_EQ(union_find.FindRoot(1), 4);
  EXPECT_EQ(union_find.FindRoot(2), 0);
  EXPECT_EQ(union_find.FindRoot(3), 3);
  EXPECT_EQ(union_find.FindRoot(4), 4);
  EXPECT_EQ(union_find.FindRoot(5), 4);
  EXPECT_EQ(union_find.FindRoot(6), 4);
  EXPECT_EQ(union_find.FindRoot(7), 4);
  EXPECT_EQ(union_find.FindRoot(8), 8);
  EXPECT_EQ(union_find.FindRoot(9), 3);
}

TEST(UNION_FIND_TEST, TEST_UNION) {
  const int size = 10;
  UnionFind union_find(size);

  std::vector<std::pair<int, int>> sets = {{0, 2}, {4, 5}, {3, 9},
                                           {5, 7}, {6, 7}, {1, 4}};
  for (auto set : sets) {
    union_find.Union(set.first, set.second);
  }

  EXPECT_EQ(union_find.FindRoot(0), union_find.FindRoot(2));
  EXPECT_EQ(union_find.FindRoot(3), union_find.FindRoot(9));
  EXPECT_EQ(union_find.FindRoot(1), union_find.FindRoot(4));
  EXPECT_EQ(union_find.FindRoot(1), union_find.FindRoot(5));
  EXPECT_EQ(union_find.FindRoot(1), union_find.FindRoot(6));
  EXPECT_EQ(union_find.FindRoot(1), union_find.FindRoot(7));
  EXPECT_EQ(union_find.FindRoot(4), union_find.FindRoot(5));
  EXPECT_EQ(union_find.FindRoot(4), union_find.FindRoot(6));
  EXPECT_EQ(union_find.FindRoot(4), union_find.FindRoot(7));
  EXPECT_EQ(union_find.FindRoot(5), union_find.FindRoot(6));
  EXPECT_EQ(union_find.FindRoot(5), union_find.FindRoot(7));
  EXPECT_EQ(union_find.FindRoot(6), union_find.FindRoot(7));
}

TEST(UNION_FIND_TEST, TEST_INIT_WITH_NODES) {
  const int size = 10;
  UnionFind union_find(size);

  const std::vector<size_t> nodes = {2, 5, 8, 9, 12, 13, 15, 17, 20, 21};
  union_find.InitWithNodes(nodes);

  std::vector<std::pair<int, int>> sets = {{2, 8},   {12, 13}, {9, 21},
                                           {13, 17}, {15, 17}, {5, 12}};
  for (auto set : sets) {
    union_find.Union(set.first, set.second);
  }

  EXPECT_EQ(union_find.FindRoot(2), union_find.FindRoot(8));
  EXPECT_EQ(union_find.FindRoot(9), union_find.FindRoot(21));
  EXPECT_EQ(union_find.FindRoot(5), union_find.FindRoot(12));
  EXPECT_EQ(union_find.FindRoot(5), union_find.FindRoot(13));
  EXPECT_EQ(union_find.FindRoot(5), union_find.FindRoot(15));
  EXPECT_EQ(union_find.FindRoot(5), union_find.FindRoot(17));
  EXPECT_EQ(union_find.FindRoot(12), union_find.FindRoot(13));
  EXPECT_EQ(union_find.FindRoot(12), union_find.FindRoot(15));
  EXPECT_EQ(union_find.FindRoot(12), union_find.FindRoot(17));
  EXPECT_EQ(union_find.FindRoot(13), union_find.FindRoot(15));
  EXPECT_EQ(union_find.FindRoot(13), union_find.FindRoot(17));
  EXPECT_EQ(union_find.FindRoot(15), union_find.FindRoot(17));
}

TEST(UNION_FIND_TEST, TEST_CONNECTED_COMPONENTS) {
  const int size = 10;
  UnionFind union_find(size);

  std::vector<std::pair<int, int>> sets = {{0, 2}, {4, 5}, {3, 9},
                                           {5, 7}, {6, 7}, {1, 4}};
  for (auto set : sets) {
    union_find.Union(set.first, set.second);
  }

  const std::unordered_set<size_t> components =
      union_find.GetConnectedComponents();
  EXPECT_EQ(components.size(), 4);
  // for (auto parent_id : components) {
  //     std::cout << parent_id << " ";
  // }
  // std::cout << std::endl;
}
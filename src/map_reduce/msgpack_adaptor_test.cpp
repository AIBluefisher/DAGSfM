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

#include "msgpack_adaptor.h"

#include <gtest/gtest.h>

#include <iostream>

TEST(EIGEN_MSGPACK_ADPATOR_TEST, test_vector2d) {
  Eigen::Vector2d v(1.0, 2.0);
  std::cout << "Original vector: " << v(0, 0) << ", " << v(1, 0) << std::endl;

  std::stringstream ss;
  clmdep_msgpack::pack(ss, v);

  const std::string& str = ss.str();
  clmdep_msgpack::object_handle oh =
      clmdep_msgpack::unpack(str.data(), str.size());
  clmdep_msgpack::object obj = oh.get();
  Eigen::Vector2d vb = obj.as<Eigen::Vector2d>();

  ASSERT_EQ(obj.as<Eigen::Vector2d>(), v);
  std::cout << "Unpacked vector: " << vb(0, 0) << ", " << vb(1, 0) << std::endl;
}

TEST(EIGEN_MSGPACK_ADPATOR_TEST, test_vector3d) {
  Eigen::Vector3d v(1.0, 2.0, 3.0);
  std::cout << "Original vector: " << v(0, 0) << ", " << v(1, 0) << ", "
            << v(2, 0) << std::endl;

  std::stringstream ss;
  clmdep_msgpack::pack(ss, v);

  const std::string& str = ss.str();
  clmdep_msgpack::object_handle oh =
      clmdep_msgpack::unpack(str.data(), str.size());
  clmdep_msgpack::object obj = oh.get();
  Eigen::Vector3d vb = obj.as<Eigen::Vector3d>();

  ASSERT_EQ(obj.as<Eigen::Vector3d>(), v);
  std::cout << "Unpacked vector: " << vb(0, 0) << ", " << vb(1, 0) << ", "
            << vb(2, 0) << std::endl;
}

TEST(EIGEN_MSGPACK_ADPATOR_TEST, test_vector4d) {
  Eigen::Vector4d v(1.0, 2.0, 3.0, 4.0);
  std::cout << "Original vector: " << v(0, 0) << ", " << v(1, 0) << ", "
            << v(2, 0) << ", " << v(3, 0) << std::endl;

  std::stringstream ss;
  clmdep_msgpack::pack(ss, v);

  const std::string& str = ss.str();
  clmdep_msgpack::object_handle oh =
      clmdep_msgpack::unpack(str.data(), str.size());
  clmdep_msgpack::object obj = oh.get();
  Eigen::Vector4d vb = obj.as<Eigen::Vector4d>();

  ASSERT_EQ(obj.as<Eigen::Vector4d>(), v);
  std::cout << "Unpacked vector: " << vb(0, 0) << ", " << vb(1, 0) << ", "
            << vb(2, 0) << ", " << vb(3, 0) << std::endl;
}

TEST(EIGEN_MSGPACK_ADPATOR_TEST, test_vector3ub) {
  Eigen::Vector3ub v = Eigen::Matrix<uint8_t, 3, 1>::Zero();
  std::cout << "Original vector: " << v(0, 0) << ", " << v(1, 0) << ", "
            << v(2, 0) << std::endl;

  std::stringstream ss;
  clmdep_msgpack::pack(ss, v);

  const std::string& str = ss.str();
  clmdep_msgpack::object_handle oh =
      clmdep_msgpack::unpack(str.data(), str.size());
  clmdep_msgpack::object obj = oh.get();
  Eigen::Vector3ub vb = obj.as<Eigen::Vector3ub>();

  ASSERT_EQ(obj.as<Eigen::Vector3ub>(), v);
  std::cout << "Unpacked vector: " << vb(0, 0) << ", " << vb(1, 0) << ", "
            << vb(2, 0) << std::endl;
}

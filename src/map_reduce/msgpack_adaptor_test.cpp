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

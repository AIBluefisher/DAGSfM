#ifndef _SRC_MAP_REDUCE_MSGPACK_ADAPTOR_H_
#define _SRC_MAP_REDUCE_MSGPACK_ADAPTOR_H_

#include <Eigen/Core>
#include <rpc/msgpack.hpp>

#include "util/types.h"

// This class implements the interface that is used for packing and
// unpacking Eigen data types.
// Ref:
// https://github.com/msgpack/msgpack-c/blob/master/example/cpp03/class_non_intrusive.cpp

namespace clmdep_msgpack {
MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
  namespace adaptor {
  /////////////////////////////////////////////
  // Adaptor for Eigen data type. /////////////
  /////////////////////////////////////////////
  template <typename T, int R, int C, int O>
  struct convert<Eigen::Matrix<T, R, C, O>> {
    clmdep_msgpack::object const& operator()(
        clmdep_msgpack::object const& o, Eigen::Matrix<T, R, C, O>& v) const {
      if (o.type != clmdep_msgpack::type::ARRAY)
        throw clmdep_msgpack::type_error();
      if (o.via.array.size != R * C) throw clmdep_msgpack::type_error();

      int k = 0;
      for (int i = 0; i < R; ++i) {
        for (int j = 0; j < C; ++j) {
          v(i, j) = o.via.array.ptr[k++].as<T>();
        }
      }
      return o;
    }
  };

  template <typename T, int R, int C, int O>
  struct pack<Eigen::Matrix<T, R, C, O>> {
    template <typename Stream>
    packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o,
                               Eigen::Matrix<T, R, C, O> const& v) const {
      // packing member variables as an array.
      o.pack_array(R * C);

      for (int i = 0; i < R; ++i) {
        for (int j = 0; j < C; ++j) {
          o.pack(v(i, j));
        }
      }

      return o;
    }
  };

  template <typename T, int R, int C, int O>
  struct object_with_zone<Eigen::Matrix<T, R, C, O>> {
    void operator()(clmdep_msgpack::object::with_zone& o,
                    Eigen::Matrix<T, R, C, O> const& v) const {
      o.type = type::ARRAY;
      o.via.array.size = R * C;
      o.via.array.ptr =
          static_cast<clmdep_msgpack::object*>(o.zone.allocate_align(
              sizeof(clmdep_msgpack::object) * o.via.array.size,
              MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object)));

      int k = 0;
      for (int i = 0; i < R; ++i) {
        for (int j = 0; j < C; ++j) {
          o.via.array.ptr[k++] = clmdep_msgpack::object(v(i, j), o.zone);
        }
      }
    }
  };

  }  // namespace adaptor
}  // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
}  // namespace clmdep_msgpack

#endif
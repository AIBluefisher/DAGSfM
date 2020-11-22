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

#ifndef _SRC_MAPREDUCE_RUNNING_INFO_H_
#define _SRC_MAPREDUCE_RUNNING_INFO_H_

#include <rpc/client.h>

#include <rpc/msgpack.hpp>
#include <string>

namespace DAGSfM {

namespace {
inline std::string ConnectStatus(const rpc::client::connection_state cs) {
  switch (cs) {
    case rpc::client::connection_state::connected:
      return "CONNECTED";
      break;
    case rpc::client::connection_state::disconnected:
      return "DISCONNECTED";
      break;
    case rpc::client::connection_state::initial:
      return "INITIAL";
      break;
    case rpc::client::connection_state::reset:
      return "RESET";
      break;
  }
}
}  // namespace

struct RunningInfo {
  RunningInfo(){};

  RunningInfo(const RunningInfo& other) {
    is_reachable = other.is_reachable;
    idle = other.idle;
    in_progress = other.in_progress;
    completed = other.completed;
    running_time = other.running_time;
  }

  bool is_reachable = false;

  bool idle = true;

  bool in_progress = false;

  bool completed = false;

  double running_time = 0.0;

  // MSGPACK_DEFINE(idle, in_progress, completed, ip, running_time);
};

struct SfMRunningInfo : RunningInfo {
  SfMRunningInfo(){};

  SfMRunningInfo(const SfMRunningInfo& other) {
    is_reachable = other.is_reachable;
    idle = other.idle;
    in_progress = other.in_progress;
    completed = other.completed;
    running_time = other.running_time;
    total_image_num = other.total_image_num;
    registered_image_num = other.registered_image_num;
    total_matching_pairs = other.total_matching_pairs;
    matched_image_pairs = other.matched_image_pairs;
  }

  size_t total_image_num = 0;

  size_t registered_image_num = 0;

  size_t total_matching_pairs = 0;
  size_t matched_image_pairs = 0;

  void Reset() {
    is_reachable = false;
    idle = true;
    in_progress = false;
    completed = false;
    running_time = 0.0;
    total_image_num = 0;
    registered_image_num = 0;
    total_matching_pairs = 0;
    matched_image_pairs = 0;
  }

  MSGPACK_DEFINE(is_reachable, idle, in_progress, completed, running_time,
                 total_image_num, registered_image_num, total_matching_pairs,
                 matched_image_pairs);
};

}  // namespace DAGSfM

#endif
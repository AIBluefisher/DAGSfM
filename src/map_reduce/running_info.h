#ifndef _SRC_MAPREDUCE_RUNNING_INFO_H_
#define _SRC_MAPREDUCE_RUNNING_INFO_H_

#include <rpc/client.h>

#include <rpc/msgpack.hpp>
#include <string>

namespace GraphSfM {

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

}  // namespace GraphSfM

#endif
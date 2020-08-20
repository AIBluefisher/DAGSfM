#include "map_reduce/worker.h"

#include <fstream>

#include "rpc/this_server.h"

namespace GraphSfM {

uint16_t Worker::kRPCDefaultPort = 8080;

SfMWorker::SfMWorker() : Worker() {
  server_.bind("ResetWorkerInfo", [this]() { info_.Reset(); });

  server_.bind("GetRunningInfo", [this]() {
    LOG(INFO) << "Worker get running info";
    return info_;
  });

  server_.bind("SetNonIdle", [this]() { this->SetIdle(false); });

  server_.bind("Stop", []() { rpc::this_server().stop(); });
}

rpc::server& SfMWorker::Server() { return server_; }

const SfMRunningInfo SfMWorker::GetRunningInfo() const { return info_; }

void SfMWorker::SetIdle(const bool idle) { info_.idle = idle; }

void SfMWorker::SetInprogress(const bool in_progress) {
  info_.in_progress = false;
}

void SfMWorker::SetCompleted(const bool is_completed) {
  info_.completed = is_completed;
}

void SfMWorker::SetTotalImageNum(const size_t image_num) {
  info_.total_image_num = image_num;
}

void SfMWorker::SetRegImageNum(const size_t reg_image_num) {
  info_.registered_image_num = reg_image_num;
}

void SfMWorker::SetTotalMatchingPairs(const size_t matching_pairs) {
  info_.total_matching_pairs = matching_pairs;
}

void SfMWorker::SetMatchedPairs(const size_t matched_pairs) {
  info_.matched_image_pairs = matched_pairs;
}

}  // namespace GraphSfM
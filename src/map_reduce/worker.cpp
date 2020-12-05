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

#include "map_reduce/worker.h"

#include <fstream>

#include "rpc/this_server.h"

namespace DAGSfM {

// uint16_t Worker::kRPCDefaultPort = 8080;

SfMWorker::SfMWorker() : Worker() {}

bool SfMWorker::BindSfMBaseFuncs() {
  if (server_ == nullptr) {
    return false;
  }

  server_->bind("ResetWorkerInfo", [this]() { info_.Reset(); });
  server_->bind("GetRunningInfo", [this]() { return info_; });
  server_->bind("SetNonIdle", [this]() { this->SetIdle(false); });
  server_->bind("Stop", []() { rpc::this_server().stop(); });

  return true;
}

rpc::server* SfMWorker::Server() { return server_.get(); }

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

}  // namespace DAGSfM
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

#ifndef _SRC_MAPREDUCE_WORKER_H_
#define _SRC_MAPREDUCE_WORDER_H_

#include <rpc/client.h>
#include <rpc/server.h>
#include <rpc/this_server.h>

#include <fstream>
#include <memory>

#include "map_reduce/running_info.h"
#include "util/misc.h"

namespace std {
template <typename T, typename... Ts>
std::unique_ptr<T> make_unique(Ts&&... params) {
  return std::unique_ptr<T>(new T(std::forward<Ts>(params)...));
}
}  // namespace std

namespace DAGSfM {

class Worker {
 public:
  Worker() : server_(nullptr) {}

  inline void RunServer() { server_->run(); }

  inline void BindServer(const uint16_t port = 8080) {
    server_ = std::make_unique<rpc::server>(port);

    server_->bind("StopServer", []() { rpc::this_server().stop(); });

    server_->bind("SaveImage", [](const std::string& image_path,
                                  const std::string& image_name,
                                  const std::vector<char>& buffer, int length) {
      if (!colmap::ExistsDir(image_path)) {
        boost::filesystem::create_directories(image_path);
      }

      std::ofstream of(colmap::JoinPaths(image_path, image_name).c_str(),
                       std::ifstream::out | std::ifstream::binary);
      of.write(buffer.data(), length);
      of.close();

      return true;
    });
  }

 protected:
  // static uint16_t kRPCDefaultPort;
  std::unique_ptr<rpc::server> server_;
};

class SfMWorker : public Worker {
 public:
  SfMWorker();

  rpc::server* Server();

  bool BindSfMBaseFuncs();

  void SetIdle(const bool idle);

  void SetInprogress(const bool in_progress);

  void SetCompleted(const bool is_completed);

  void SetTotalImageNum(const size_t image_num);

  void SetRegImageNum(const size_t reg_image_num);

  void SetTotalMatchingPairs(const size_t matching_pairs);

  void SetMatchedPairs(const size_t matched_pairs);

  const SfMRunningInfo GetRunningInfo() const;

 protected:
  SfMRunningInfo info_;
};

}  // namespace DAGSfM

#endif
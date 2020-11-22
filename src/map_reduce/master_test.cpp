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

#include "map_reduce/master.h"

#include <chrono>
#include <thread>

using namespace DAGSfM;

int main() {
  MapReduceConfig map_reduce_config;
  map_reduce_config.server_ips.push_back("localhost");
  map_reduce_config.server_ports.push_back(uint16_t(8080));

  SfMMaster sfm_master;
  sfm_master.SetMapReduceConfig(map_reduce_config);

  rpc::client c("localhost", 8080);

  for (int i = 0; i < 10; i++) {
    c.call("HelloWorld");
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  c.call("Exit");

  // std::string master_image_path = "";
  // std::string worker_image_path = "/home/chenyu/Pictures/dataset/pku";
  // std::vector<std::string> image_names = {"DJI_0221.JPG", "DJI_0222.JPG"};
  // LOG(INFO) << "Run distributed mode";
  // sfm_master.RunDistributed();
  // sfm_master.CallSaveImages(0, master_image_path, worker_image_path,
  // image_names);
}
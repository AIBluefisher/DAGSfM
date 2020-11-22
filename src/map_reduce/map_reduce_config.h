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

#ifndef _SRC_MAPREDUCE_CONFIG_H_
#define _SRC_MAPREDUCE_CONFIG_H_

#include <glog/logging.h>

#include <fstream>
#include <iostream>
#include <string>

namespace DAGSfM {

struct MapReduceConfig {
  size_t num_machine = 0;

  std::vector<std::string> server_ips;

  std::vector<uint16_t> server_ports;

  std::vector<std::string> image_paths;

  bool ReadConfig(std::string& filename) {
    size_t n;
    std::string ip;
    uint16_t port;
    std::string image_path;

    std::ifstream config_file(filename);
    if (!config_file.is_open()) {
      LOG(ERROR) << "Can't open file " << filename;
      return false;
    }

    config_file >> n;
    CHECK_GT(n, 0) << "No server provided!";
    server_ips.reserve(n);
    server_ports.reserve(n);
    image_paths.reserve(n);

    for (size_t i = 0; i < n; i++) {
      config_file >> ip >> port >> image_path;
      server_ips.push_back(ip);
      server_ports.push_back(port);
      image_paths.push_back(image_path);
    }

    return true;
  }
};

}  // namespace DAGSfM

#endif
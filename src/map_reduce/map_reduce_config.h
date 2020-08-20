#ifndef _SRC_MAPREDUCE_CONFIG_H_
#define _SRC_MAPREDUCE_CONFIG_H_

#include <glog/logging.h>

#include <fstream>
#include <iostream>
#include <string>

namespace GraphSfM {

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

}  // namespace GraphSfM

#endif
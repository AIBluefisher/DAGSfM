#ifndef _SRC_MAPREDUCE_CONFIG_H_
#define _SRC_MAPREDUCE_CONFIG_H_

#include <iostream>
#include <string>
#include <fstream>

#include <glog/logging.h>

namespace GraphSfM {

struct MapReduceConfig
{
    size_t num_machine = 0;
    
    // size_t mapper_magabytes = 0;

    // size_t reducer_mega_bytes = 0;

    std::vector<std::string> server_ips;

    std::vector<uint16_t> server_ports;

    bool ReadConfig(std::string& filename)
    {
        size_t n;
        std::string ip;
        uint16_t port;

        std::ifstream config_file(filename);
        if (!config_file.is_open()) {
            LOG(ERROR) << "Can't open file " << filename;
            return false;
        }
        
        config_file >> n;
        CHECK_GT(n, 0) << "No server provided!";
        server_ips.reserve(n);
        server_ports.reserve(n);

        for (size_t i = 0; i < n; i++) {
            config_file >> ip >> port;
            server_ips.push_back(ip);
            server_ports.push_back(port);
        }

        return true;
    }
};

} // namespace GraphSfM

#endif
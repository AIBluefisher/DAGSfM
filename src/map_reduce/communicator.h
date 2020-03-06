#ifndef _SRC_MAPREDUCE_COMMUNICATOR_H_
#define _SRC_MAPREDUCE_COMMUNICATOR_H_

#include <string>
#include <unordered_map>
#include <thread>
#include <mutex>

#include "rpc/server.h"
#include "rpc/client.h"
#include "rpc/config.h"
#include "rpc/this_server.h"
#include "map_reduce/running_info.h"
// #include "controllers/incremental_mapper_controller.h"
#include "base/reconstruction_manager.h"

#include <glog/logging.h>

using namespace colmap;

namespace GraphSfM {

struct CommunicatorOptions
{
    std::string local_ip = "127.0.0.1";
    
    uint16_t port = uint16_t(8080);

    size_t max_connect_num = 120;

    size_t connect_duration = 1;
};

// A communicator is a server and also a client.
class Communicator
{
public:
    Communicator();

    Communicator(const CommunicatorOptions& options);

    void Listening(const bool async);

    bool IsReachable(const std::string& ip, const uint16_t port) const;

    rpc::server& Server();

protected:
    CommunicatorOptions options_;

    rpc::server server_;
};
 

} // namespace GraphSfM

#endif
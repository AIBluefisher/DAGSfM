#ifndef _SRC_MAPREDUCE_WORKER_H_
#define _SRC_MAPREDUCE_WORDER_H_

#include <memory>

#include "map_reduce/communicator.h"
#include "map_reduce/running_info.h"
#include "rpc/server.h"
#include "rpc/client.h"

namespace GraphSfM {

class Worker
{
public:
    Worker() : server_(rpc::server(Worker::kRPCDefaultPort)) 
    {
        // port_ = kRPCDefaultPort++;

        server_.bind("StopServer", []() {
            rpc::this_server().stop();
        });
    }

    inline void InitializeServer()
    {
        server_ = std::move(rpc::server(8080));
    }

    inline void RunServer()
    {
        server_.run();
    }

protected:
    // uint16_t port_;
    static uint16_t kRPCDefaultPort;
    rpc::server server_;
};

class SfMWorker : public Worker
{
public:
    SfMWorker();

    rpc::server& Server();

    void SetIdle(const bool idle);

    void SetInprogress(const bool in_progress);

    void SetCompleted(const bool is_completed);

    void SetIP(const std::string& ip);

    void SetTotalImageNum(const size_t image_num);

    void SetRegImageNum(const size_t reg_image_num);

    const SfMRunningInfo GetRunningInfo() const;

protected:
    SfMRunningInfo info_;
};

} // namespace GraphSfM

#endif
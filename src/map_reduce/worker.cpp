#include "map_reduce/worker.h"
#include "rpc/this_server.h"

namespace GraphSfM {

uint16_t Worker::kRPCDefaultPort = 8080;

SfMWorker::SfMWorker()
    : Worker()
{
    server_.bind("GetRunningInfo", [this]() {
        return info_;
    });

    server_.bind("SetNonIdle", [this]() {
        this->SetIdle(false);
    });

    server_.bind("Stop", []() {
        rpc::this_server().stop();
    });
}

rpc::server& SfMWorker::Server()
{
    return server_;
}

const SfMRunningInfo SfMWorker::GetRunningInfo() const
{
    return info_;
}

void SfMWorker::SetIdle(const bool idle)
{
    info_.idle = idle;
}

void SfMWorker::SetInprogress(const bool in_progress)
{
    info_.in_progress = false;
}

void SfMWorker::SetCompleted(const bool is_completed)
{
    info_.completed = is_completed;
}

void SfMWorker::SetIP(const std::string& ip)
{
    info_.ip = ip;
}

void SfMWorker::SetTotalImageNum(const size_t image_num)
{
    info_.total_image_num = image_num;
}

void SfMWorker::SetRegImageNum(const size_t reg_image_num)
{
    info_.registered_image_num = reg_image_num;
}

} // namespace GraphSfM
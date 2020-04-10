#ifndef _SRC_MAPREDUCE_MASTER_H_
#define _SRC_MAPREDUCE_MASTER_H_

#include <unordered_map>
#include <vector>
#include <iomanip>
#include <cstdlib>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <fstream>

#include "map_reduce/communicator.h"
#include "map_reduce/running_info.h"
#include "map_reduce/map_reduce_config.h"
#include "map_reduce/map_reduce_result.h"
#include "util/threading.h"
#include "util/timer.h"
#include "util/misc.h"

#include <glog/logging.h>

namespace GraphSfM {

class Master
{
public:
    Master() {}

    virtual bool RunSequential() = 0;
    virtual bool RunDistributed() = 0;

    inline void SetMapReduceConfig(
            const MapReduceConfig& map_reduce_config)
    {
        map_reduce_config_ = map_reduce_config;
    }

    inline virtual void ShowProgress() = 0;

    inline virtual void ShowResult() const = 0;

    virtual int GetIdleWorker() const = 0;

    virtual bool AllServerCompleted() const = 0;

    inline void StopWorker(const size_t id)
    {
        rpc::client c(map_reduce_config_.server_ips[id],
                      map_reduce_config_.server_ports[id]);
        c.async_call("Stop");
    }

    inline void CallSaveImages(const int id,
                               const std::string& master_image_path,
                               const std::string& worker_image_path,
                               const std::vector<std::string>& image_names)
    {
        rpc::client c(map_reduce_config_.server_ips[id],
                      map_reduce_config_.server_ports[id]);
        for (auto image_name : image_names) {
            std::fstream is(
                colmap::JoinPaths(master_image_path, image_name).c_str(), 
                std::ifstream::in | std::ifstream::binary);
            // Compute buffer length.
            is.seekg(0, is.end);
            int length = is.tellg();
            is.seekg(0, is.beg);

            // Create buffers.
            // char* buffer = new char [length];
            std::vector<char> buffer(length);

            // Read image to buffer.
            is.read(buffer.data(), length);

            // Remote procedure call to save image.
            c.call("SaveImage", worker_image_path, image_name, buffer, length);

            // delete [] buffer;
            is.close();
        }
    }

    inline void CallSaveImages(const int id,
                               const std::string& master_image_path,
                               const std::string& worker_image_path,
                               const std::unordered_set<std::string>& image_names)
    {
        rpc::client c(map_reduce_config_.server_ips[id],
                      map_reduce_config_.server_ports[id]);
        for (auto image_name : image_names) {
            std::fstream is(
                colmap::JoinPaths(master_image_path, image_name).c_str(), 
                std::ifstream::in | std::ifstream::binary);
            // Compute buffer length.
            is.seekg(0, is.end);
            int length = is.tellg();
            is.seekg(0, is.beg);

            // Create buffers.
            // char* buffer = new char [length];
            std::vector<char> buffer(length);

            // Read image to buffer.
            is.read(buffer.data(), length);

            // Remote procedure call to save image.
            c.call("SaveImage", worker_image_path, image_name, buffer, length);

            // delete [] buffer;
            is.close();
        }
    }

protected:
    MapReduceConfig map_reduce_config_;

    MapReduceResult map_reduce_result_;
};

class SfMMaster : public Master
{
public:
    SfMMaster();

    virtual bool RunSequential() override { return true; }

    virtual bool RunDistributed() override 
    {
        // Currently just an implementation for playing and testing.
        CHECK_GT(map_reduce_config_.server_ips.size(), 0);

        int idle_server_id = GetIdleWorker();
        while (idle_server_id != -1) {
            // ShowProgress();
            LOG(INFO) << "Idle server id: " << idle_server_id;
            rpc::client c(map_reduce_config_.server_ips[idle_server_id],
                          map_reduce_config_.server_ports[idle_server_id]);
            c.call("SetNonIdle");
            timers_[idle_server_id].Start();

            // Distributing your tasks here.
            // ......
            // For example: c.async_call("RunSfM");
            std::this_thread::sleep_for(std::chrono::seconds(1));

            CallGetRunningInfo(idle_server_id, false);
            // if (worker_infos_[idle_server_id].completed) {
            //     StopWorker(idle_server_id);
            // }

            idle_server_id = GetIdleWorker();
        }

        while (!AllServerCompleted()) {
            ShowProgress();
            UpdateRunningInfo(false);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        return true; 
    }
    
    virtual int GetIdleWorker() const override;

    virtual bool AllServerCompleted() const override;

    virtual void ShowProgress() override;

    virtual void ShowResult() const override;

    void CallRunSfM(const size_t id,
                    const bool async);

    void CallGetRunningInfo(const size_t id,
                            const bool async);

    void UpdateRunningInfo(const bool async);

protected:
    std::unordered_map<size_t, SfMRunningInfo> worker_infos_;

    std::unordered_map<size_t, colmap::Timer> timers_;

    MapReduceResult map_reduce_result_;

private:
    const SfMRunningInfo GetRunningInfo(const size_t id,
                                        const bool async) const;

    
};

} // namespace GraphSfM

#endif
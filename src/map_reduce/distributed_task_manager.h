#ifndef _SRC_MAPREDUCE_DISTRIBUTED_TASK_MANAGER_H_
#define _SRC_MAPREDUCE_DISTRIBUTED_TASK_MANAGER_H_

#include <rpc/client.h>

#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "base/database_cache.h"
#include "base/reconstruction.h"
#include "clustering/image_clustering.h"
#include "map_reduce/master.h"
#include "util/types.h"

namespace GraphSfM {

enum class DistributedDataType { SFM, MATCHING };

struct TaskDataContainer {
  std::vector<std::string> server_ips;
  std::vector<uint16_t> server_ports;
  size_t cluster_num = 0;

  std::string master_image_path;
  std::unordered_map<size_t, std::vector<std::string>> cluster_images;

  std::string task_type = "";

  virtual void DistributeTask(const size_t job_id, const int worker_id){};
  virtual void ReduceTask(const int worker_id){};
  virtual void MergeData(){};
};

struct SfMDataContainer : public TaskDataContainer {
  std::vector<Reconstruction> reconstructions;
  std::unordered_map<size_t, DatabaseCache> cluster_database_caches;

  virtual void DistributeTask(const size_t job_id,
                              const int worker_id) override {
    rpc::client c(server_ips[worker_id], server_ports[worker_id]);
    LOG(INFO) << "Call run sfm";
    c.async_call("RunSfM", cluster_database_caches[job_id]);
    LOG(INFO) << "end call RunSfM";
    c.call("SetNonIdle");
  }

  virtual void ReduceTask(const int worker_id) override {
    rpc::client c(server_ips[worker_id], server_ports[worker_id]);

    auto local_recons_obj = c.call("GetLocalRecons");
    Reconstruction recon = local_recons_obj.get().as<Reconstruction>();

    reconstructions.emplace_back(recon);

    c.async_call("ResetWorkerInfo");
  }

  virtual void MergeData() override {
    // implement outside.
  }
};

template <typename DataType>
class DistributedTaskManager : public SfMMaster {
 public:
  DistributedTaskManager();

  inline const DataType& DataContainer() const;
  inline DataType& DataContainer();

  inline size_t ClusterNum();

  inline size_t WaittingJobNum();
  inline void Push(const size_t job_id);
  inline size_t Pop();

  inline int GetWorkingWorkerId(const size_t job_id);
  inline bool RemoveWorkingJob(const size_t job_id);
  inline void InsertWorkingJob(const size_t job_id, const int worker_id);
  std::vector<size_t> GetWorkingJobIds();

  inline size_t GetFinishedJobNum();
  inline void IncrementFinishedJobNum();

  virtual bool RunSequential();
  virtual bool RunDistributed();

  void SetDataContainer(const DataType& data_container);

 private:
  void ReduceTasks();
  void DistributeTasks();
  void MonitorTasks();

 private:
  DataType data_container_;
  std::mutex data_mutex_;

  // TODO: (chenyu) Consider to use a priority_queue.
  std::queue<size_t> job_queue_;
  std::mutex job_queue_mutex_;

  // { (job_id, worker_id) }.
  std::unordered_map<size_t, int> working_jobs_;
  std::mutex working_jobs_mutex_;

  size_t finished_job_num_;
  std::mutex finished_jobs_mutex_;
};

template <typename DataType>
inline const DataType& DistributedTaskManager<DataType>::DataContainer() const {
  return data_container_;
}

template <typename DataType>
inline DataType& DistributedTaskManager<DataType>::DataContainer() {
  return data_container_;
}

template <typename DataType>
inline size_t DistributedTaskManager<DataType>::ClusterNum() {
  // size_t cluster_num = 0;
  // data_mutex_.lock();
  // cluster_num = data_container_.cluster_num;
  // data_mutex_.unlock();
  // return cluster_num;
  return data_container_.cluster_num;
}

template <typename DataType>
inline size_t DistributedTaskManager<DataType>::WaittingJobNum() {
  std::lock_guard<std::mutex> guard(job_queue_mutex_);
  return job_queue_.size();
}

template <typename DataType>
inline void DistributedTaskManager<DataType>::Push(const size_t job_id) {
  std::lock_guard<std::mutex> guard(job_queue_mutex_);
  job_queue_.push(job_id);
}

template <typename DataType>
inline size_t DistributedTaskManager<DataType>::Pop() {
  size_t job_id;
  std::lock_guard<std::mutex> guard(job_queue_mutex_);
  job_id = job_queue_.front();
  job_queue_.pop();

  return job_id;
};

template <typename DataType>
inline int DistributedTaskManager<DataType>::GetWorkingWorkerId(
    const size_t job_id) {
  int worker_id = -1;

  std::lock_guard<std::mutex> guard(working_jobs_mutex_);
  if (working_jobs_.count(job_id) > 0) {
    worker_id = working_jobs_.at(job_id);
  }

  return worker_id;
}

template <typename DataType>
inline bool DistributedTaskManager<DataType>::RemoveWorkingJob(
    const size_t job_id) {
  std::lock_guard<std::mutex> guard(working_jobs_mutex_);
  if (working_jobs_.count(job_id) == 0) {
    return false;
  }

  working_jobs_.erase(job_id);

  return true;
}

template <typename DataType>
inline void DistributedTaskManager<DataType>::InsertWorkingJob(
    const size_t job_id, const int worker_id) {
  std::lock_guard<std::mutex> guard(working_jobs_mutex_);
  working_jobs_.emplace(job_id, worker_id);
}

template <typename DataType>
inline std::vector<size_t>
DistributedTaskManager<DataType>::GetWorkingJobIds() {
  std::vector<size_t> working_job_ids;

  working_jobs_mutex_.lock();
  for (const auto& working_job : working_jobs_) {
    working_job_ids.push_back(working_job.first);
  }
  working_jobs_mutex_.unlock();

  return working_job_ids;
};

template <typename DataType>
inline size_t DistributedTaskManager<DataType>::GetFinishedJobNum() {
  size_t finished_job_num = 0;

  finished_jobs_mutex_.lock();
  finished_job_num = finished_job_num_;
  finished_jobs_mutex_.unlock();

  return finished_job_num;
};

template <typename DataType>
inline void DistributedTaskManager<DataType>::IncrementFinishedJobNum() {
  finished_jobs_mutex_.lock();
  finished_job_num_++;
  finished_jobs_mutex_.unlock();
};

}  // namespace GraphSfM

#include "map_reduce/distributed_task_manager.inl"

#endif
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

#include "map_reduce/distributed_task_manager.h"

namespace DAGSfM {

template <typename DataType>
DistributedTaskManager<DataType>::DistributedTaskManager()
    : finished_job_num_(0) {}

template <typename DataType>
bool DistributedTaskManager<DataType>::RunSequential() {
  // Do nothing.
  return true;
}

template <typename DataType>
bool DistributedTaskManager<DataType>::RunDistributed() {
  std::thread task_reduce_thread{&DistributedTaskManager<DataType>::ReduceTasks,
                                 this};
  std::thread task_distribute_thread{
      &DistributedTaskManager<DataType>::DistributeTasks, this};
  std::thread task_monitor_thread{
      &DistributedTaskManager<DataType>::MonitorTasks, this};

  task_distribute_thread.join();
  task_reduce_thread.join();
  task_monitor_thread.join();

  return true;
}

template <typename DataType>
void DistributedTaskManager<DataType>::SetDataContainer(
    const DataType& data_container) {
  data_container_ = data_container;
}

template <typename DataType>
void DistributedTaskManager<DataType>::ReduceTasks() {
  while (GetFinishedJobNum() != ClusterNum()) {
    const std::vector<size_t> working_job_ids = GetWorkingJobIds();

    for (const size_t job_id : working_job_ids) {
      const int worker_id = GetWorkingWorkerId(job_id);

      if (worker_id == -1) {
        continue;
      }

      const SfMRunningInfo running_info = GetRunningInfo(worker_id);

      if (running_info.completed) {
        IncrementFinishedJobNum();
        RemoveWorkingJob(job_id);

        // Reset timers.
        PauseWorkerTimer(worker_id);

        // Retrieve task data once a job is completed.
        data_container_.ReduceTask(worker_id);

        // Reset worker infomation.
        ResetRunningInfo(worker_id);
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }
}

template <typename DataType>
void DistributedTaskManager<DataType>::DistributeTasks() {
  while (WaittingJobNum() > 0 && (GetFinishedJobNum() != ClusterNum())) {
    int idle_worker_id = GetIdleWorker();

    if (idle_worker_id == -1) {
      // Sleep this thread to avoid get into a dead loop.
      std::this_thread::sleep_for(std::chrono::seconds(3));
      continue;
    }

    const size_t job_id = Pop();

    if (data_container_.cluster_images.count(job_id) != 0) {
      const std::vector<std::string>& image_names =
          data_container_.cluster_images[job_id];

      LOG(INFO) << "Transferring images to worker #" << idle_worker_id << ".";
      CallSaveImages(
          idle_worker_id, data_container_.master_image_path,
          colmap::JoinPaths(map_reduce_config_.image_paths[idle_worker_id],
                            std::to_string(job_id) + "/images"),
          image_names);
      LOG(INFO) << "Transferring images to worker #" << idle_worker_id
                << " completed.";
    }

    // Distributing a job to an idle worker.
    data_container_.DistributeTask(job_id, idle_worker_id);

    StartWorkerTimer(idle_worker_id);

    InsertWorkingJob(job_id, idle_worker_id);

    CallGetRunningInfo(idle_worker_id, true);

    std::this_thread::sleep_for(std::chrono::seconds(3));
  }
}

template <typename DataType>
void DistributedTaskManager<DataType>::MonitorTasks() {
  do {
    ShowProgress(data_container_.task_type);
    UpdateRunningInfo(true);

    std::this_thread::sleep_for(std::chrono::seconds(1));
  } while (GetFinishedJobNum() != ClusterNum());
}

}  // namespace DAGSfM
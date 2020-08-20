#include "map_reduce/distributed_task_manager.h"

namespace GraphSfM {

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
    LOG(INFO) << "start update running info";
    UpdateRunningInfo(true);
    LOG(INFO) << "end update running info";

    std::this_thread::sleep_for(std::chrono::seconds(1));
  } while (GetFinishedJobNum() != ClusterNum());
}

}  // namespace GraphSfM
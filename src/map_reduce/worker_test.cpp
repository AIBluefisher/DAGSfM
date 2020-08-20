#include "map_reduce/worker.h"

#include <rpc/this_session.h>

#include <chrono>
#include <thread>

using namespace GraphSfM;

int main() {
  SfMWorker sfm_worker;
  // sfm_worker.RunServer();

  bool in_progress = false;
  bool completed = false;
  size_t total_image_num = 100;
  size_t reg_image_num = 0;

  sfm_worker.SetTotalImageNum(total_image_num);
  // sfm_worker.SetIP("localhost");

  rpc::server& server = sfm_worker.Server();
  // rpc::server server(8080);
  bool exit = false;

  server.bind("HelloWorld", []() { std::cout << "Hello World!\n\n"; });

  server.bind("Exit", [&]() {
    LOG(INFO) << "Exit";
    rpc::this_session().post_exit();
    exit = true;
  });

  server.async_run(2);

  // while (!exit) {
  //   // do nothing.
  //   LOG(INFO) << "exit: " << exit;
  //   std::this_thread::sleep_for(std::chrono::seconds(2));
  // }
  // return 0;

  for (int i = 0; i < 100; i++) {
    std::cout << "Registered image num: "
              << sfm_worker.GetRunningInfo().registered_image_num << "\n";
    reg_image_num += 2;

    if (reg_image_num == total_image_num) {
      completed = true;
      in_progress = false;
      sfm_worker.SetCompleted(completed);
    }
    // sfm_worker.SetInprogress(in_progress);
    sfm_worker.SetRegImageNum(reg_image_num);

    std::this_thread::sleep_for(std::chrono::seconds(12));
  }
}
#include "map_reduce/master.h"

#include <chrono>
#include <thread>

using namespace GraphSfM;

int main() {
  MapReduceConfig map_reduce_config;
  map_reduce_config.server_ips.push_back("localhost");
  map_reduce_config.server_ports.push_back(uint16_t(8080));

  // map_reduce_config.server_ips.push_back("162.105.86.72");
  // map_reduce_config.server_ports.push_back(uint16_t(8080));

  SfMMaster sfm_master;
  sfm_master.SetMapReduceConfig(map_reduce_config);

  rpc::client c("localhost", 8080);

  for (int i = 0; i < 10; i++) {
    c.call("HelloWorld");
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  c.call("Exit");

  // std::string master_image_path =
  // "/home/chenyu/Pictures/dataset/pku/anyuanmen"; std::string
  // worker_image_path = "/home/chenyu/Pictures/dataset/pku";
  // std::vector<std::string> image_names = {"DJI_0221.JPG", "DJI_0222.JPG"};
  // LOG(INFO) << "Run distributed mode";
  // sfm_master.RunDistributed();
  // sfm_master.CallSaveImages(0, master_image_path, worker_image_path,
  // image_names);
}
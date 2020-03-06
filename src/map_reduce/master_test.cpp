#include "map_reduce/master.h"

using namespace GraphSfM;

int main()
{
    MapReduceConfig map_reduce_config;
    map_reduce_config.server_ips.push_back("localhost");
    map_reduce_config.server_ports.push_back(uint16_t(8080));

    map_reduce_config.server_ips.push_back("162.105.86.72");
    map_reduce_config.server_ports.push_back(uint16_t(8080));

    SfMMaster sfm_master;
    sfm_master.SetMapReduceConfig(map_reduce_config);
    LOG(INFO) << "Run distributed mode";
    sfm_master.RunDistributed();
}
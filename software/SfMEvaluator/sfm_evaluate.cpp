#include <string>
#include <iostream>
#include <fstream>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "i23dSFM/sfm/sfm_data.hpp"
#include "i23dSFM/sfm/sfm_data_io.hpp"
#include "i23dSFM/sfm/pipelines/sfm_engine.hpp"

using namespace i23dSFM;
using namespace i23dSFM::sfm;
using namespace i23dSFM::geometry;
using namespace Eigen;
using namespace std;

int main(int argc, char* argv[])
{
    if(argc < 2) {
        cout << "please input the absolute path of sfm_data\n";
        return 0;
    }

    string sfm_data_filename = argv[1];

    // Load input SfM_Data scene
    SfM_Data sfm_data;
    if (!Load(sfm_data, sfm_data_filename, ESfM_Data(ALL))) {
        std::cerr << std::endl
        << "The input SfM_Data file \""<< sfm_data_filename << "\" cannot be read." << std::endl;
        return EXIT_FAILURE;
    }

    Poses poses = sfm_data.GetPoses();
    string dir = stlplus::folder_part(sfm_data_filename);\
    cout << dir << endl;
    ofstream poses_file(dir + "/poses.log");
    if(!poses_file.is_open()) {
        cout << "poses.log cannot be opened!\n";
        return 0;
    }
    int i = 0;
    cout << "size: " << poses.size() << endl;
    for(auto it  = poses.begin(); it != poses.end(); it++) {
        cout << i++ << endl;
        Pose3 pose = it->second;
        Matrix<double, 3, 3> rotation = pose.rotation();
        Vector3d translation = -rotation * pose.center();
        poses_file << it->first << " " << it->first << " 0\n";
        poses_file  << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << " " << translation(0, 0) << "\n"
                          << rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << " " << translation(1, 0) << "\n"
                          << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << " " << translation(2, 0) << "\n"
                          << "0.0000000" << " " << "0.0000000" << " " <<  "0.0000000" << " " << "1.0000000" << "\n";
    }
    poses_file.close();
}

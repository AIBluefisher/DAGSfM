// BSD 3-Clause License

// Copyright (c) 2018, 陈煜
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

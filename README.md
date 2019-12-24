# Graph Structure from Motion (GraphSfM)
[![Join the chat at https://gitter.im/hlzz/libvot](https://badges.gitter.im/hlzz/libvot.svg)](https://gitter.im/GraphSfM/Lobby)
![issues](https://img.shields.io/github/issues/AIBluefisher/GraphSfM.svg)
![forks](https://img.shields.io/github/forks/AIBluefisher/GraphSfM.svg)
![stars](https://img.shields.io/github/stars/AIBluefisher/GraphSfM.svg)
![license](https://img.shields.io/github/license/AIBluefisher/GraphSfM.svg)

[中文简介](./docs/README_ch.md)

**A similar version of GraphSfM based on [OpenMVG](https://github.com/openMVG/openMVG) has been released in: https://github.com/AIBluefisher/EGSfM.**

## 1. Overview of GraphSfM
Our Structure from Motion approach, named **`Graph Structure from Motion (GraphSfM)`**, is aimed at large scale 3D reconstruction. Besides, we aimed at exploring the computation ability of computer and making SfM easily transferred to distributed system. This work has been reconstructed and now it is based on [COLMAP](https://github.com/colmap/colmap).

In our work, 3D reconstruction is deemed as a ```divide-and-conquer``` problem. Our graph cluster algorithm divides images into different clusters, while images with high relativity remained in the same group. After the completion of local SfM in all clusters, an elaborate graph initialization and MST construction algorithm is designed to accurately merge clusters, and cope well with drift problems. The two proposed graph-based algorithms make SfM more efficient and robust - the graph cluster algorithm accelerate the SfM step while guarantee the robustness of clusters merging, and the MST construction makes point clouds alignment as accurate as possible. Our approach can reconstruct large scale data-set in one single machine with very high accuracy and efficiency.

If you use this project for your research, please cite:
```
@misc{chen2019graphbased,
    title={Graph-Based Parallel Large Scale Structure from Motion},
    author={Yu Chen and Shuhan Shen and Yisong Chen and Guoping Wang},
    year={2019},
    eprint={1912.10659},
    archivePrefix={arXiv},
    primaryClass={cs.CV}
}

@inproceedings{schoenberger2016sfm,
    author={Sch\"{o}nberger, Johannes Lutz and Frahm, Jan-Michael},
    title={Structure-from-Motion Revisited},
    booktitle={Conference on Computer Vision and Pattern Recognition (CVPR)},
    year={2016},
}
```

## 2. How to Build

### 2.1 Required

```
sudo apt-get install \
    git \
    cmake \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-regex-dev \
    libboost-system-dev \
    libboost-test-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev \
    libcgal-qt5-dev
```

Then install [ceres-solver]()
```sh
sudo apt-get install libatlas-base-dev libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout $(git describe --tags) # Checkout the latest release
mkdir build
cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make
sudo make install
```

Build our GraphSfM
```sh
git clone https://github.com/AIBluefisher/GraphSfM.git
cd GraphSfM
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

## 3. Usage

As our algorithm is not integrated in the `GUI` of `COLMAP`, we offer a script to run the 
distributed SfM (We hope there is anyone that is interested in integrating this pipeline into the GUI):
```sh
sudo chmod +x scripts/shell/distributed_sfm.sh
./distributed_sfm.sh $image_dir $num_images_ub $log_folder $completeness_ratio
```
- ```$image_dir```:   The directory that stores images
- ```$num_images_ub```: The maximum image number in each cluster. For example, ```80~120```.
- ```$log_folder```:  The directory that stores the logs
- ```$completeness_ratio```: The ratio that measure the repeatitive rate of adjacent clusters.

If succeed, camera poses and sparse points should be included in `$DATASET/sparse` folder, you can use COLMAP's GUI to 
import it and show the visual result:
```sh
./build/src/exe/colmap gui
```
For small scale reconstruction, you can set the `$num_images_ub` equal to the number of images, the program would just use the incremental SfM pipeline of [COLMAP](https://github.com/colmap/colmap).

For large scale reconstruction, our `GraphSfM` is highly recommended, these parameters should be tuned carefully: larger `$num_images_ub` and `$completeness_ratio` can make reconstruction more robust, but also may lead to low efficiency and even degenerate to incremental one.

## Licence

```
BSD 3-Clause License

Copyright (c) 2018, 陈煜
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

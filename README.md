# DAGSfM: Distributed and Graph-Based Structure-from-Motion Library

If you use this project for your research, please cite:
```
@article{article,
  author = {Chen, Yu and Shen, Shuhan and Chen, Yisong and Wang, Guoping},
  year = {2020},
  month = {07},
  pages = {107537},
  title = {Graph-Based Parallel Large Scale Structure from Motion},
  journal = {Pattern Recognition},
  doi = {10.1016/j.patcog.2020.107537}
}
```
```
@inproceedings{schoenberger2016sfm,
    author={Sch\"{o}nberger, Johannes Lutz and Frahm, Jan-Michael},
    title={Structure-from-Motion Revisited},
    booktitle={Conference on Computer Vision and Pattern Recognition (CVPR)},
    year={2016},
}
```

## 2. How to Build

### 2.1 Required

#### Basic Requirements
```sh
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

#### [ceres-solver]()

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

#### [igraph](https://igraph.org)
[igraph](https://github.com/igraph/igraph) is used for `Community Detection` and graph visualization.

```sh
sudo apt-get install build-essential libxml2-dev
wget https://igraph.org/nightly/get/c/igraph-0.7.1.tar.gz
tar -xvf igraph-0.7.1.tar.gz
cd igraph-0.7.1
./configure
make
make check
sudo make install
```

#### [rpclib](https://github.com/qchateau/rpclib)
[rpclib](https://github.com/qchateau/rpclib) is a light-weight Remote Procedure Call (RPC) library. Other RPC libs, such as GRPC, etc, are not chosen by this project for flexibility and convinience.

```sh
git clone https://github.com/qchateau/rpclib.git
cd rpclib
mkdir build && cd build
cmake ..
make -j8
sudo make install
```

### 2.2 Optional
#### Python Modules (Python 2.7 only)
This module is used for similarity seaching, while needs more evaluation.
```sh
sudo pip install scikit-learn tensorflow-gpu==1.7.0 scipy numpy progressbar2

# if the version of scikit-learn is not compatible, upgrade it by:
# pip install --upgrade scikit-learn
```

### 2.3 Build DAGSfM

```sh
git clone https://github.com/AIBluefisher/DAGSfM.git
cd DAGSfM
mkdir build && cd build
cmake .. && make -j8
```

## 3. Usage

As our algorithm is not integrated in the `GUI` of `COLMAP`, the scripts to run the 
distributed SfM are given (We hope there is anyone that is interested in integrating this pipeline into the GUI):

### Sequential Mode
```sh
sudo chmod +x scripts/shell/distributed_sfm.sh
./distributed_sfm.sh $image_dir $num_images_ub $log_folder $completeness_ratio
```
- ```$image_dir```:   The directory that stores images
- ```$num_images_ub```: The maximum image number in each cluster. For example, ```80~120```.
- ```$log_folder```:  The directory that stores the logs
- ```$completeness_ratio```: The ratio that measure the repeatitive rate of adjacent clusters.

### Distributed Mode
(1) At first, we need to establish the server for every worker: 
```sh
cd build/src/exe
./colmap local_sfm_worker --output_path=$output_path --port=$your_port
```
The RPC server establishes on local worker would be listening on the given port, and keep waitting until master assigns a job. We can also establish multiple workers on one machine, but to notice that ***port number should be unique!***

(2) Then, the ip and port for every server should be written in a `config.txt` file.
The file format should follow:
```txt
server_num
ip1 port1 image_path1
ip2 port2 image_path2
... ...
```
**note: image_path of each worker must be consistent with the `--output_path` option*.*

(3) At last, start our master
```sh
cd GraphSfM_PATH/scripts/shell
# The project folder must contain a folder "images" with all the images.
DATASET_PATH=/path/to/project
CONFIG_FILE_PATH=/path_to_config_file
num_images_ub=100
log_folder=/path_to_log_dir

./distributed_sfm sh $DATASET_PATH $num_images_ub $log_folder $CONFIG_FILE_PATH
```

The `distributed_sfm.sh` actually executes the following command in SfM module:
```sh
/home/chenyu/Projects/Disco/build/src/exe/colmap distributed_mapper \
$DATASET_PATH/$log_folder \
--database_path=$DATASET_PATH/database.db \
--image_path=$DATASET_PATH/images \
--output_path=$DATASET_PATH/$log_folder \
--config_file_name=$CONFIG_FILE_PATH/config.txt \
--num_workers=8 \
--distributed=1 \
--repartition=0 \
--num_images=100 \
--script_path=/home/chenyu/Projects/Disco/scripts/shell/similarity_search.sh \
--dataset_path=$DATASET_PATH \
--output_dir=$DATASET_PATH/$log_folder \
--mirror_path=/home/chenyu/Projects/Disco/lib/mirror \
--assign_cluster_id=0 \
--write_binary=1 \
--retriangulate=0 \
--final_ba=1 \
--select_tracks_for_bundle_adjustment=1 \
--long_track_length_threshold=10 \
--graph_dir=$DATASET_PATH/$log_folder \
--num_images_ub=$num_images_ub \
--completeness_ratio=0.7 \
--relax_ratio=1.3 \
--cluster_type=SPECTRA #SPECTRA #NCUT COMMUNITY_DETECTION #
# --max_num_cluster_pairs=$max_num_cluster_pairs \
# --image_overlap=$image_overlap \
```

Thus, you need to overwrite `/home/chenyu/Projects/Disco/build/src/exe/colmap`, `--script_path`, `--mirror_path` options by yours.

The parameters need to be reset for different purpose:
- `--transfer_images_to_server`: The option decides whether to transfer images that are stored on
master's disk to workers' disks. If we want to execute a further MVS process, we want this option to be set to `1`, because each worker that execute MVS needs to access the raw images.

- `--distributed`: This option decides the SfM module runs in distributed mode or sequential mode.
For example, if we just have one computer, we should set it to `0`, then SfM would run in sequential mode and allows you to reconstruct large scale images on a single computer. If we set it to `1`, we must ensure the `--config_file_name` option is valid, so that we we run SfM among
a lot of computers, which in a really distributed mode.

- `assign_cluster_id`: We use this option to indicate the program to assign each image with a
`cluster_id`, if we divide images into several clusters. This option allows us to render different
image poses that are clustered in different clusters by different colors.

- `write_binary`: This option indicates to save the SfM results in text format or in binary format.

- `final_ba`: This option indicates whether to perform a final bundle adjustment after merging all
local maps. As a very large scale map requires much time to optimize scene structures and camera poses, users should tune this option by their need.

- `select_tracks_for_bundle_adjustment`: As the final bundle adjustment requires too much time, we can select good tracks to optimize and achieves a comparable accuracy as full bundle adjustment.

- `long_track_length_threshold`: The maximum track length when selects good tracks for bundle adjustment.

- `num_images_ub`: The maximum number of images in each cluster.

- `completeness_ratio`: This option indicate the overlapping between clusters. 0.5~0.7 is enough 
in practice.

- `cluster_type`: This option decides which cluster method we choose for image clustering. We support `NCut` and `Spectral` Clustering. `Spectra` clustering is more accurate than `NCut` but it might be slower if we want to divide images into many clusters, as it needs much time to compute
eigen vectors.

If succeed, camera poses and sparse points should be included in `$DATASET/sparse` folder, you can use COLMAP's GUI to 
import it and show the visual result:
```sh
./build/src/exe/colmap gui
```
For small scale reconstruction, you can set the `$num_images_ub` equal to the number of images, the program would just use the incremental SfM pipeline of [COLMAP](https://github.com/colmap/colmap).

For large scale reconstruction, our `GraphSfM` is highly recommended, these parameters should be tuned carefully: larger `$num_images_ub` and `$completeness_ratio` can make reconstruction more robust, but also may lead to low efficiency and even degenerate to incremental one.

### Segment large scale maps
In some cases where we have a very large scale map, such that a latter Multi-View Stereo becomes
infeasible because of memory limitation. We can use the `point_cloud_segmenter` to segment original
map that is stored in colmap format into multiple small maps.
 ```sh
./build/src/exe/colmap point_cloud_segmenter \
--colmap_data_path=path_to_colmap_data \ 
--output_path=path_to_store_small_maps \
--max_image_num=max_number_image_for_small_map \
--write_binary=1
```

Before running this command, make sure `path_to_colmap_data` contains `images.txt`, `cameras.txt`, `points3D.txt` or `images.bin`, `cameras.bin`, `points3D.bin`.
- `max_image_num`: As colmap data includes images data, where store all registered images' data. We
limit the image number of each small map, and use this parameter to segment large maps. Though it's better to use the number of point clouds in practice, we haven't release the related implementation and we will enhance this helper further.

- 'write_binary`: set to `1`  if save colmap data in binary format, or set to  `0`  to save colmap data in text format.

## ChangeLog

- 2020.12.05
  - OpenMP for DAGSfM when running in sequential mode. This can be faster as one partition
    always cannot have an 100% CPU occupation (usually 40% ~ 60%). And the acceleration
    depends on the hardware and datasets scale.
  - Lazy initialization of RPC server, user should provide the port number in command line.
    Server would not listen on a port when running in sequential mode, so as to avoid crash
    when viewing models in GUI.

- 2020.06.24
  - Refactor distributed SfM and distributed matching in a consistent architecture.
  - Distributed matching.
  - Replaced vocabulary tree by deep learning model.
  - Use `Image Graph`, `Similarity Graph`, `View Graph` to handle different distributed tasks.

- 2020.04.11
  - Interface for extracting largest connected component in graph implementation.
  - Merge largest connected component in SfMAligner.
  - Command line helper for merging multiple local maps.
  - Command line helper for segmenting large scale map into several sub-maps.

- 2020.04.10
  - Select good tracks for final bundle adjustment.
  - Image transfer from master to workers.
  - Extract largest connected component for Structure-from-Motion.

- 2020.03.04
  - Using sparse matrix in Lagrange rotation averaging estimator. The efficiency for both row-by-row SDP solver and rank-deficient solver are highly improved. Rank-deficient solver is the fastest among current existing rotation averaging solver. (the solver is not public now.)

- 2020.01.15
  - Distributed implementation for Structure-from-Motion, which 
    only relies on `rpclib` for Remote Procedure Call(RPC). The distributed
    implementation follows the Map-Reduce architecture.

- 2020.01.10
  - Using Spectra for solving large scale eigenvalue problem in spectral 
    clustering. The efficiency is highly improved than original eigenvalue
    solver in Eigen.

- 2019.11.26
  - Normalized Cut for image clustering.
  - Spectral Clustering for image clustering.
  - Community detection for image clustering.
  - Graph-based sub-reconstruction Merging algorithm.
  - Fast view graph filtering algorithm.
  - Rotation averaging algorithms: Nonlinear RA, Lagrange Dual RA.
  - translation averaging algorithms: LUD (Not available currently).

# install gcc because of c++11 support
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt-get update -qq
if [ "$CXX" = "g++" ]; then sudo apt-get install -qq g++-4.8; fi
if [ "$CXX" = "g++" ]; then export CXX="g++-4.8" CC="gcc-4.8"; fi

# sudo add-apt-repository "deb http://ppa.launchpad.net/alexei.colin/opencv/ubuntu precise main" -y
sudo add-apt-repository -y ppa:philip5/extra
sudo apt-get install autoconf automake libtool unzip
sudo apt-get update -qq
# sudo apt-get -qq install libopencv-dev #libopencv-nonfree-dev

# OpenCV dependencies - Details available at: http://docs.opencv.org/trunk/doc/tutorials/introduction/linux_install/linux_install.html
sudo apt-get install -y build-essential
sudo apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libav-tools libavformat-dev libswscale-dev libavutil-dev 
sudo apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

# clone the latest opencv repo in github
git clone -b 2.4 https://github.com/opencv/opencv.git
cd opencv
# set build instructions for linux 
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_opencv_nonfree=ON -DWITH_TBB=ON -DWITH_V4L=ON -DWITH_QT=ON -DWITH_OPENGL=ON -DWITH_GTK=ON ..
make -j4
sudo make install

# return to the root directory
cd ../../

# install CUDA 7.5 and cuDNN v5
CUDA_REPO_PKG=cuda-repo-ubuntu1404_7.5-18_amd64.deb
wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/$CUDA_REPO_PKG
sudo dpkg -i $CUDA_REPO_PKG
rm $CUDA_REPO_PKG

ML_REPO_PKG=nvidia-machine-learning-repo-ubuntu1404_4.0-2_amd64.deb
wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1404/x86_64/$ML_REPO_PKG
sudo dpkg -i $ML_REPO_PKG

# update package lists
sudo apt-get -y update

# install packages
CUDA_PKG_VERSION="7-5"
CUDA_VERSION="7.5"
sudo apt-get install -y --no-install-recommends \
cuda-core-$CUDA_PKG_VERSION \
cuda-cudart-dev-$CUDA_PKG_VERSION \
cuda-cublas-dev-$CUDA_PKG_VERSION \
cuda-curand-dev-$CUDA_PKG_VERSION
# manually create CUDA symlink
sudo ln -s /usr/local/cuda-$CUDA_VERSION /usr/local/cuda

sudo apt-get install -y --no-install-recommends libcudnn5-dev

FROM nvidia/cuda:9.0-cudnn7-devel-ubuntu16.04

RUN pip install scikit-learn tensorflow-gpu==1.7.0 scipy numpy progressbar2

# if the version of scikit-learn is not compatible, upgrade it by:
# pip install --upgrade scikit-learn

# Prepare and empty machine for building
RUN apt-get update && apt-get install -y \
  git \
  cmake \
  vim \
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

# Build and install ceres solver
RUN apt-get -y install \
  libatlas-base-dev \
  libsuitesparse-dev
RUN git clone https://github.com/ceres-solver/ceres-solver.git --branch 1.14.0
RUN cd ceres-solver && \
  mkdir build && \
  cd build && \
  cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF && \
  make -j4 && \
  make install

# Build and install igraph
RUN apt-get install build-essential libxml2-dev
RUN wget https://igraph.org/nightly/get/c/igraph-0.7.1.tar.gz && \
  tar -xvf igraph-0.7.1.tar.gz && \
  cd igraph-0.7.1 && \
  ./configure && \
  make && \
  make check && \
  make install

# Build and install rpclib
RUN git clone https://github.com/AIBluefisher/rpclib.git && \
  cd rpclib && \
  mkdir build && cd build && \
  cmake .. && \
  make -j8 && \
  make install

# Build and install COLMAP

# Note: This Dockerfile has been tested using COLMAP pre-release 3.6-dev.3.
# Later versions of COLMAP (which will be automatically cloned as default) may
# have problems using the environment described thus far. If you encounter
# problems and want to install the tested release, then uncomment the branch
# specification in the line below
RUN git clone https://github.com/AIBluefisher/DAGSfM.git

RUN cd DAGSfM && \
  git checkout dev && \
  mkdir build && \
  cd build && \
  cmake .. && \
  make -j4 && \
  make install

docker pull DAGSfM/DAGSfM:latest
docker run --gpus all -w /working -v $1:/working -it DAGSfM/DAGSfM:latest;

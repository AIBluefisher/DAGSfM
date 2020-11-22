docker build -t="DAGSfM:latest" .;
docker run --gpus all -w /working -v $1:/working -it DAGSfM:latest;

DATASET_PATH=$1

/home/amax/Projects/colmap/build/src/exe/colmap exhaustive_matcher \
--database_path=$DATASET_PATH/database.db \
--SiftMatching.num_threads=8 \
--SiftMatching.use_gpu=0 \
--SiftMatching.gpu_index=-1
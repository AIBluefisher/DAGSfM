DATASET_PATH=$1

/home/amax/Projects/colmap/build/src/exe/colmap feature_extractor \
--database_path=$DATASET_PATH/database.db \
--image_path=$DATASET_PATH/images \
--SiftExtraction.num_threads=8 \
--SiftExtraction.use_gpu=0 \
--SiftExtraction.gpu_index=-1
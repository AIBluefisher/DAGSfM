DATASET_PATH=$1
VOC_TREE_PATH=$2

/home/amax/Projects/colmap/build/src/exe/colmap vocab_tree_matcher \
--database_path=$DATASET_PATH/database.db \
--SiftMatching.num_threads=8 \
--SiftMatching.use_gpu=0 \
--SiftMatching.gpu_index=-1 \
--VocabTreeMatching.num_images=100 \
--VocabTreeMatching.num_nearest_neighbors=5 \
--VocabTreeMatching.vocab_tree_path=$VOC_TREE_PATH
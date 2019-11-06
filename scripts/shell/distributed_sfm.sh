DATASET_PATH=$1
num_images_ub=$2
log_folder=$3
# image_overlap=$3
# max_num_cluster_pairs=$4


/home/amax/Projects/colmap/build/src/exe/colmap feature_extractor \
--database_path=$DATASET_PATH/database.db \
--image_path=$DATASET_PATH/images \
--SiftExtraction.num_threads=8 \
--SiftExtraction.use_gpu=0 \
--SiftExtraction.gpu_index=-1

/home/amax/Projects/colmap/build/src/exe/colmap exhaustive_matcher \
--database_path=$DATASET_PATH/database.db \
--SiftMatching.num_threads=8 \
--SiftMatching.use_gpu=0 \
--SiftMatching.gpu_index=-1

/home/amax/Projects/colmap/build/src/exe/colmap distributed_mapper \
$DATASET_PATH/$log_folder \
--database_path=$DATASET_PATH/database.db \
--image_path=$DATASET_PATH/images \
--output_path=$DATASET_PATH/sparse \
--num_workers=8 \
--graph_dir=$DATASET_PATH/$log_folder \
--num_images_ub=$num_images_ub \
--completeness_ratio=0.5 \
--relax_ratio=1.3 \
--cluster_type=NCUT
# --max_num_cluster_pairs=$max_num_cluster_pairs \
# --image_overlap=$image_overlap \

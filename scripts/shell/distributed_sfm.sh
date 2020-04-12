DATASET_PATH=$1
num_images_ub=$2
log_folder=$3
# completeness_ratio=$4
#VOC_TREE_PATH=$5
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
## Or use vocabulary tree matcher
# /home/amax/Projects/colmap/build/src/exe/colmap vocab_tree_matcher \
# --database_path=$DATASET_PATH/database.db \
# --SiftMatching.num_threads=8 \
# --SiftMatching.use_gpu=1 \
# --SiftMatching.gpu_index=0 \
# --VocabTreeMatching.num_images=100 \
# --VocabTreeMatching.num_nearest_neighbors=5 \
# --VocabTreeMatching.vocab_tree_path=$VOC_TREE_PATH

/home/amax/Projects/colmap/build/src/exe/colmap distributed_mapper \
$DATASET_PATH/$log_folder \
--database_path=$DATASET_PATH/database.db \
--transfer_images_to_server=0 \
--image_path=$DATASET_PATH/images \
--output_path=$DATASET_PATH/$log_folder \
--num_workers=8 \
--distributed=0 \
--repartition=0 \
--assign_cluster_id=1 \
--write_binary=1 \
--retriangulate=0 \
--final_ba=0 \
--select_tracks_for_bundle_adjustment=1 \
--long_track_length_threshold=10 \
--graph_dir=$DATASET_PATH/$log_folder \
--num_images_ub=$num_images_ub \
--completeness_ratio=0.7 \
--relax_ratio=1.3 \
--cluster_type=NCUT # SPECTRA
# --image_overlap=$image_overlap \

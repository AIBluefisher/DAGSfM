#= GL3D =============================================================================================================================
net_type=resnet-50 #googlenet
pooling=MAX
step=720000 #400000 
ckpt_save_dir=./data/model/model.ckpt 
img_size=896
feature_dim=256
#pca_file='-1' 
pca_file=./data/model/pca_mat_$step'_'$feature_dim'd.npy'
gl3d_root_dir=/path/to/your/gl3d/root/dir
test_image_list=./data/gl3d/eval_img_list.txt
test_feature_list=./data/gl3d/eval_feature_list.txt
rmac_step=1,3,5
output_dir=./output
#python retrieval/inference.py --img_list $test_image_list --ckpt_step $step --img_size $img_size \
#    --net $net_type --pool $pooling --rmac_step $rmac_step --ckpt_path $ckpt_save_dir --gl3d_root $gl3d_root_dir
#python retrieval/deep_query.py --feature_list $test_feature_list --out_dir $output_dir --top 200 --out_dim $feature_dim --dataset_root $gl3d_root_dir
#python retrieval/compute_overlap_benchmark.py $test_image_list $output_dir/match_pairs data/gl3d/gl3d_gt_pairs
#=======================================================================================================================================


#= oxford or paris =======================================================================================================================
dataset=oxford  #paris 
oxford_image_list=/home/tianwei/Data/oxford5k/image_list #/path/to/your/oxford/image/list
ground_truth_folder=./data/$dataset/groundtruth
query_output_path=$dataset'_queries'
db_output_path=$dataset'_output'
oxford_feature_list=$db_output_path/feature_list
oxford_query_feat_list=$query_output_path/query_list
img_size=896
rmac_step=1,2
output_dir=./output

# extract query features
rm -rf $query_output_path
python retrieval/inference.py --img_list $oxford_image_list --oxford_gt $ground_truth_folder --net $net_type --pool $pooling \
    --ckpt_step $step --ckpt_path $ckpt_save_dir --img_size $img_size --rmac_step $rmac_step --multi_scale 
ls -d $PWD/$query_output_path/*.npy > $query_output_path/query_list

# extract db features
rm -rf $db_output_path
python retrieval/inference.py --output_folder $db_output_path --img_list $oxford_image_list --net $net_type --pool $pooling \
    --ckpt_step $step --ckpt_path $ckpt_save_dir --img_size $img_size --rmac_step $rmac_step --multi_scale 
ls -d $PWD/$db_output_path/*.npy > $db_output_path/feature_list

# query
python retrieval/deep_query.py --out_dir $output_dir --feature_list $oxford_feature_list \
    --query_list oxford_queries/query_list --top 6500 --out_dim $feature_dim --qe 10 --et 2 #--pca_thresh 0.90 #--rmac --pca_file $pca_file 

# evaluate
python retrieval/oxford_bench.py data/$dataset/query_list.txt data/$dataset/image_list.txt $output_dir/match_pairs $ground_truth_folder \
    --output_dir $output_dir

# remove temporary feature files
rm -rf $query_output_path
rm -rf $db_output_path
#=======================================================================================================================================


#= holiday =============================================================================================================================
dataset=holiday
# You are responsible for producing the following required input lists
holiday_image_list=/path/to/holiday/image/list
query_image_list=/path/to/query/image/list
query_feature_list=/path/to/query/feature/list
db_feature_list=/path/to/all/feature/list
feature_dim=512
# Then run the commands
#python retrieval/inference.py --img_list $holiday_image_list --net $net_type --pool $pooling --rmac_step $rmac_step \
#    --ckpt_step $step --ckpt_path $ckpt_save_dir --img_size $img_size --multi_scale
#python retrieval/deep_query.py --feature_list $db_feature_list --query_list $query_feature_list --out_dir $output_dir --top 10 --out_dim $feature_dim  #--qe 10 --et 1 #--rmac 
#mv $output_dir/match_pairs $output_dir/match_pairs_$dataset
#cd data/holidays
#python convert2holiday_eval.py ../../$output_dir/match_pairs_$dataset $query_image_list $holiday_image_list ../../$output_dir/holidays_output.dat
#python holidays_map.py ../../$output_dir/holidays_output.dat
#cd -  > /tmp/holiday_null

#!/bin/bash

#./do.sh img_path recon_path use_gms
#i=1
#for i in $@
#do
img_path=$1
sfm_path=$1/$2
mvs_path=$sfm_path/mvs
max_img_num=$3
gms=$4
record=$sfm_path/timerecord.txt
start_sfm=$(date "+%Y-%m-%d %H:%M:%S")

echo "before sfm time:",$start_sfm >> $record
# python /home/chenyu/projects/i23dSfM/build/software/SfM/SfM_SequentialPipeline.py $img_path $sfm_path $3 $4 $5
python /home/chenyu/projects/i23dSfM/build/software/SfM/SfM_SequentialPipeline.py $img_path $sfm_path $max_img_num $gms
end_sfm=$(date "+%Y-%m-%d %H:%M:%S")

#spend_sfm=$end_sfm-$start_sfm


echo "finished time:",$end_sfm >> $record

#done


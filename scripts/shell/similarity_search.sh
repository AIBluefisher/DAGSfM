#  BSD 3-Clause License

#  Copyright (c) 2020, Chenyu
#  All rights reserved.

#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:

#  1. Redistributions of source code must retain the above copyright notice,
#  this
#     list of conditions and the following disclaimer.

#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.

#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

DATASET_PATH=$1
OUTPUT_PATH=$2
MOST_SIMILAR_IMAGES_NUM=$3
MIRROR_PATH=$4

# MIRROR_PATH=/home/chenyu/Projects/Disco/lib/mirror
MODEL_PATH=$MIRROR_PATH/data/model/model.ckpt

ls -d $DATASET_PATH/images/* > $DATASET_PATH/image_list.txt

# 1. generate the .npy features.
python $MIRROR_PATH/retrieval/inference.py \
--img_list $DATASET_PATH/image_list.txt \
--ckpt_step 720000 \
--img_size 896 \
--net resnet-50 \
--pool MAX \
--rmac_step 1,3,5 \
--ckpt_path $MODEL_PATH

# 2. output all the absolute paths of feature files (*.npy) to a file and query
mkdir $DATASET_PATH/dl_features
mv $DATASET_PATH/images/*.npy $DATASET_PATH/dl_features
ls -d $DATASET_PATH/dl_features/*.npy > $DATASET_PATH/feature_list.txt

# 3. Deep query
python $MIRROR_PATH/retrieval/deep_query.py \
--feature_list $DATASET_PATH/feature_list.txt \
--out_dir $OUTPUT_PATH \
--top $MOST_SIMILAR_IMAGES_NUM \
--out_dim 256

# 4. Remove redundency files.
# rm $DATASET_PATH/image_list.txt
rm $DATASET_PATH/feature_list.txt
rm -rf $DATASET_PATH/dl_features
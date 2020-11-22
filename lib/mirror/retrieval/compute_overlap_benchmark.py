#!/usr/bin/env python

# Copyright 2017, Tianwei Shen, HKUST.
# compute mAP score for overlap benchmark

import sys, os
sys.path.append('..')
from tools.common import read_list

def compute_overlap_ap(gt_list, res_list, d):
    gt_size = len(gt_list)
    old_recall = 0.0
    old_precision = 1.0
    ap = 0.0
    intersect_size = 0
    for i in xrange(len(res_list)):
        if res_list[i] in gt_list:
            intersect_size = intersect_size + 1
        recall = float(intersect_size) / gt_size
        precision = float(intersect_size) / (i + 1.0)
        ap += (recall - old_recall) * (old_precision + precision) / 2.0
        old_precision = precision
        old_recall = recall
    return ap


def compute_GL3D_map(image_list, overlap_result, ground_truth_file):
    image_filenames = read_list(image_list)
    overlap_pair_lines = read_list(overlap_result)
    gt_lines = read_list(ground_truth_file)

    gt_vec = [[] for i in image_filenames]
    for line in gt_lines:
        split_items = line.split()
        index1 = int(split_items[0])
        index2 = int(split_items[1])
        if index1 == index2:            # don't count (i, i) pair
            continue
        gt_vec[index1].append(index2)

    res_vec = [[] for i in image_filenames]
    for line in overlap_pair_lines:
        split_items = line.split()
        index1 = int(split_items[0])
        index2 = int(split_items[1])
        if index1 == index2:            # don't count (i, i) pair
            continue
        res_vec[int(index1)].append(int(index2))

    num_test = 0
    mAP = 0
    for i in xrange(len(image_filenames)):
        if len(gt_vec[i]) != 0:
            num_test = num_test + 1
            ap = compute_overlap_ap(gt_vec[i], res_vec[i], i)
            #print(i, image_filenames[i], ap, i)
            mAP = mAP + ap
    mAP = mAP / num_test
    print(mAP, num_test)
    return mAP


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print(sys.argv[0], '<test_image_list> <overlap_result> <ground_truth_pairs>')
        exit()

    image_list = sys.argv[1]
    overlap_result = sys.argv[2]
    ground_truth_file = sys.argv[3]
    compute_GL3D_map(image_list, overlap_result, ground_truth_file)
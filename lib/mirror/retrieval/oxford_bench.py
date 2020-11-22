#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 - Present, Tianwei Shen, HKUST.

"""
query images and get rank lists.
evaluate oxford5k (paris6k) benchmark datasets.
"""

from __future__ import print_function

import os
import sys
import time
from tempfile import NamedTemporaryFile

import numpy as np

_file_path = os.path.abspath(sys.argv[0])
_cur_dir = os.path.dirname(_file_path)
sys.path.append(os.path.dirname(_cur_dir))
from tools.common import read_list, write_list, Notify


def get_ap(inds, query_name, index_names, groundtruth_dir, ranked_dir=None, disp_each=True):
    """
    Given a query, index data, and path to groundtruth data, perform the query,
    and evaluate average precision for the results by calling to the compute_ap
    script. Optionally save ranked results in a file.

    :param ndarray inds:
        the indices of index vectors in ascending order of distance
    :param str query_name:
        the name of the query
    :param list index_names:
        the name of index items
    :param str groundtruth_dir:
        directory of groundtruth files
    :param str ranked_dir:
        optional path to a directory to save ranked list for query

    :returns float:
        the average precision for this query
    """

    if ranked_dir is not None:
        # Create dir for ranked results if needed
        if not os.path.exists(ranked_dir):
            os.makedirs(ranked_dir)
        rank_file = os.path.join(ranked_dir, '%s.txt' % query_name)
        f = open(rank_file, 'w')
    else:
        f = NamedTemporaryFile(delete=False)
        rank_file = f.name

    f.writelines([index_names[i] + '\n' for i in inds])
    f.close()

    groundtruth_prefix = os.path.join(groundtruth_dir, query_name)
    oxford_benchmark_prog = os.path.join('cpp', 'compute_ap')
    cmd = ' %s %s' % (groundtruth_prefix, rank_file)
    cmd = oxford_benchmark_prog + cmd
    ap = os.popen(cmd).read()

    # Delete temp file
    if ranked_dir is None:
        os.remove(rank_file)

    if disp_each:
        print(Notify.UNDERLINE, query_name, float(ap.strip()), Notify.ENDC)
    return float(ap.strip())


def oxford5k_bench(query_list, image_list, match_pair, gt_dir, output_dir):
    query_names = []
    with open(query_list) as f:
        lines = f.readlines()
        for line in lines:
            query_names.append(line.strip())

    image_names = []
    with open(image_list) as f:
        lines = f.readlines()
        for line in lines:
            image_names.append(line.strip())

    rank_lists = [[] for _ in range(len(query_names))]
    with open(match_pair) as f:
        lines = f.readlines()
        for line in lines:
            items = line.split()
            ind1 = int(items[0])
            ind2 = int(items[1])
            rank_lists[ind1].append(ind2)

    aps = []
    for i in range(len(query_names)):
        aps.append(get_ap(rank_lists[i], query_names[i], image_names, gt_dir, output_dir))

    return np.array(aps).mean()


if __name__ == '__main__':
    from argparse import ArgumentParser
    PARSER = ArgumentParser()
    PARSER.add_argument('query_list', type=str, default=None,
                        help='path to the query list')
    PARSER.add_argument('image_list', type=str, default=None,
                        help='path to the db list')
    PARSER.add_argument('match_pair', type=str, default=None,
                        help='match_pair')
    PARSER.add_argument('gt_dir', type=str,
                        default=None, help='directory containing groundtruth files')
    PARSER.add_argument('--output_dir', dest='output_dir', type=str,
                        default='output', help='directory containing output files')
    ARGS = PARSER.parse_args()
    mAP = oxford5k_bench(ARGS.query_list, ARGS.image_list, ARGS.match_pair, ARGS.gt_dir, ARGS.output_dir)
    print('mAP:', mAP)
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 - Present, Tianwei Shen, HKUST.
# Copyright 2017 - Present, Zixin Luo, HKUST.

"""
query images and get rank lists.
evaluate oxford5k (paris6k) benchmark datasets.
"""

from __future__ import print_function

import os
import sys
import time
from random import randint
from functools import partial
from tempfile import NamedTemporaryFile
from threading import Thread
from Queue import Queue

import tensorflow as tf
from sklearn.preprocessing import normalize as sknormalize
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans
import numpy as np
import progressbar
from sklearn.externals import joblib

_file_path = os.path.abspath(sys.argv[0])
_cur_dir = os.path.dirname(_file_path)
sys.path.append(os.path.dirname(_cur_dir))
from tools.common import read_list, write_list, Notify


def normalize(x, copy=False):
    """
    A helper function that wraps the function of the same name in sklearn.
    This helper handles the case of a single column vector.
    """
    if type(x) == np.ndarray and len(x.shape) == 1:
        return np.nan_to_num(np.squeeze(sknormalize(x.reshape(1, -1), copy=copy)), copy=copy)
    else:
        return np.nan_to_num(sknormalize(x, copy=copy), copy=copy)


def simple_query_expansion(query_feat, db_feat, result, num_regions, top_k=10):
    """
    Get the top-k closest (regional) vectors, average and re-query
    """
    qe_feat = []
    num_query = query_feat.shape[0] / num_regions
    for i in range(num_query):
        inds = result[i][0][0:top_k]
        dists = result[i][1][0:top_k]
        single_query = query_feat[i * num_regions:(i + 1) * num_regions]
        for j in range(len(inds)):
            weight = dists[j]/num_regions
            single_query = single_query + weight * db_feat[inds[j] * num_regions:(inds[j] + 1) * num_regions]
        qe_feat.append(single_query)

    qe_feat = np.vstack(qe_feat)
    qe_feat = normalize(qe_feat)
    return qe_feat


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


def _prc_match_graph(query_features, db_features, num_regions, top_k, euclidean_dist, aml):
    """TensorFlow graph for PR-MAC distance calculation."""
    num_regions = tf.constant(num_regions, dtype=tf.int32)
    num_query = tf.shape(query_features)[0]/num_regions
    num_db = tf.shape(db_features)[0]/num_regions
    feat_dim = tf.shape(db_features)[1]

    if euclidean_dist:
        norm_db = tf.reshape(tf.reduce_sum(db_features * db_features, axis=1), [-1, 1])

    def body(single_query):
        """Loop body."""

        if euclidean_dist:
            # compute L2 norm
            norm_query = tf.reshape(tf.reduce_sum(single_query * single_query, axis=1), [-1, 1])
            # distance matrix (Euclidean)
            dist_mat = norm_query - 2 * \
                tf.matmul(single_query, db_features, transpose_b=True) + tf.transpose(norm_db)
            dist_seg = tf.reduce_min(tf.reshape(dist_mat, [-1, num_regions]), axis=1)
        else:
            dist_mat = tf.matmul(single_query, db_features, transpose_b=True)
            dist_seg = tf.reduce_max(tf.reshape(dist_mat, [-1, num_regions]), axis=1)
        max_rv_mat = tf.reshape(dist_seg, [num_regions, num_db])
        if aml:
            final_dist = tf.reduce_max(max_rv_mat, axis=0)
        else:
            final_dist = tf.reduce_sum(max_rv_mat, axis=0)
        return final_dist

    reshape_query_features = tf.reshape(query_features, (num_query, num_regions, feat_dim))
    res_dist = tf.map_fn(lambda x: body(x), reshape_query_features, back_prop=False)

    top_k = tf.squeeze(tf.cast(top_k, tf.int32))
    if euclidean_dist:
        sorted_dist, sorted_indx = tf.nn.top_k(-res_dist, top_k)
        sorted_dist = -sorted_dist
    else:
        sorted_dist, sorted_indx = tf.nn.top_k(res_dist, top_k, name='dist')
        sorted_indx = tf.cast(sorted_indx, tf.float32, name='rank')
    return [sorted_dist, sorted_indx]


def match_gpu(query_features, db_features, num_regions, top, euclidean_dist=False, aml=False):
    """GPU supported PR-MAC."""
    tf.reset_default_graph()
    assert query_features.shape[1] == db_features.shape[1]
    query_features = tf.constant(query_features, dtype=tf.float32, name='query_features')
    db_features = tf.constant(db_features, dtype=tf.float32, name='db_features')

    # a weird bug with tensorflow 1.3, otherwise it is of type 'Dimension'
    num_query = int(int(query_features.shape[0]) / num_regions)
    num_db = int(int(db_features.shape[0]) / num_regions)
    top_k = top
    if num_db < top:
        top_k = num_db

    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    with tf.Session(config=config) as sess:
        tf_output = _prc_match_graph(query_features, db_features, num_regions, top_k, euclidean_dist, aml)
        sess.run(tf.global_variables_initializer())
        dist, indx = sess.run(tf_output)

    query_result = []
    for i in range(num_query):
        query_result.append([indx[i].astype(np.int32), dist[i]])
    return query_result


def read_feature(feature_list, euclidean_dist=False, rmac=False, mac=False, num_rv=None):
    """Read features stored in .npy."""
    names = []
    num_regions = 1
    prog_bar = progressbar.ProgressBar()
    prog_bar.maxval = len(feature_list)
    feature = None

    def _worker(q, feature, num_regions):
        while True:
            data = q.get()
            feat = data[0]
            idx = data[1]

            if feat is None or idx is None:
                return

            if False:
                # Not ready yet. KMeans or GMM to reduce regional vector number.
                if idx % 100 == 0:
                    print(idx)

                kmeans = KMeans(n_clusters=5, random_state=0).fit(feat)
                trans_feat = kmeans.cluster_centers_
                trans_feat = np.zeros([5, 512])
                for i in range(feat.shape[0]):
                    trans_feat[kmeans.labels_[i]] += feat[i]
            else:
                feature[idx * num_regions:(idx + 1) * num_regions] = feat
            q.task_done()

    q = Queue()
    worker_thread = None
    prog_bar.start()

    for idx, val in enumerate(feature_list):
        feat = np.load(val)
        if num_rv is not None:
            if num_rv > 0:
                feat = feat[:, -num_rv:]
            else:
                feat = feat[:, (0, 5, 10)]
        feat = np.squeeze(feat)
        feat = feat.transpose()
        if rmac:
            feat = np.sum(feat, axis=0)
        if mac:
            feat = feat[0,:]
        # initialization
        if feature is None:
            if len(feat.shape) == 2:
                num_regions = feat.shape[0]
                dim = feat.shape[1]
            elif len(feat.shape) == 1:
                dim = feat.shape[0]
            else:
                print(Notify.FAIL, 'Invalid feature', val, Notify.ENDC)
                exit(-1)
            feature = np.zeros([len(feature_list) * num_regions, dim])
            worker_thread = Thread(target=_worker, args=(q, feature, num_regions))
            worker_thread.daemon = True
            worker_thread.start()

        def graipher(pts, dim, K):
            def calc_distances(p0, points):
                return ((p0 - points)**2).sum(axis=1)
            farthest_pts = np.zeros((K, dim))
            farthest_pts[0] = pts[0]
            distances = calc_distances(farthest_pts[0], pts)
            for i in range(1, K):
                idx = np.argmax(distances)
                farthest_pts[i] = pts[idx]
                distances = np.minimum(distances, calc_distances(farthest_pts[i], pts))
            return farthest_pts

        # sampled_feat = graipher(feat, dim, 5)
        q.put([feat, idx])
        names.append(os.path.splitext(os.path.basename(val))[0])
        prog_bar.update(idx)

    # poison pill
    q.put([None, None])
    worker_thread.join()

    if not euclidean_dist:
        feature = normalize(feature)
    return num_regions, feature, names


def whitening(feat, num_regions, pca_thresh=None, out_dim=None, pca=None):
    """PCA whitening."""
    if pca is not None:
        # use pre-computed PCA
        trans_feat = pca.transform(feat)
    else:
        # compute PCA here
        if out_dim is None:
            # use adaptive PCA
            print(Notify.INFO, 'Adaptive PCA with threshold', pca_thresh, Notify.ENDC)
            pca = PCA(n_components=pca_thresh, whiten=True, copy=True)
            pca.fit(feat)
            trans_feat = pca.transform(feat)
        else:
            # reduce to specified dimention
            print(Notify.INFO, 'Reduce dimensionality to', out_dim, Notify.ENDC)
            pca = PCA(n_components=out_dim, whiten=True, copy=True, random_state=0)
            pca.fit(feat)
            trans_feat = pca.transform(feat)
    trans_feat = normalize(trans_feat)
    return trans_feat, pca


def mask_prediction(rv0, rv1, path0, path1):
    def _get_coord_and_scale(indx):
        if indx == 0:
            return (0, 0), 28
        elif indx > 0 and indx < 10:
            row = (indx - 1) / 3 * 7
            col = (indx - 1) % 3 * 7
            return (row, col), 14
        elif indx >= 10 and indx < 35:
            row = (indx - 10) / 5 * 5
            col = (indx - 10) % 5 * 5
            return (row, col), 8

    def _get_mask_map(rv0, rv1, rv_num):
        mask_map = np.zeros((28, 28), np.uint8)
        r_inds = np.zeros(rv_num, np.int32)
        r_dist = np.zeros(rv_num)
        for i in range(rv_num):
            tmp = np.multiply(rv0[i], rv1).sum(axis=1)
            r_inds[i] = np.argmax(tmp)
            if tmp[r_inds[i]] > 0.90:
                if i == 0:
                    r_dist[i] = 1
                elif i > 0 and i < 10:
                    r_dist[i] = 2
                elif i >= 10 and i < 35:
                    r_dist[i] = 3

        if r_dist[0] == 1:
            print(Notify.INFO, 'Globally similar', Notify.ENDC)

        for i in range(rv_num):
            if r_dist[i] > 0:
                coord, scale = _get_coord_and_scale(r_inds[i])
                mask_map[coord[0]: coord[0] + scale,
                         coord[1]: coord[1] + scale] = r_dist[i] * 85
        return np.uint8(mask_map)

    rv_num = rv0.shape[0]
    mask_map0 = _get_mask_map(rv1, rv0, rv_num)
    mask_map1 = _get_mask_map(rv0, rv1, rv_num)

    # Warn(Do not move): to show mask, add
    #
    # import matplotlib.pyplot as plt
    # from scipy import misc
    #
    # and un-comment the following lines:

    #img0 = misc.imread(path0)
    #mask_map0 = misc.imresize(mask_map0, (896, 896))
    #img0[:, :, 0] = mask_map0
    #plt.subplot(121)
    #plt.imshow(img0)
    #img1 = misc.imread(path1)
    #mask_map1 = misc.imresize(mask_map1, (896, 896))
    #img1[:, :, 0] = mask_map1
    #plt.subplot(122)
    #plt.imshow(img1)
    #plt.show()


def query(query_list, feature_list, out_dir, top=200, 
          pca_thresh=0.9, out_dim=None, pca_file='-1', qe_fn=None,
          mask_pred=False, euclidean_dist=False, rmac=False, mac=False, aml=False):

    """Query by list."""
    print(Notify.INFO, 'Read feature', Notify.ENDC)
    print(Notify.INFO, 'Use R-MAC: ', rmac, Notify.ENDC)
    num_regions, db_feat, image_names = read_feature(feature_list, euclidean_dist, rmac, mac)

    # below codes are for predicted mask visualization.
    itv = 10
    while mask_pred:
        idx0 = randint(itv, len(feature_list) - 1 - itv)
        idx1 = randint(idx0 - itv, idx0 + itv)
        print(Notify.INFO, 'Pair idx', (idx0, idx1), Notify.ENDC)
        # FIXME: adapt the image ext.
        image_path0 = feature_list[idx0].replace('npy', 'JPG')
        image_path1 = feature_list[idx1].replace('npy', 'JPG')
        # some images end with '.jpg'
        if not os.path.exists(image_path0):
            image_path0 = feature_list[idx0].replace('npy', 'jpg')
        if not os.path.exists(image_path1):
            image_path1 = feature_list[idx1].replace('npy', 'jpg')
        rv0 = db_feat[idx0 * num_regions:(idx0 + 1) * num_regions]
        rv1 = db_feat[idx1 * num_regions:(idx1 + 1) * num_regions]
        mask_prediction(rv0, rv1, image_path0, image_path1)

    print(Notify.INFO, '# Feature', len(feature_list), Notify.ENDC)
    print(Notify.INFO, '# Dim', db_feat.shape[-1], Notify.ENDC)
    print(Notify.INFO, '# Reginal vector', num_regions, Notify.ENDC)
    # perform PCA whitening.
    use_pca = (pca_thresh is not None or out_dim is not None or pca_file != '-1') and len(image_names) > out_dim

    if pca_file != '-1':
        pca_data = np.load(pca_file).item()
        pca = PCA(whiten=True, copy=True, random_state=0)
        pca.mean_ = pca_data['mean']
        pca.components_ = pca_data['eigvec']
        pca.explained_variance_ = pca_data['eigval']
    else:
        pca = None

    if use_pca:
        db_trans_feat, pca = whitening(db_feat, num_regions, pca_thresh, out_dim, pca=pca)
        print(Notify.INFO, 'PCA-ed feature dim', db_trans_feat.shape[1], Notify.ENDC)
    else:
        print(Notify.WARNING, 'No whitening', Notify.ENDC)
        db_trans_feat = db_feat

    if query_list is not None:
        query_num_regions, query_feat, query_names = read_feature(query_list, euclidean_dist, rmac, mac)
        assert(num_regions == query_num_regions)
        if use_pca:
            query_trans_feat, _ = whitening(query_feat, num_regions, pca=pca)
        else:
            query_trans_feat = query_feat
        query_num = len(query_list)
    else:
        query_trans_feat = db_trans_feat
        query_num = len(feature_list)

    # output path name
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)
    match_index_file = os.path.join(out_dir, 'match_pairs')

    print(Notify.INFO, 'Compute nn distance', Notify.ENDC)
    start = time.time()
    query_result = match_gpu(query_trans_feat, db_trans_feat, num_regions,
                             top, euclidean_dist=euclidean_dist, aml=aml)
    end = time.time()
    print(Notify.INFO, 'Time cost in matching', end - start, 's', Notify.ENDC)

    if qe_fn is not None:
        for _ in range(ARGS.et):
            print(Notify.INFO, 'Expand queries and re-match', Notify.ENDC)
            qe_feature = qe_fn(query_trans_feat, db_trans_feat, query_result, num_regions)
            query_result = match_gpu(qe_feature, db_trans_feat, num_regions, top, aml=aml)

    content = []
    aps = []
    for i in range(query_num):
        inds = query_result[i][0]
        dists = query_result[i][1]
        content.extend([' '.join([str(i), str(inds[j]), str(dists[j]/num_regions)]) for j in range(len(inds))])
    write_list(content, match_index_file)
    return 0


if __name__ == '__main__':
    from argparse import ArgumentParser
    PARSER = ArgumentParser()
    PARSER.add_argument('--dataset_root', dest='dataset_root', type=str, default=None,
                        help='dataset root dir')
    PARSER.add_argument('--feature_list', dest='feature_list', type=str, required=True,
                        help='path to the feature file containing all .npy files')
    PARSER.add_argument('--query_list', dest='query_list', type=str, default=None,
                        help='path to the query list containing all .npy files')
    PARSER.add_argument('--out_dir', dest='out_dir', type=str,
                        default='./', help='optional path to save ranked output')
    PARSER.add_argument('--top', dest='top', type=int,
                        default=200, help='top match output')
    PARSER.add_argument('--pca_thresh', dest='pca_thresh',
                        type=float, default=None, help='threshold for pca')
    PARSER.add_argument('--out_dim', dest='out_dim', type=int,
                        default=None, help='dimension of final feature, control whether and how to do pca')
    PARSER.add_argument('--pca_file', dest='pca_file', type=str,
                        default='-1', help='path to pre-computed pca file')
    PARSER.add_argument('--qe', dest='qe', type=int,
                        default=None, help='perform query expansion with this many top results')
    PARSER.add_argument('--et', dest='et', type=int,
                        default=0, help='query expansion times')
    PARSER.add_argument('--mask_pred', dest='mask_pred', default=False,
                        action='store_true', help='whether to predict and visualize mask')
    PARSER.add_argument('--euclidean_dist', dest='euclidean_dist', default=False,
                        action='store_true', help='test each data set separately')
    PARSER.add_argument('--rmac', dest='rmac', default=False,
                        action='store_true', help='use r-mac instead of pr-mac when --rmac is set')
    PARSER.add_argument('--mac', dest='mac', default=False,
                        action='store_true', help='use mac --mac is set')
    PARSER.add_argument('--aml', dest='aml', default=False,
                        action='store_true', help='use aml --aml is set')
    ARGS = PARSER.parse_args()
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
    QE_FN = partial(simple_query_expansion, top_k=ARGS.qe) if ARGS.qe > 0 else None
    # read feature list
    FEATURE_LIST = read_list(ARGS.feature_list)
    if ARGS.dataset_root != None:
        FEATURE_LIST = [os.path.join(ARGS.dataset_root, p) for p in FEATURE_LIST]
    # read query_list if it's different from feature_list
    QUERY_LIST = None
    if ARGS.query_list is not None:
        QUERY_LIST = read_list(ARGS.query_list)
        if ARGS.dataset_root != None:
            QUERY_LIST = [os.path.join(ARGS.dataset_root, p) for p in QUERY_LIST]
    if not os.path.exists(ARGS.out_dir):
        os.mkdir(ARGS.out_dir)

    # compute aggregated features and run the evaluation
    mAP = query(QUERY_LIST, FEATURE_LIST, ARGS.out_dir, ARGS.top,
                ARGS.pca_thresh, ARGS.out_dim, ARGS.pca_file, QE_FN,
                ARGS.mask_pred, ARGS.euclidean_dist, ARGS.rmac, ARGS.mac, ARGS.aml)
#!/usr/bin/env python
"""
Copyright 2017, Tianwei Shen, HKUST.
Convert results to INRIA Holidays dataset evaluation format
"""

import os
import sys

if __name__ == '__main__':
    if len(sys.argv) < 5:
        print('Usage:', sys.argv[0], '<match_pairs> <query_image_list> <db_image_list> <output_file>')
        exit(-1)
    match_pairs_path = sys.argv[1]
    query_image_list_path = sys.argv[2]
    db_image_list_path = sys.argv[3]

    # read query image names
    query_image_list = []
    with open(query_image_list_path) as f:
        lines = f.readlines()
        for line in lines:
            query_image_list.append(os.path.split(line.strip())[1])
    
    # read db image names
    db_image_list = []
    with open(db_image_list_path) as f:
        lines = f.readlines()
        for line in lines:
            db_image_list.append(os.path.split(line.strip())[1])

    # read match pairs result
    query_results = [[] for _ in range(len(query_image_list))]
    with open(match_pairs_path) as f:
        lines = f.readlines()
        for line in lines:
            fir, sec, score = line.split()
            query_results[int(fir)].append(int(sec))
    
    output_filename = sys.argv[4]
    fout = open(output_filename, 'w')
    for i in range(len(query_results)):
        out_line = query_image_list[i]
        for j in range(len(query_results[i])):
            target_name = query_results[i][j]
            out_line = out_line + ' ' + str(j) + ' ' + db_image_list[target_name]
        fout.write(out_line+'\n')
    fout.close()
    print('output to ', output_filename)

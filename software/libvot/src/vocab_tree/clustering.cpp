/*
Copyright (c) 2015, Tianwei Shen
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of libvot nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*! \file clustering.cpp
 * \brief clustering algorithms, such as k-means, implementations
 */
#include <iostream>
#include <limits>
#include <cassert>
#include <vector>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <thread>
#include <algorithm>

#include "clustering.h"

namespace vot
{
/** Random sample k indexes for [1...n] without replacement */
void GetIntialCenters(size_t n, int k, size_t *initial_idx)
{
	for (int i = 0; i < k; i++) {
		size_t tmp = rand() % n;
		int j;
		for (j = 0; j < i; j++) {
			if (tmp == initial_idx[j]) {
				i--; j = -1;
				break;
			}
		}
		if (j != -1)
			initial_idx[i] = tmp;
	}
}

inline void CopyDes2Double(double *dst, DTYPE *src, int length)
{
	for (int i = 0; i < length; i++)
		dst[i] = (double) src[i];
}

inline double DisDes2Double(double *dst, DTYPE *src, int length)
{
	double dis = 0.0;
	for (int i = 0; i < length; i++) {
		double d = dst[i] - (double)src[i];
		dis += d * d;
	}
	dis = sqrt(dis);
	return dis;
}

void MultiComputeAssignment(size_t num, int dim, int k, DTYPE **des, double *means, int *assignment, double *error_out, size_t *changed_num)
{
	double error = 0.0;
	for (size_t i = 0; i < num; i++) {
		double min_dis = std::numeric_limits<double>::max();
		int min_idx = -1;
		for (int j = 0; j < k; j++) {
			double dis_tmp = DisDes2Double(means + j * dim, des[i], dim);

			if (min_dis > dis_tmp) {
				min_dis = dis_tmp;
				min_idx = j;
			}
		}
		error += min_dis;
		if (assignment[i] != min_idx) {
			assignment[i] = min_idx;
			(*changed_num)++;
		}
	}
	*error_out = error;
}

/**
	 * @brief Compute new assignment for each feature points
	 * @param num - the total number of feature points
	 * @param dim - the dimension of the feature points
	 * @param k - the number of clusters
	 * @param p - the address to the feature points
	 * @param means - the descriptors of the clusters
	 * @param assignment - the input and output assignments
	 * @param error - total error of the new assignment
	 * @param thread_num - thread number
	 * @return
	 */
size_t ComputeAssignment(size_t num, int dim, int k, DTYPE **p, double *means, int *assignment, double &error_out, int thread_num)
{
	size_t change_num = 0;
	double error = 0.0;
	if (thread_num == 1 || num < thread_num * 100) {		// single-thread
		for (size_t i = 0; i < num; i++) {
			double min_dis = std::numeric_limits<double>::max();
			int min_idx = -1;
			for (int j = 0; j < k; j++) {
				double dis_tmp = DisDes2Double(means + j * dim, p[i], dim);

				if (min_dis > dis_tmp) {
					min_dis = dis_tmp;
					min_idx = j;
				}
			}
			error += min_dis;
			if (assignment[i] != min_idx) {
				assignment[i] = min_idx;
				change_num++;
			}
		}
	}
	else {		// multi-thread
		std::vector<std::thread> threads;
		size_t num_points = num /thread_num;
		size_t count = 0;
		size_t *changes= new size_t [thread_num];
		double *error_vec = new double [thread_num];
		for (int i = 0; i < thread_num; i++) {
			changes[i] = 0;
			error_vec[i] = 0.0;
		}

		for (int i = 0; i < thread_num; i++) {
			size_t total_num = 0;
			if (i == thread_num - 1)
				total_num = num - (thread_num - 1) * num_points;
			else
				total_num = num_points;
			threads.push_back(std::thread(MultiComputeAssignment, total_num, dim, k, p+count, means, assignment+count, &error_vec[i], &changes[i]));
			count += num_points;
		}
		std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));
		for (int i = 0; i < thread_num; i++) {
			change_num += changes[i];
			error += error_vec[i];
		}
	}
	error_out = error;
	//std::cout << change_num << " " << error << std::endl;

	return change_num;
}

/**
	 * @brief Accumulate the feature descriptors to the total, a subroutine used by multi-threaded version of ComputeMeans
	 * @param num - the total number of feature points
	 * @param start_id - the start index for this thread
	 * @param dim - the dimension of the feature points
	 * @param k - the number of clusters
	 * @param p - the address to the starting feature point
	 * @param assignment_curr - the current assignments computed by ComputeAssignment
	 * @param total - accumulator
	 * @param counts - counter
	 * @return
	 */
void MultiDesAccumulation(size_t num, int dim, int k, DTYPE **p, int *assignment_curr, double *totals, size_t *counts)
{
	for (size_t i = 0; i < num; i++) {
		int label = assignment_curr[i];
		counts[label]++;
		for (int j = 0; j < dim; j++)
			totals[label * dim + j] += (double)p[i][j];
	}
}

/**
	 * @brief Compute new means for each cluster
	 * @param num - the total number of feature points
	 * @param dim - the dimension of the feature points
	 * @param k - the number of clusters
	 * @param assignment_curr - the current assignments computed by ComputeAssignment
	 * @param means_curr - the means of clusters
	 * @param thread_num - the number of threads
	 * @return
	 */
void ComputeMeans(size_t num, int dim, int k, DTYPE **p, int *assignment_curr, double *means_curr, int thread_num)
{
	// initialization
	size_t *counts = new size_t [k];
	for (int i = 0; i < k; i++) {
		counts[i] = 0;
		for (int j = 0; j < dim; j++)
			means_curr[i * dim + j] = 0;
	}

	// accumulation
	if(thread_num == 1) {
		for (size_t i = 0; i < num; i++) {
			int label = assignment_curr[i];
			counts[label]++;
			for (int j = 0; j < dim; j++)
				means_curr[label * dim + j] += (double)p[i][j];
		}
	}
	else {
		std::vector<double *>sub_totals(thread_num);
		std::vector<size_t *>sub_counts(thread_num);
		for (int i = 0; i < thread_num; i++) {
			sub_totals[i] = new double [dim * k];
			sub_counts[i] = new size_t [k];
			for (int j = 0; j < dim * k; j++)
				sub_totals[i][j] = 0.0;
			for (int j = 0; j < k; j++)
				sub_counts[i][j] = 0;
		}
		std::vector<std::thread> threads;
		size_t off = 0;
		for (int i = 0; i < thread_num; i++) {
			size_t feature_num = num / thread_num;
			if (i == thread_num - 1)
				feature_num = num - (thread_num - 1) * feature_num;
			threads.push_back(std::thread(MultiDesAccumulation, feature_num, dim, k, p+off, assignment_curr+off, sub_totals[i], sub_counts[i]));
			off += feature_num;
		}
		std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));
		for (int i = 0; i < thread_num; i++) {
			for (int j = 0; j < k * dim; j++)
				means_curr[j] += sub_totals[i][j];
			for (int j = 0; j < k; j++)
				counts[j] += sub_counts[i][j];
		}
		for (int i = 0; i < thread_num; i++) {
			delete [] sub_counts[i];
			delete [] sub_totals[i];
		}
	}

	// normalization
	for(int i = 0; i < k; i++) {
		if (counts[i] == 0) continue;
		for (int j = 0; j < dim; j++) {
			means_curr[i * dim + j] /= counts[i];
		}
	}

	delete [] counts;
}

double ComputeError(size_t num, int dim, int k, DTYPE **p, double *means, int *assignment)
{
	double error = 0;
	for (int i = 0; i < num; i++) {
		int c = assignment[i];
		for (int j = 0; j < dim; j++) {
			double d = means[c*dim+j] - p[i][j];
			error += d * d;
		}
	}

	return error;
}

double Kmeans(size_t num, int dim, int k, DTYPE **p, double *means, int *assignment, int thread_num)
{
	if (num < k) {
		std::cerr << "[Error] Number of keys is less than the number of clusters, Exit...\n";
		return -1;
	}
	double min_dis = std::numeric_limits<double>::max();
	double *means_curr;
	size_t *initial_idx;
	int *assignment_curr;
	double change_pct_threshold = 0.05;
	int total_iter = 1;

	means_curr = new double [k * dim];
	initial_idx = new size_t [k];
	assignment_curr = new int [num];

	if (means_curr == NULL || initial_idx == NULL || assignment_curr == NULL) {
		std::cerr << "[Error] Memory allocation fails in kmeans\n";
		return -1;
	}

	while (total_iter--) {
		double dis = 0;
		GetIntialCenters(num, k, initial_idx);
		for (int i = 0; i < k; i++) {
			CopyDes2Double(means_curr + i * dim, p[initial_idx[i]], dim);
		}

		// initial assignment
		for (size_t i = 0; i < num; i++)
			assignment_curr[i] = -1;

		size_t change_num = ComputeAssignment(num, dim, k, p, means_curr, assignment_curr, dis, thread_num);
		double change_pct = (double) change_num / num;
		assert(change_num == num);

		while (change_pct > change_pct_threshold) {
			// recompute means
			ComputeMeans(num, dim, k, p, assignment_curr, means_curr, thread_num);

			// recompute assignments
			change_num = ComputeAssignment(num, dim, k, p, means_curr, assignment_curr, dis, thread_num);
			change_pct = (double) change_num / num;
		}

		ComputeMeans(num, dim, k, p, assignment_curr, means_curr, thread_num);
		if (dis < min_dis) {
			min_dis = dis;
			memcpy(means, means_curr, sizeof(double) * k * dim);
			memcpy(assignment, assignment_curr, sizeof(int) * num);
		}
	}

	delete [] means_curr;
	delete [] initial_idx;
	delete [] assignment_curr;

	return ComputeError(num, dim, k, p, means, assignment);
}
}	// end of namespace vot

/*
Copyright (c) 2015 - 2016, Tianwei Shen
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

/** \file svt.cpp
 *	\brief singular value threshold (immature)
 */
#include <iostream>
#include <vector>
#include <sstream>
#include <Eigen/Dense>
#include <random>

#include "utils/io_utils.h"

using namespace std;
using namespace Eigen;

float Fnorm(Eigen::MatrixXf &f)
{
	float norm = 0;
	for (int i = 0; i < f.rows(); i++) {
		for (int j = 0; j < f.cols(); j++) {
			norm += f(i, j) * f(i, j);
		}
	}
	return sqrt(norm);
}

double Fnorm(Eigen::MatrixXd &d)
{
	double norm = 0;
	for (int i = 0; i < d.rows(); i++) {
		for (int j = 0; j < d.cols(); j++) {
			norm += d(i, j) * d(i, j);
		}
	}
	return sqrt(norm);
}

// sample_set.size() = d.rows();
Eigen::MatrixXd MatrixProjection(Eigen::MatrixXd &d, std::vector<std::vector<int> > sample_set)
{
	Eigen::MatrixXd pd = Eigen::MatrixXd::Constant(d.rows(), d.cols(), 0);
	for (int i = 0; i < d.rows(); i++) {
		for (int j = 0; j < sample_set[i].size(); j++) {
			pd(i, sample_set[i][j]) = d(i, sample_set[i][j]);
		}
	}
	return pd;
}

int main(int argc, char ** argv)
{
	if (argc != 2) {
		cout << "Usage: " << argv[0] << " <mat_file>\n";
		exit(-1);
	}
	const char *mat_file = argv[1];

	// here we assume that the input matrix is a square matrix
	vector<string> mat_string;
	tw::IO::ExtractLines(mat_file, mat_string);
	int mat_size = mat_string.size();
	MatrixXd input_mat = MatrixXd::Constant(mat_size, mat_size, 0);
	for (int i = 0; i < mat_size; i++) {
		stringstream ss;
		ss << mat_string[i];
		float temp;
		for (int j = 0; j < mat_size; j++) {
			ss >> temp;
			input_mat(i, j) = temp;
		}
	}

	MatrixXd sample_mat = MatrixXd::Constant(mat_size, mat_size, 0);
	const float sample_ratio = 0.4;
	const int sample_size = mat_size * sample_ratio;
	vector<vector<int> > sample_set;
	sample_set.resize(mat_size);

	// sample without replacement
	default_random_engine e(0);
	uniform_int_distribution<int> uni_rand(0, mat_size-1);
	for (int i = 0; i < mat_size; i++) {
		for (int j = 0; j < sample_size; j++) {
			int curr_sample = uni_rand(e);
			int k;
			for (k = 0; k < j; k++) {
				if (curr_sample == sample_set[i][k]) {
					j--; k = -1;
					break;
				}
			}
			if (k != -1) {
				sample_set[i].push_back(curr_sample);
			}
		}
	}

	for (int i = 0; i < mat_size; i++) {
		for (int j = 0; j < sample_set[i].size(); j++) {
			sample_mat(i, sample_set[i][j]) = input_mat(i, sample_set[i][j]);
		}
	}

	cout << sample_mat << endl;

	// singular value thresholding
	float step_size = 1.5;//1.2 * mat_size / sample_ratio;
	float tau = 5 * mat_size;
	int k0 = tau / (step_size * Fnorm(sample_mat)) + 1;
	int max_iter = 200;
	MatrixXd Y = k0 * step_size * sample_mat;
	MatrixXd X;
	double stop_threshold = 1e-4;
	for (int i = 0; i < max_iter; i++) {
		JacobiSVD<MatrixXd> svd(Y, ComputeThinU | ComputeThinV);
		MatrixXd U = svd.matrixU();
		MatrixXd V = svd.matrixV();
		VectorXd singular_vector = svd.singularValues();
		MatrixXd S = MatrixXd::Constant(mat_size, mat_size, 0);
		for (int i = 0; i < mat_size; i++) {
			if (singular_vector[i] > tau) {
				S(i, i) = singular_vector[i] - tau;
			}
		}
		X = U * S * V.transpose();

		// compute error
		MatrixXd residual_mat = X - sample_mat;
		MatrixXd residual_proj = MatrixProjection(residual_mat, sample_set);
		double error = Fnorm(residual_proj);
		double sample_mat_norm = Fnorm(sample_mat);
		if (error / sample_mat_norm < stop_threshold) {
			cout << "break at iter " << i << endl;
			break;
		}

		// refresh Y
		for (int i = 0; i < sample_mat.rows(); i++) {
			for (int j = 0; j < sample_set[i].size(); j++) {
				Y(i, sample_set[i][j]) = Y(i, sample_set[i][j]) + step_size * (sample_mat(i, sample_set[i][j]) - X(i, sample_set[i][j]));
			}
		}
	}

	// compute completion error
	float completion_error = 0;
	float max_error = 0;
	float error_bound = 10;
	int within_bound_count = 0;
	for (int i = 0; i < mat_size; i++) {
		for (int j = 0; j < mat_size; j++) {
			completion_error += (X(i, j) - input_mat(i, j)) * (X(i, j) - input_mat(i, j));
			if (max_error < abs(X(i, j) - input_mat(i, j)) ) {
				max_error = abs(X(i, j) - input_mat(i, j));
			}
			if (abs(X(i, j) - input_mat(i, j)) < error_bound) {
				within_bound_count++;
			}
		}
	}
	MatrixXd error_mat = X - input_mat;
	float relative_error = Fnorm(error_mat) / Fnorm(input_mat);
	cout << "completion_error " << completion_error << endl;
	cout << "max_error " << max_error << endl;
	cout << "within_bound_count " << within_bound_count << endl;
	cout << "relative_error " << relative_error << endl;

	return 0;
}

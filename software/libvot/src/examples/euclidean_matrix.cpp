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

/** \file euclidean_matrix.cpp
 *	\brief euclidean matrix completion (exe)
 */
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <stdio.h>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace std;

/**
 * @brief An ad-hoc simple 2d point struct
 */
namespace {
struct point2d
{
	float x,y;
};
}	// end of namespace vot

float EuclideanDistance(point2d x1, point2d x2)
{
	return (x1.x - x2.x) * (x1.x - x2.x) + (x1.y - x2.y) * (x1.y - x2.y);
}

int main(int argc, char **argv)
{
	int input_size;
	cout << "the size of the matrix: \n";
	cin >> input_size;
	const int MATRIX_SIZE = input_size;
	string matrix_filename = "euclidean_matrix";
	stringstream ss;
	ss << matrix_filename << "_" << MATRIX_SIZE;
	ss >> matrix_filename;
	FILE *matrix_file = fopen(matrix_filename.c_str(), "w");

	std::vector<point2d> points(MATRIX_SIZE);
	for (int i = 0; i < MATRIX_SIZE; i++) {
		points[i].x = rand() % 1000;
		points[i].x /= 1000;
		points[i].y = rand() % 1000;
		points[i].y /= 1000;
	}

	Eigen::MatrixXf distance_matrix(MATRIX_SIZE, MATRIX_SIZE);
	for (int i = 0; i < MATRIX_SIZE; i++) {
		distance_matrix(i, i) = 0.0;
		for (int j = i+1; j < MATRIX_SIZE; j++) {
			distance_matrix(i, j) = EuclideanDistance(points[i], points[j]);
			distance_matrix(j, i) = distance_matrix(i, j);
		}
	}

	// output to the file
	for (int i = 0; i < MATRIX_SIZE; i++) {
		for (int j = 0; j < MATRIX_SIZE; j++) {
			cout << distance_matrix(i, j) << " ";
			fprintf(matrix_file, "%f ", distance_matrix(i, j));
		}
		cout << endl;
		fprintf(matrix_file, "\n");
	}

	Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(distance_matrix);
	//lu_decomp.setThreshold(1e-5);
	cout << "rank of distance matrix: " << lu_decomp.rank() << endl;

	//Eigen::JacobiSVD<Eigen::MatrixXf> svd(distance_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
	//cout << svd.singularValues() << endl;

	fclose(matrix_file);
	return 0;
}

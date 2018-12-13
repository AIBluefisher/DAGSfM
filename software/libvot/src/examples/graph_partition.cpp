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

/** \file graph_partition.cpp
 *	\brief graph partition example (exe)
 */

#include <iostream>
#include <vector>
#include <fstream>
#include "graph/image_graph.h"
#include "utils/io_utils.h"

using namespace std;

int main(int argc, char **argv)
{
	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " <adj_lists_file>\n";
		exit(1);
	}

	const char* adj_lists_file = argv[1];

	/**
	 *  Case #1: make a undirected graph data
	 */
	vot::ImageGraph ig(6);
	ig.addEdge(0, 1, 1.0);
	ig.addEdge(0, 2, 1.0);
	ig.addEdge(1, 2, 1.0);
	ig.addEdge(2, 3, 1.0);
	ig.addEdge(3, 4, 1.0);
	ig.addEdge(3, 5, 1.0);
	ig.addEdge(4, 5, 1.0);

	std::vector<std::vector<int> > cuts;
	// Karger's algorithm requires the graph to be a connected component
	if (ig.numConnectedComponents()) {
		if (!ig.kargerCut(cuts)) return -1;
		cout << cuts[0].size() << " " << cuts[1].size() << endl;
		cout << "part1\n";
		for (int i = 0; i < cuts[0].size(); i++)
			cout << cuts[0][i] << endl;
		cout << "part2\n";
		for (int i = 0; i < cuts[1].size(); i++)
			cout << cuts[1][i] << endl;
	}

	/**
	 *  Case #2: read graph data from a file
	 */
	ifstream fin;
	fin.open(adj_lists_file);
	int image_num, src, dst;
	double score;
	fin >> image_num;
	vot::ImageGraph large_graph(image_num);
	while (fin >> src >> dst >> score) {
		large_graph.addEdge(src, dst, score);
	}

	if (large_graph.numConnectedComponents()) {
		large_graph.kargerCut(cuts);
		cout << cuts[0].size() << " " << cuts[1].size() << endl;
		cout << "part1\n";
		for(int i = 0; i < cuts[0].size(); i++)
			cout << cuts[0][i] << endl;
		cout << "part2\n";
		for(int i = 0; i < cuts[1].size(); i++)
			cout << cuts[1][i] << endl;
	}

	return 0;
}

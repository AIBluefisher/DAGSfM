// This is a modified version of the retrieval benchmark program provided at
// http://www.robots.ox.ac.uk/~vgg/data/oxbuildings/
// It is modified to explicitly include cstdlib. All other functions except
// main are moved to oxford5k.h

#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <cstdlib>
#include "oxford5k.h"

using namespace std;

int main(int argc, char **argv)
{
	if (argc != 3) {
		cout << "Usage: ./compute_ap [GROUNDTRUTH QUERY] [RANKED LIST]\n";
		return -1;
	}

	string gtq = argv[1];

	vector<string> ranked_list = load_list(argv[2]);
	set<string> good_set = vector_to_set(load_list(gtq + "_good.txt"));
	set<string> ok_set = vector_to_set(load_list(gtq + "_ok.txt"));
	set<string> junk_set = vector_to_set(load_list(gtq + "_junk.txt"));

	set<string> pos_set;
	pos_set.insert(good_set.begin(), good_set.end());
	pos_set.insert(ok_set.begin(), ok_set.end());

	float ap = compute_ap(pos_set, junk_set, ranked_list);

	cout << ap << "\n";

	return 0;
}

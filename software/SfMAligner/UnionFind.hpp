/*
Copyright (c) 2018, Yu Chen
All rights reserved.
*/

#ifndef UNION_FIND_HPP
#define UNION_FIND_HPP

#include <vector>

using namespace std;

namespace i23dSFM {
class UnionFind
{
private:
    vector<int> _parent;
    vector<int> _rank;
    int _n;

public:
    UnionFind(int n);

    int FindRoot(int x);

    void UnionSet(int x, int y);
};
}

#endif
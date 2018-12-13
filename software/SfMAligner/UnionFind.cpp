/*
Copyright (c) 2018, Yu Chen
All rights reserved.
*/

#include "UnionFind.hpp"

namespace i23dSFM {
UnionFind::UnionFind(int n)
{
    _n = n;
    for (int i = 0; i < n; i++) {
        _parent.push_back(i);
        _rank.push_back(0);
    }
}

int UnionFind::FindRoot(int x)
{
    return (_parent[x] == x ? x : _parent[x] = FindRoot(_parent[x]));
}

void UnionFind::UnionSet(int x, int y)
{
    x = FindRoot(x);
    y = FindRoot(y);

    if (x == y) return;

    if (_rank[x] < _rank[y]) {
        _parent[x] = y;
    }

    else {
        _parent[y] = x;
        if (_rank[x] == _rank[y]) _rank[x]++;
    }
}
}
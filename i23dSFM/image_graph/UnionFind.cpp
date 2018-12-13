/*
Copyright (c) 2018, Yu Chen
All rights reserved.

*/

/** \file UnionFind.cpp
 *	\brief data structures 
 */
#include <cstring>

#include "UnionFind.hpp"

namespace i23dSFM
{
    UnionFind::UnionFind(size_t n)
    {
        father = new size_t[n];
        size = n;
        set_num = n;
        for(size_t i = 0; i < size; i++)
            father[i] = i;
    }

    UnionFind::~UnionFind()
    {
        delete [] father;
    }

    size_t UnionFind::Find(size_t x)
    {
        if(x != father[x])
            father[x] = Find(father[x]);	// path compression
        return father[x];
    }

    bool UnionFind::UnionSet(size_t x, size_t y)
    {
        x = Find(x);
        y = Find(y);
        if(x == y) return false;	// already in the same set
        father[y] = x;		        // append y to x sets
        set_num--;
        return true;
    }

}	// end of namespace i23dSFM
#include "graph/union_find.h"

namespace GraphSfM {
namespace graph {

UnionFind::UnionFind(size_t n)
{
    this->Init(n);
}

void UnionFind::Init(size_t n)
{
    for (size_t i = 0; i < n; i++) {
        parents_.push_back(i);
        ranks_.push_back(0);
    }
}

void UnionFind::InitWithNodes(const std::vector<size_t>& nodes)
{
    nodes_.assign(nodes.begin(), nodes.end());
    for (uint i = 0; i < nodes.size(); i++) {
        nodes_mapper_[nodes[i]] = i;
    }
}

size_t UnionFind::FindRoot(size_t x)
{
    size_t idx = nodes_mapper_[x];
    return (parents_[idx] == idx) ? idx : 
            (parents_[idx] = FindRoot(nodes_[parents_[idx]]));
}

void UnionFind::Union(size_t x, size_t y)
{
    x = FindRoot(x);
    y = FindRoot(y);
    if (x == y) return;
    
    if (ranks_[x] < ranks_[y]) parents_[x] = y;
    else {
        parents_[y] = x;
        if (ranks_[x] == ranks_[y]) ranks_[x]++;
    }
}

std::vector<size_t> UnionFind::GetRanks() const
{
    return ranks_;
}

std::vector<size_t> UnionFind::GetParents() const
{
    return parents_;
}

} // namespace graph
} // namespace GraphSfM
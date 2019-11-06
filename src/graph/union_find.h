#ifndef GRAPH_UNION_FIND_H
#define GRAPH_UNION_FIND_H

#include <vector>
#include <iostream>
#include <unordered_map>

namespace GraphSfM {
namespace graph {

class UnionFind 
{
private:
    std::vector<size_t> ranks_;
    std::vector<size_t> parents_;
    std::vector<size_t> nodes_;
    std::unordered_map<size_t, size_t> nodes_mapper_;

public:
    // constructor
    UnionFind() {}
    UnionFind(size_t n);

    // union find operations
    void Init(size_t n);
    void InitWithNodes(const std::vector<size_t>& nodes);
    size_t FindRoot(size_t x);
    void Union(size_t x, size_t y);

    // get functions
    std::vector<size_t> GetRanks() const;
    std::vector<size_t> GetParents() const;
};

} // namespace graph
} // namespace GraphSfM

#endif
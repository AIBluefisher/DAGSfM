#ifndef GRAPH_BASE_HPP
#define GRAPH_BASE_HPP

#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace i23dSFM {
namespace match_graph {
class LinkEdgeBase
{
private:
    size_t _src;
    size_t _dst;

public:
    LinkEdgeBase(size_t i, size_t j)
    {
        _src = i;
        _dst = j;
    }

    LinkEdgeBase(LinkEdgeBase& edge)
    {
        _src = edge.src();
        _dst = edge.dst();
    }

    size_t src() const { return _src; }

    size_t dst() const { return _dst; }
};

class WeightedLinkEdge : public LinkEdgeBase
{
private:
    size_t _weight;

public:
    WeightedLinkEdge(size_t i, size_t j, size_t w) : LinkEdgeBase(i, j), _weight(w) {}

    size_t weight() const { return _weight; }

};

class GraphBase
{
private:
    std::unordered_map<size_t, size_t> node;
    
public:
    bool AddNode(size_t i);
    bool AddEdge(size_t i, size_t j);
    bool AddEdge(size_t i, size_t j, size_t w);
    bool ConstructMST();
    bool NormalizedCut();
};
}   // namespace match_graph
}   // namespace i23dSFM

#endif
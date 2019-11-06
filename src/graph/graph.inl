#include <fstream>

#include "graph/graph.h"
#include "graph/union_find.h"
#include "graph/graph_cut.h"

#include <glog/logging.h>


// #define __DEBUG__

namespace GraphSfM {
namespace graph {

template <typename T1, typename T2>
Graph<T1, T2>::Graph() { _size = 0; }

template <typename T1, typename T2>
Graph<T1, T2>::Graph(size_t n)
{
    _size = n;
    for (size_t i = 0; i < n; i++) _degrees[i] = 0;
}

template <typename T1, typename T2>
Graph<T1, T2>::Graph(const Graph<T1, T2>& graph)
{
    std::unordered_map<size_t, T1> nodes = graph.GetNodes();
    std::unordered_map<size_t, EdgeMap> edges = graph.GetEdges();

    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        this->AddNode(it->second);
    }
    for (auto it = edges.begin(); it != edges.end(); ++it) {
        EdgeMap em = it->second;
        for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
            this->AddEdge(em_it->second);
        }
    }
}

template <typename T1, typename T2>
Graph<T1, T2> Graph<T1, T2>::Clone() const
{
    Graph<T1, T2> graph(this->_size);

    for (auto it = _nodes.begin(); it != _nodes.end(); ++it) {
        graph.AddNode(it->second);
    }
    for (auto it = _edges.begin(); it != _edges.end(); ++it) {
        EdgeMap em = it->second;
        for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
            graph.AddEdge(em_it->second);
        }
    }
    return graph;
}

template <typename T1, typename T2>
T1 Graph<T1, T2>::GetNode(size_t idx) const
{
    if (!HasNode(idx)) return T1();
    return _nodes.at(idx);
}

template <typename T1, typename T2>
std::unordered_map<size_t, T1> Graph<T1, T2>::GetNodes() const
{
    return _nodes;
}

template <typename T1, typename T2>
size_t Graph<T1, T2>::GetNodesNum() const
{
    return _nodes.size();
}

template <typename T1, typename T2>
bool Graph<T1, T2>::HasNode(const size_t& idx) const
{
    if (_nodes.find(idx) == _nodes.end()) return false;
    return true;
}

template <typename T1, typename T2>
bool Graph<T1, T2>::AddNode(const T1& node)
{
    if (HasNode(node.id)) return false;

    _size++;
    // if (node.idx == -1) node.idx = _size;
    _nodes[node.id] = node;
    return true;
}

template <typename T1, typename T2>
bool Graph<T1, T2>::DeleteNode(const size_t& idx)
{
    if (!HasNode(idx)) return false;
    _size--;
    _nodes.erase(idx);
    return true;
}

template <typename T1, typename T2>
bool Graph<T1, T2>::RemoveSingletonNodes()
{
    // this->CountDegrees();
    for (auto it = _degrees.begin(); it != _degrees.end(); ++it) {
        if (it->second == 0) {
            _nodes.erase(it->first);
            _degrees.erase(it->first);
        }
    }
    return true;
}

template <typename T1, typename T2>
std::vector<T1> Graph<T1, T2>::FindSingletonNodes()
{
    std::vector<T1> singleton_nodes;
    this->CountOutDegrees();
    this->CountInDegrees();
    this->CountDegrees();
    for (auto it = _degrees.begin(); it != _degrees.end(); ++it) {
        if (it->second == 0) {
            singleton_nodes.push_back(this->GetNode(it->first));
        }
    }
    return singleton_nodes;
}

template <typename T1, typename T2>
std::unordered_map<size_t, std::unordered_map<size_t, T2>> Graph<T1, T2>::GetEdges() const
{
    return _edges;
}

template <typename T1, typename T2>
T2 Graph<T1, T2>::GetEdge(size_t src, size_t dst) const
{
    if (!HasEdge(src, dst)) return T2();
    return _edges.at(src).at(dst);
}

template <typename T1, typename T2>
size_t Graph<T1, T2>::GetEdgesNum() const
{
    size_t sum = 0;
    for (auto it = _edges.begin(); it != _edges.end(); ++it) {
        EdgeMap em = it->second;
        sum += em.size();
    }
    // std::vector<std::unordered_map<size_t, T2>> edges = this->SerializeEdges();
    // for (auto em : edges) { sum += em.size(); }
    return sum;
}

template <typename T1, typename T2>
bool Graph<T1, T2>::HasEdge(const size_t& src, const size_t& dst) const
{
    const auto em_ite = _edges.find(src);
    if (em_ite == _edges.end()) return false;
    EdgeMap em = em_ite->second;
    if (em.find(dst) == em.end()) return false;
    return true;
}

template <typename T1, typename T2>
bool Graph<T1, T2>::AddEdge(const T2& edge)
{
    if (HasEdge(edge.src, edge.dst)) return false;
    if (!HasNode(edge.src)) this->AddNode(edge.src);
    if (!HasNode(edge.dst)) this->AddNode(edge.dst);

    auto em_ite = _edges.find(edge.src);
    if (em_ite == _edges.end()) {
        EdgeMap em;
        em.insert(std::make_pair(edge.dst, edge));
        _edges.insert(std::make_pair(edge.src, em));
    }
    else em_ite->second.insert(std::make_pair(edge.dst, edge));
    return true;
}

template <typename T1, typename T2>
bool Graph<T1, T2>::AlterEdge(const T2& edge)
{
    if (!HasEdge(edge.src, edge.dst)) return false;
    if (!HasNode(edge.src) || !HasNode(edge.dst)) return false;
    
    _edges.at(edge.src).at(edge.dst) = edge;
    return true;
}

template <typename T1, typename T2>
bool Graph<T1, T2>::AddUEdge(const T2& edge, const T2& rev_edge)
{
    return this->AddEdge(edge) && 
           this->AddEdge(rev_edge);
}

template <typename T1, typename T2>
bool Graph<T1, T2>::DeleteEdge(const size_t& src, const size_t& dst)
{
    if (!HasEdge(src, dst)) return false;
    auto em_ite = _edges.find(src);
    em_ite->second.erase(em_ite->second.find(dst));
    if (_edges[src].empty()) _edges.erase(em_ite);
    return true;
}

template <typename T1, typename T2>
T2 Graph<T1, T2>::FindConnectedEdge(const int& idx) const
{
    T2 edge;
    for (auto it = _edges.begin(); it != _edges.end(); ++it) {
        auto em = it->second;
        bool find = false;
        for (auto em_it = em.begin(); em_it != em.end(); em_it++) {
            if (em_it->second.src == idx || em_it->first == idx) {
                edge = em_it->second;
                find = true;
                break;
            }
        }
        if (find) { break; }
    }
    return edge;
}

template <typename T1, typename T2>
size_t Graph<T1, T2>::GetSize() const
{
    return _nodes.size();
}

template <typename T1, typename T2>
void Graph<T1, T2>::CountDegrees()
{
    // this->CountInDegrees();
    // this->CountOutDegrees();
    _degrees.clear();
    for (auto it = _nodes.begin(); it != _nodes.end(); ++it) {
        size_t id = it->second.id;
        _degrees[it->second.id] = _in_degrees[id] + _out_degrees[id];
    }
}

template <typename T1, typename T2>
std::unordered_map<size_t, size_t> Graph<T1, T2>::GetDegrees() const
{
    return _degrees;
}

template <typename T1, typename T2>
void Graph<T1, T2>::CountOutDegrees()
{
    _out_degrees.clear();
    for (auto ite = _edges.begin(); ite != _edges.end(); ++ite) {
        EdgeMap em = ite->second;
        _out_degrees[ite->first] = em.size();
    }
}

template <typename T1, typename T2>
std::unordered_map<size_t, size_t> Graph<T1, T2>::GetOutDegrees() const
{
    return _out_degrees;
}

template <typename T1, typename T2>
void Graph<T1, T2>::CountInDegrees()
{
    _in_degrees.clear();
    // initializing degree before using
    for (auto it = _nodes.begin(); it != _nodes.end(); ++it) {
        _in_degrees[it->second.id] = 0;
    }

    for (auto ite = _edges.begin(); ite != _edges.end(); ++ite) {
        EdgeMap em = ite->second;
        for (auto edge_ite = em.begin(); edge_ite != em.end(); ++edge_ite) {
            _in_degrees[edge_ite->first]++;
        }
    }
}

template <typename T1, typename T2>
int Graph<T1, T2>::FindLeafNode(const std::unordered_map<size_t, size_t>& degrees) const
{
    // Finding nodes with degree equals 1
    int idx = -1;
    for (auto ite = degrees.begin(); ite != degrees.end(); ++ite) {
        if (ite->second == 1) { idx = ite->first; break; }
    }
	return idx;
}

template <typename T1, typename T2>
std::vector<T2> Graph<T1, T2>::Kruskal() const
{
    std::vector<T2> mst_edges;
    std::priority_queue<T2> edges = this->CollectEdges();

    std::vector<size_t> nodes;
    nodes.reserve(_nodes.size());
    for (auto it = _nodes.begin(); it != _nodes.end(); ++it) {
        nodes.push_back(it->second.id);
    }
    std::sort(nodes.begin(), nodes.end());
    
    UnionFind union_find(_nodes.size());
    union_find.InitWithNodes(nodes);
    
    while (!edges.empty()) {
        T2 edge = edges.top(); edges.pop();
        size_t src = edge.src, dst = edge.dst;
        if (union_find.FindRoot(src) != union_find.FindRoot(dst)) {
            mst_edges.push_back(edge);
            union_find.Union(src, dst);
        }
    }
    return mst_edges;
}

template <typename T1, typename T2>
std::vector<T2> Graph<T1, T2>::ShortestPath(const size_t& src, const size_t& dst) const
{
    std::vector<T2> paths;
    std::queue<int> qu;
    std::unordered_map<size_t, int> parents;
    std::unordered_map<size_t, bool> visited;

    std::unordered_map<size_t, EdgeMap> edges;
    for (auto it = _edges.begin(); it != _edges.end(); ++it) {
        EdgeMap em = it->second;
        for (auto em_it = em.begin();  em_it != em.end(); ++em_it) {
            edges[it->first].insert(std::make_pair(em_it->first, em_it->second));
            T2 edge(em_it->second.dst, em_it->second.src, em_it->second.weight);
            edges[edge.src].insert(std::make_pair(edge.dst, edge));
        }
    }

    for (auto it = _nodes.begin(); it != _nodes.end(); ++it) { 
        visited.insert(std::make_pair(it->second.id, false)); 
    }
    qu.push(src);
    parents[src] = -1;
    visited[src] = true;

    while (!qu.empty()) {
        int cur_id = qu.front(); qu.pop();
        if (cur_id == dst) { // arrive destination
            int id = cur_id;
            std::cout << "Shortest Path(BFS): " << id << "->";
            while (parents[id] != -1) {
                // #ifdef __DEBUG__
                std::cout << parents[id] << "->";
                // #endif
                T2 edge = edges.at(parents[id]).at(id);
                paths.push_back(edge);
                id = parents[id];
            }
            std::cout << dst << std::endl;
            break;
        }

        EdgeMap em = edges.at(cur_id);
        for (auto it = em.begin(); it != em.end(); ++it) {
            if (!visited[it->first]) {
                visited[it->first] = true;
                qu.push(it->first);
                parents[it->first] = cur_id;
            }
        }
    }
    std::reverse(paths.begin(), paths.end());
    return paths;
}

template <typename T1, typename T2>
std::unordered_map<int, int> Graph<T1, T2>::NormalizedCut(const size_t cluster_num) const
{
    std::vector<std::pair<int, int>> edges;
    std::vector<int> weights;
    
    for (auto em_it = _edges.begin(); em_it != _edges.end(); ++em_it) {
        EdgeMap em = em_it->second;
        for (auto it = em.begin(); it != em.end(); ++it) {
            edges.emplace_back(it->second.src, it->second.dst);
            weights.push_back(it->second.weight);
        }
    } 

    return ComputeNormalizedMinGraphCut(edges, weights, cluster_num);
}

template <typename T1, typename T2>
void Graph<T1, T2>::ShowInfo() const
{
    std::vector<T1> nodes = this->SerializeNodes();
    // std::vector<EdgeMap> edges = this->SerializeEdges();

    LOG(INFO) << "[Graph Info]\n";
    LOG(INFO) << "Total nodes: " << std::to_string(GetNodesNum());
    LOG(INFO) << "\nTotal edges: " << std::to_string(GetEdgesNum());
    LOG(INFO) << "\n[Node]: \n";
    for (int i = 0; i < nodes.size(); i++) {
        std::cout << nodes[i].id << " ";
    }
    LOG(INFO) << "\n[Edge]: \n";
    for (auto it = _edges.begin(); it != _edges.end(); ++it) {
        auto em = it->second;
        if (em.size() == 0) continue;
        for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
            std::cout << "(" << em_it->second.src << ", " << em_it->second.dst << ") ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    // for (int i = 0; i < edges.size(); i++) {
        // if (edges[i].empty()) continue;
        // for (auto it = edges[i].begin(); it != edges[i].end(); ++it) {
            // std::cout << "(" << it->second.src << ", " << it->second.dst << ") ";
        // }
        // std::cout << std::endl;
    // }
}

template <typename T1, typename T2>
void Graph<T1, T2>::ShowInfo(const std::string& filename) const
{   
    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cout << filename << " cannot be opened!\n";
        return;
    }

    std::vector<T1> nodes = this->SerializeNodes();
    // std::vector<EdgeMap> edges = this->SerializeEdges();
    // auto edges = this->GetEdges();
    
    out << "[Graph Info]\n";
    out << "Total nodes: " + std::to_string(GetNodesNum());
    out << "\nTotal edges: " + std::to_string(GetEdgesNum());
    out << "\n[Node]: \n";
    for (int i = 0; i < nodes.size(); i++) {
        out << nodes[i].id << " ";
    } 
    
    out << "\n[Edge]: \n";
    for (auto it = _edges.begin(); it != _edges.end(); ++it) {
        auto em = it->second;
        if (em.size() == 0) continue;
        for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
            out << "(" << em_it->second.src << ", " << em_it->second.dst << ") ";
        }
        out << std::endl;
    }
    // for (int i = 0; i < edges.size(); i++) {
        // if (edges[i].empty()) continue;
        // for (auto it = edges[i].begin(); it != edges[i].end(); ++it) {
            // out << "(" << it->second.src << ", " << it->second.dst << ") ";
        // }
        // out << std::endl;
    // }
}

template <typename T1, typename T2>
size_t Graph<T1, T2>::FindConnectedComponents() const
{
    std::unordered_map<size_t, bool> visited;
    std::queue<size_t> qu;
    int connected_num = 0;

    auto edges = this->SerializeEdges();

    for (auto it = _nodes.begin(); it != _nodes.end(); ++it) { 
        visited[it->second.id] = false; 
    }

    for (auto node_it = _nodes.begin(); node_it != _nodes.end(); ++node_it) {
        if (!visited[(node_it->second).id]) {
            connected_num++;
            visited[(node_it->second).id] = true;
            qu.push((node_it->second).id);
            while (!qu.empty()) {
                size_t id =  qu.front(); qu.pop();
                EdgeMap em = edges[id];
                for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
                    if (!visited[em_it->first]) {
                        visited[em_it->first] = true;
                        qu.push(em_it->first);
                    }
                }
            }
        }
    }
    return connected_num;
}

template <typename T1, typename T2>
std::priority_queue<T2> Graph<T1, T2>::CollectEdges() const
{
    std::priority_queue<T2> edges;
    for (auto ite = _edges.begin(); ite != _edges.end(); ++ite) {
        EdgeMap em = ite->second;
        for (auto edge_ite = em.begin(); edge_ite != em.end(); ++edge_ite) {
            edges.push(edge_ite->second);
        }
    }
    return edges;
}

template <typename T1, typename T2>
std::vector<T1> Graph<T1, T2>::SerializeNodes() const
{   
    std::vector<T1> nodes;
    for (auto it = _nodes.begin(); it != _nodes.end(); ++it) {
        nodes.push_back(it->second);
    }
    std::sort(nodes.begin(), nodes.end(), Node::CompareById);
    return nodes;
}

template <typename T1, typename T2>
std::vector<std::unordered_map<size_t, T2>> Graph<T1, T2>::SerializeEdges() const
{
    std::vector<std::unordered_map<size_t, T2>> edges;
    edges.resize(_nodes.size());
    for (auto it = _edges.begin(); it != _edges.end(); ++it) {
        EdgeMap em = it->second;
        for (auto em_it = em.begin();  em_it != em.end(); ++em_it) {
            edges[it->first].insert(std::make_pair(em_it->first, em_it->second));
            T2 edge(em_it->second.dst, em_it->second.src, em_it->second.weight);
            edges[edge.src].insert(std::make_pair(edge.dst, edge));
        }
    }
    return edges;
}

template <typename T1, typename T2>
void Graph<T1, T2>::UpdateGraph()
{
    std::vector<T1> serial_nodes = this->SerializeNodes();
    std::vector<EdgeMap> serial_edges = this->SerializeEdges();

    // adjust the id of node
    for (size_t i = 0; i < serial_nodes.size(); i++) {
        size_t ori_id = serial_nodes[i].id;
        _nodes[ori_id].id = i;
    }

    // adjust the id and index of edges
    for (auto& it = _edges.begin(); it != _edges.end(); ++it) {
        size_t src = it->first;
        EdgeMap& em = it->second;
        for (auto& em_it = em.begin(); em_it != em.end(); ++em_it) {
            size_t dst = em_it->first;
            T2& edge = em_it->second;
            edge.src = _nodes[src].id;
            edge.dst = _nodes[dst].id;
            em_it->first = _nodes[dst].id;
        }
        it->first = _nodes[src].id;
    }

    // re-adjust the index of nodes
    _nodes.clear();
    for (size_t i = 0; i < serial_nodes.size(); i++) {
        size_t ori_id = serial_nodes[i].id;
        _nodes[ori_id].id = i;
    }
}

} // namespace graph
} // namespace GraphSfM
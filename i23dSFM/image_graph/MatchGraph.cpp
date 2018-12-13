#include "MatchGraph.hpp"

namespace i23dSFM{
    // MatchGraph::MatchGraph(int size) : ImageGraph(size)
    // {

    // }

    // MatchGraph::MatchGraph(const std::vector<std::string> &image_filenames, const std::vector<std::string> &sift_filenames)
    //             : ImageGraph(image_filenames, sift_filenames)
    // {

    // }

    std::vector<MotionMap> MatchGraph::GetMotionMap() const
    {
        return _adj_motion_map;
    }

    void MatchGraph::ComputeMotionMap()
    {
        // TODO
    }

} //namespace i23dSFM
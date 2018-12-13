#ifndef MATCH_GRAPH_H
#define MATCH_GRAPH_H

#include <unordered_map>

#include "Eigen/Core"
#include "ImageGraph.hpp"


namespace i23dSFM{
    
typedef std::unordered_map<int, Eigen::Matrix3d> MotionMap;

class MatchGraph : public ImageGraph
{
private:
    std::vector<MotionMap> _adj_motion_map;

public:
    MatchGraph(int size) : ImageGraph(size) {};
    MatchGraph(const std::vector<std::string> &image_filenames, const std::vector<std::string> &sift_filenames)
                : ImageGraph(image_filenames, sift_filenames){};
    /**
     * @brief Get motion map
     */ 
    std::vector<MotionMap> GetMotionMap() const;
    /**
     * @breif Compute relative motion between image(i, j)
     * @param
     */ 
    void ComputeMotionMap();
};

}   //namespace i23dSFM

#endif
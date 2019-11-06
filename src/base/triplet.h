#ifndef TRIPLET_H_INCLUDED
#define TRIPLET_H_INCLUDED

#include "estimators/rotation_averaging.h" //
#include "util/types.h"
#include "estimators/two_view_geometry.h"
#include "util/func.h"

using namespace colmap;

namespace GraphSfM
{

/**
* @brief Simple container for tuple of three value
* @note It is used to store the node id of triplets of a graph.
* @todo Why not using std::tuple ?
*/
struct Triplet
{

  /**
  * @brief Constructor
  * @param ii First element of the triplet
  * @param jj Second element of the triplet
  * @param kk Third element of the triplet
  */
  Triplet( IndexT ii, IndexT jj, IndexT kk )
    : i( ii ), j( jj ), k( kk )
  { }

  /**
  * @brief Indicate if an edge contains one of the element of the triplet
  * @param edge Edge to test
  * @retval true if edge contains at least one of index of the triplet
  * @retval false if edge contains none of the index of the triplet
  */
  bool contain( const std::pair<IndexT, IndexT> & edge ) const
  {
    const IndexT It = edge.first;
    const IndexT Jt = edge.second;
    if ( ( It == i || It == j || It == k ) &&
         ( Jt == i || Jt == j || Jt == k ) && It != Jt )
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  /**
  * @brief Indicate if triplet are the same
  * @param m1 First triplet
  * @param m2 Second triplet
  * @retval true if triplets are the same
  * @retval false if triplets are differents
  */
  friend bool operator==( const Triplet& m1, const Triplet& m2 )
  {
    return m1.contain( std::make_pair( m2.i, m2.j ) )
           && m1.contain( std::make_pair( m2.i, m2.k ) );
  }

  /**
  * @brief Indicate if triplet are differents
  * @param m1 First triplet
  * @param m2 Second triplet
  * @retval true if triplets are differents
  * @retval false if triplets are the same
  */
  friend bool operator!=( const Triplet& m1, const Triplet& m2 )
  {
    return !( m1 == m2 );
  }

  /**
  * @brief Stream operation
  * @param os Stream in which triplet is written
  * @param t Triplet to write
  * @return stream after write operation
  */
  friend std::ostream & operator<<( std::ostream & os, const Triplet & t )
  {
    os << t.i << " " << t.j << " " << t.k << std::endl;
    return os;
  }

  /// First index id
  IndexT i;

  /// Second index id
  IndexT j;

  /// Third index id
  IndexT k;

};

void TripletListing(std::map<image_pair_t, TwoViewGeometry>& EpipolarGeometry, std::vector<Triplet>& triplets,
                    std::map<image_pair_t, double>& EG_ratio, std::map<image_pair_t, int>& EdgeVotes)
{
    std::vector< std::vector<int> > AdjMatrix;
    std::set<int> nodes;
    std::map<image_pair_t, bool> mapEdge;
    std::map<image_pair_t, int> EdgeInliers;
    int maxNodeIndex = 0;
    for(const auto& epi : EpipolarGeometry)
    {
        int I = epi.first / kMaxNumImages;
        int J = epi.first % kMaxNumImages;
        nodes.insert(I);
        nodes.insert(J);
        mapEdge[epi.first] = false;

        if(J > maxNodeIndex)
            maxNodeIndex = J;
    }
    maxNodeIndex += 1;

    std::vector<int> nodeC;
    for(const auto& node : nodes)
    {
        nodeC.push_back(node);
    }
    LOG(INFO) << "creating adjacent matrix, with " << maxNodeIndex << " nodes" << std::endl;
    AdjMatrix.resize(maxNodeIndex);
    for(const auto& epi : EpipolarGeometry)
    {
        int I = epi.first / kMaxNumImages;
        int J = epi.first % kMaxNumImages;
        EG_ratio[epi.first] = 0.0;
        EdgeVotes[epi.first] = 0;
        EdgeInliers[epi.first] = 0;
        AdjMatrix[I].push_back(J); /// upper left triangle
    }

    triplets.clear();
    LOG(INFO) << "finding triplets" << std::endl;
    for(const auto& node : nodeC)
    {
        std::vector<int> nodeEdge = AdjMatrix[node];
        while(nodeEdge.size() > 1)
        {
            int dst1 = nodeEdge[0];
            image_pair_t dst01 = ComputePairID(node, dst1);
            for(int iN = 1; iN < (int)nodeEdge.size(); iN++)
            {
                int dst2 = nodeEdge[iN];
                image_pair_t dst12 = ComputePairID(dst1, dst2);
                if(EpipolarGeometry.count(dst12) > 0 && !mapEdge[dst12])
                {
                    if(dst1 < dst2)
                        triplets.push_back(Triplet(node, dst1, dst2));
                    else
                        triplets.push_back(Triplet(node, dst2, dst1));
                }
            }
            mapEdge[dst01] = true;
            nodeEdge.erase(nodeEdge.begin());
        }

    }
    LOG(INFO) << triplets.size() << " triplets are found" << std::endl;

    //ofstream outTri;
    //outTri.open("Triplets.txt");
    const Eigen::Matrix3d Identity = Eigen::Matrix3d::Identity();
    std::vector<Triplet> tripletsInlier;
    for(const auto& tri : triplets)
    {
        image_pair_t ij = ComputePairID(tri.i, tri.j);
        image_pair_t jk = ComputePairID(tri.j, tri.k);
        image_pair_t ik = ComputePairID(tri.i, tri.k);
        if(EpipolarGeometry.count(ij) > 0 && EpipolarGeometry.count(ik) > 0 && EpipolarGeometry.count(jk) > 0)
        {
            Eigen::Matrix3d Rij = QuaternionToRotationMatrix(EpipolarGeometry[ij].qvec);
            Eigen::Matrix3d Rik = QuaternionToRotationMatrix(EpipolarGeometry[ik].qvec);
            Eigen::Matrix3d Rjk = QuaternionToRotationMatrix(EpipolarGeometry[jk].qvec);
            Eigen::Matrix3d Rcycle = Rij * Rjk * Rik.transpose();

            double angleError = ComputeRotationError(Rcycle, Identity);
            //outTri << tri.i << " " << tri.j << " " << tri.k << " " << angleError << std::endl;
            EdgeVotes[ij]++;
            EdgeVotes[jk]++;
            EdgeVotes[ik]++;

            if(angleError <= 30.0)
            {
                EdgeInliers[ij]++;
                EdgeInliers[jk]++;
                EdgeInliers[ik]++;
                tripletsInlier.push_back(tri);
            }
        }
    }

    for(const auto& vote : EdgeInliers)
    {
        EG_ratio[vote.first] = (double)(vote.second) / EdgeVotes[vote.first];
    }
}
}

#endif // TRIPLET_H_INCLUDED

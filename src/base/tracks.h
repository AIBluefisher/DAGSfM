
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Implementation of [1] an efficient algorithm to compute track from pairwise
//  correspondences.
//
//  [1] Pierre Moulon and Pascal Monasse,
//    "Unordered feature tracking made fast and easy" CVMP 2012.
//
// It tracks the position of features along the series of image from pairwise
//  correspondences.
//
// From map< [imageI,ImageJ], [indexed matches array] > it builds tracks.
//
// Usage :
//  PairWiseMatches map_Matches;
//  PairedIndMatchImport(sMatchFile, map_Matches); // Load series of pairwise matches
//  //---------------------------------------
//  // Compute tracks from matches
//  //---------------------------------------
//  TracksBuilder tracksBuilder;
//  tracks::STLMAPTracks map_tracks;
//  tracksBuilder.Build(map_Matches); // Build: Efficient fusion of correspondences
//  tracksBuilder.Filter();           // Filter: Remove tracks that have conflict
//  tracksBuilder.ExportToSTL(map_tracks); // Build tracks with STL compliant type
//

#ifndef BASE_TRACKS_H_
#define BASE_TRACKS_H_

#include "feature/types.h"
#include "util/types.h"
#include "base/structure.h"
#include "base/union_find.h"
#include "base/flat_pair_map.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <vector>

using namespace colmap;

namespace GraphSfM  {
namespace tracks  {

// Data structure to store a track: collection of {ImageId,FeatureId}
//  The corresponding image points with their imageId and FeatureId.
typedef std::map<uint32_t,uint32_t> submapTrack;
// A track is a collection of {trackId, submapTrack}
typedef std::map< size_t, submapTrack > STLMAPTracks;

struct TracksBuilder
{
    typedef std::pair<uint32_t, uint32_t> indexedFeaturePair;

    flat_pair_map<indexedFeaturePair, uint32_t> map_node_to_index;
    UnionFind uf_tree;

    /// Build tracks for a given series of pairWise matches
    void Build( const std::map<colmap::image_pair_t, colmap::FeatureMatches> &  map_pair_wise_matches)
    {
        // 1. We need to know how much single set we will have.
        //   i.e each set is made of a tuple : (imageIndex, featureIndex)
        typedef std::set<indexedFeaturePair> SetIndexedPair;
        SetIndexedPair allFeatures;

        // For each couple of images list the used features
        const size_t kMaxNumImages = static_cast<size_t>(std::numeric_limits<int32_t>::max());
        for (const auto& iter : map_pair_wise_matches) {
            const uint32_t I = static_cast<uint32_t>(iter.first / kMaxNumImages);
            const uint32_t J = static_cast<uint32_t>(iter.first % kMaxNumImages);
            const colmap::FeatureMatches & vec_FilteredMatches = iter.second;
            //LOG(INFO) << I << " " << J << " " << vec_FilteredMatches.size() << std::endl;
            // Retrieve all shared features and add them to a set
            for(const auto& cur_filtered_match : vec_FilteredMatches) {
                allFeatures.emplace(I,cur_filtered_match.point2D_idx1);
                allFeatures.emplace(J,cur_filtered_match.point2D_idx2);
            }
        }

        // 2. Build the 'flat' representation where a tuple (the node)
        //  is attached to a unique index.
        map_node_to_index.reserve(allFeatures.size());
        unsigned int cpt = 0;
        for (const auto& feat : allFeatures) {
            map_node_to_index.emplace_back(feat, cpt);
            ++cpt;
        }
        // Sort the flat_pair_map
        map_node_to_index.sort();
        // Clean some memory
        allFeatures.clear();

        // 3. Add the node and the pairwise correpondences in the UF tree.
        uf_tree.InitSets(map_node_to_index.size());

        // 4. Union of the matched features corresponding UF tree sets
        for (const auto& iter : map_pair_wise_matches ) {
            const uint32_t I = static_cast<uint32_t>(iter.first / kMaxNumImages);
            const uint32_t J = static_cast<uint32_t>(iter.first % kMaxNumImages);
            const colmap::FeatureMatches & vec_FilteredMatches = iter.second;
            //LOG(INFO) << I << " " << J << " " << vec_FilteredMatches.size() << std::endl;
            for (const FeatureMatch & match : vec_FilteredMatches) {
                const indexedFeaturePair pairI(I, match.point2D_idx1);
                const indexedFeaturePair pairJ(J, match.point2D_idx2);
                // Link feature correspondences to the corresponding containing sets.
                uf_tree.Union(map_node_to_index[pairI], map_node_to_index[pairJ]);
            }
        }
    }

    /// Remove bad tracks (too short or track with ids collision)
    bool Filter(size_t nLengthSupTo = 2)
    {
        // Remove bad tracks:
        // - track that are too short,
        // - track with id conflicts:
        //    i.e. tracks that have many times the same image index

        // From the UF tree, create tracks of the image indexes.
        //  If an image index appears two time the track must disappear
        //  If a track is too short it has to be removed.
        std::map<unsigned int, std::set<unsigned int> > tracks;

        std::set<unsigned int> problematic_track_id;
        // Build tracks from the UF tree, track problematic ids.
        for (unsigned int k = 0; k < map_node_to_index.size(); ++k) {
            const unsigned int & track_id = uf_tree.m_cc_parent[k];
            if (problematic_track_id.count(track_id) != 0) {
                continue; // Track already marked
            }

            const auto & feat = map_node_to_index[k];

            if (tracks[track_id].count(feat.first.first)) {
                problematic_track_id.insert(track_id);
            } else {
                tracks[track_id].insert(feat.first.first);
            }
        }

        // - track that are too short,
        for (const auto & val : tracks) {
            if (val.second.size() < nLengthSupTo) {
                problematic_track_id.insert(val.first);
            }
        }

        for (unsigned int & root_index : uf_tree.m_cc_parent) {
            if (problematic_track_id.count(root_index) > 0) {
                // reset selected root
                uf_tree.m_cc_size[root_index] = 1;
                root_index = std::numeric_limits<unsigned int>::max();
            }
        }
        return false;
    }

    /// Return the number of connected set in the UnionFind structure (tree forest)
    size_t NbTracks() const
    {
        std::set<unsigned int> parent_id(uf_tree.m_cc_parent.begin(), uf_tree.m_cc_parent.end());
        // Erase the "special marker" that depicted rejected tracks
        parent_id.erase(std::numeric_limits<unsigned int>::max());
        return parent_id.size();
    }

    /// Export tracks as a map (each entry is a sequence of imageId and featureIndex):
    ///  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
    void ExportToSTL(STLMAPTracks & map_tracks)
    {
        map_tracks.clear();
        for (unsigned int k = 0; k < map_node_to_index.size(); ++k) {
            const auto & feat = map_node_to_index[k];
            const unsigned int track_id = uf_tree.m_cc_parent[k];
            if( // ensure never add rejected elements (track marked as invalid)
              track_id != std::numeric_limits<unsigned int>::max()
              // ensure never add 1-length track element (it's not a track)
              && uf_tree.m_cc_size[track_id] > 1) {
                  map_tracks[track_id].insert(feat.first);
            }
        }
    }
};
} // namespace tracks
} // namespace GraphSfM

#endif // GraphSfM_TRACKS_H_

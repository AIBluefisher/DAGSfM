// BSD 3-Clause License

// Copyright (c) 2018, 陈煜
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "i23dSFM/sfm/sfm.hpp"

#include <cstdlib>
#include <fstream>
#include <string>

using namespace i23dSFM;
using namespace i23dSFM::cameras;
using namespace i23dSFM::matching;
using namespace i23dSFM::sfm;
using namespace i23dSFM::tracks;
using namespace std;


int main(int argc, char* argv[])
{
    string matches_filename = argv[1];
    string dir = stlplus::folder_part(matches_filename);
    // pairwise matches container:
    PairWiseMatches map_Matches;

    // Fil the pairwise correspondeces or load a series of pairwise matches from a file
    PairedIndMatchImport(matches_filename, map_Matches);

    //---------------------------------------
    // Compute tracks from pairwise matches
    //---------------------------------------
    TracksBuilder tracksBuilder;
    tracks::STLMAPTracks map_tracks;  // The track container
    tracksBuilder.Build(map_Matches); // Build: Efficient fusion of correspondences
    tracksBuilder.Filter();           // Filter: Remove track that have conflict
    tracksBuilder.FilterPairWiseMinimumMatches(20);
    tracksBuilder.ExportToSTL(map_tracks); // Build tracks with STL compliant type

    ofstream out_stream(dir + "/tracks.txt");
    if (!out_stream.is_open()) {
        std::cerr << dir + "/tracks.txt cannot be created!\n";
        return false;
    }

    // for ()

    // In order to visit all the tracks, follow this code:
    for (tracks::STLMAPTracks::const_iterator iterT = map_tracks.begin();
        iterT != map_tracks.end(); ++ iterT) {
        const IndexT trackId = iterT->first;
        const tracks::submapTrack & track = iterT->second;
        for ( tracks::submapTrack::const_iterator iterTrack = track.begin();
        iterTrack != track.end(); ++iterTrack) {
            const IndexT imageId = iterTrack->first;
            const IndexT featId = iterTrack->second;
        }
    }
}
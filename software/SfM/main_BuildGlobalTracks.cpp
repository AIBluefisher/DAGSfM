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
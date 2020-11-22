// BSD 3-Clause License

// Copyright (c) 2020, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "base/track_selection.h"

#include <Eigen/Core>
#include <algorithm>
#include <unordered_set>
#include <utility>

#include "base/camera.h"
#include "base/image.h"
#include "base/projection.h"
#include "util/hash.h"

using namespace std;

namespace {

// Track statistics are the track length and mean reprojection error.
typedef pair<int, double> TrackStatistics;
typedef pair<point3D_t, TrackStatistics> GridCellElement;
typedef unordered_map<Eigen::Vector2i, vector<GridCellElement>> ImageGrid;

// Sorts the grid cell elements by the track statistics, which will sort first
// by the (truncated) track length, then by the mean reprojection error.
bool CompareGridCellElements(const pair<point3D_t, TrackStatistics>& element1,
                             const pair<point3D_t, TrackStatistics>& element2) {
  return element1.second < element2.second;
}

// Compute the reprojrection error and truncated track length for the
// specific track.
TrackStatistics ComputeStatisticsForTrack(
    const Reconstruction& reconstruction, const point3D_t track_id,
    const int long_track_length_threshold) {
  // Any track that reach this function are guaranteed to exist and
  // be estimated, so no need to check for that here.
  const Point3D& point3d = reconstruction.Point3D(track_id);
  const std::vector<TrackElement> track_elements = point3d.Track().Elements();
  std::vector<image_t> images_observing_track;
  for (const auto track_element : track_elements) {
    images_observing_track.push_back(track_element.image_id);
  }

  double sq_reprojection_error_sum = 0.0;
  int num_valid_reprojections = 0;
  // Compute the squared reprojection error for each view that observes this
  // track and add it to the accumulating sum.
  for (const TrackElement element : track_elements) {
    if (!reconstruction.IsImageRegistered(element.image_id)) {
      continue;
    }

    const Image& image = reconstruction.Image(element.image_id);
    const Camera& camera = reconstruction.Camera(image.CameraId());
    const Point2D& point2d = image.Point2D(element.point2D_idx);

    sq_reprojection_error_sum += colmap::CalculateSquaredReprojectionError(
        point2d.XY(), point3d.XYZ(), image.Qvec(), image.Tvec(), camera);
    num_valid_reprojections++;
  }

  // Compute and return the track statistics.
  const int truncated_track_length =
      std::min(num_valid_reprojections, long_track_length_threshold);
  const double mean_sq_reprojection_error =
      sq_reprojection_error_sum / static_cast<double>(num_valid_reprojections);

  return TrackStatistics(truncated_track_length, mean_sq_reprojection_error);
}

// Compute the mean reprojection error and the truncated track length of each
// track. We truncate the track length based on the observation that while
// larger track lengths provide better constraints for bundle adjustment, larger
// tracks are also more likely to contain outliers in our experience. Truncating
// the track lengths enforces that the long tracks with the lowest reprojection
// error are chosen.
void ComputeTrackStatistics(
    const Reconstruction& reconstruction,
    const std::unordered_set<image_t> image_ids,
    const int long_track_length_threshold,
    std::unordered_map<point3D_t, TrackStatistics>* track_statistics) {
  // Iterate over all registered views and compute the track statistics for each
  // track we encounter.
  for (const image_t image_id : image_ids) {
    const Image image = reconstruction.Image(image_id);
    const std::vector<Point2D> points2d = image.Points2D();
    // Compute statistics for each point that has point3d and compute the
    // track statistics for each track we encounter.
    for (const Point2D& point2d : points2d) {
      if (!point2d.HasPoint3D()) {
        continue;
      }
      const point3D_t track_id = point2d.Point3DId();
      if (track_statistics->count(track_id) != 0) {
        continue;
      }

      // Compute the track statistics and add it to the output map.
      const TrackStatistics& statistics_for_this_track =
          ComputeStatisticsForTrack(reconstruction, track_id,
                                    long_track_length_threshold);
      track_statistics->emplace(track_id, statistics_for_this_track);
    }
  }
}

// Select tracks from the image to ensure good spatial coverage of the image. To
// do this, we first bin the tracks into grid cells in an image grid. Then
// within each cell we find the best ranked track and add it to the list of
// tracks to optimize.
void SelectBestTracksFromEachImageGridCell(
    const Reconstruction& reconstruction, const Image& image,
    const int grid_cell_size,
    const std::unordered_map<point3D_t, TrackStatistics>& track_statistics,
    std::unordered_set<point3D_t>* tracks_to_optimize) {
  static const double inv_grid_cell_size = 1.0 / grid_cell_size;

  // Hash each feature into a grid cell.
  ImageGrid image_grid;
  const std::vector<Point2D> points2d = image.Points2D();
  for (const Point2D& point2d : points2d) {
    if (!point2d.HasPoint3D()) {
      continue;
    }

    const Eigen::Vector2d feature = point2d.XY();
    const point3D_t track_id = point2d.Point3DId();

    if (track_statistics.count(track_id) == 0) {
      continue;
    }

    const TrackStatistics& current_track_statistics =
        track_statistics.at(track_id);
    const Eigen::Vector2i grid_cell =
        (feature * inv_grid_cell_size).cast<int>();

    image_grid[grid_cell].emplace_back(track_id, current_track_statistics);
  }

  // Select the best feature from each grid cell and add it to the tracks to
  // optimize.
  for (auto& grid_cell : image_grid) {
    // Order the features in each cell by track length first, then mean
    // reprojection error.
    const GridCellElement& grid_cell_element =
        *std::min_element(grid_cell.second.begin(), grid_cell.second.end(),
                          CompareGridCellElements);
    // Insert the track id in to the tracks to optimize.
    tracks_to_optimize->emplace(grid_cell_element.first);
  }
}

// Selects the top ranked tracks that have not already been chosen until the
// view observes the minimum number of optimized tracks.
void SelectTopRankedTracksInView(
    const Reconstruction& reconstruction,
    const std::unordered_map<point3D_t, TrackStatistics>& track_statistics,
    const Image& image, const int min_num_optimized_tracks_per_view,
    std::unordered_set<point3D_t>* tracks_to_optimize) {
  int num_optimized_tracks = 0;
  int num_estimated_tracks = 0;
  std::vector<GridCellElement> ranked_candidate_tracks;

  const std::vector<Point2D> points2d = image.Points2D();
  for (const Point2D& point2d : points2d) {
    if (!point2d.HasPoint3D()) {
      continue;
    }

    // We only reach this point if the track is estimated.
    num_estimated_tracks++;

    const point3D_t track_id = point2d.Point3DId();
    // If the track is already slated for optimization, increase the count of
    // optimized features.
    if (tracks_to_optimize->count(track_id) != 0) {
      num_optimized_tracks++;
      // If the number of optimized tracks is greater than the minimum then
      // we can return early since we know that no more features need to added
      // for this image.
      if (num_optimized_tracks >= min_num_optimized_tracks_per_view) {
        return;
      }
    } else {
      // If the track is not already set to be optimized then add it to the list
      // of candidate tracks.
      if (track_statistics.count(track_id) != 0) {
        ranked_candidate_tracks.emplace_back(track_id,
                                             track_statistics.at(track_id));
      }
    }
  }

  // We only reach this point if the number of optimized tracks is less than the
  // minimum. If that is the case then we add the top candidate features until
  // the minimum number of features observed is met.
  if (num_optimized_tracks != num_estimated_tracks) {
    // Select how many tracks to add. If we need more tracks than are estimated
    // then we simply add all remaining features.
    const int num_optimized_tracks_needed =
        std::min(min_num_optimized_tracks_per_view, num_estimated_tracks) -
        num_optimized_tracks;
    std::partial_sort(
        ranked_candidate_tracks.begin(),
        ranked_candidate_tracks.begin() + num_optimized_tracks_needed,
        ranked_candidate_tracks.end());
    // Add the candidate tracks to the list of tracks to be optimized.
    for (int i = 0; i < num_optimized_tracks_needed; i++) {
      tracks_to_optimize->emplace(ranked_candidate_tracks[i].first);
    }
  }
}

}  // namespace

namespace DAGSfM {

// The efficiency of large scale bundle adjustment can be dramatically increased
// by choosing only a subset of 3d points to optimize, as the 3d points tend to
// have increasing scene redundancy. If the points are chosen in a way that
// Properly constraints the nonlinear optimization, similar results in quality
// may be observed compared to optimized all tracks.
//
// The 3d points are chosen such that they fit the following criteria:
//    a) High confidence (i.e. low reprojection error).
//    b) Longer tracks are preferred.
//    c) The tracks used for optimization provide a good spatial coverage in
//    each image.
bool SelectGoodTracksForBundleAdjustment(
    const Reconstruction& reconstruction, const int long_track_length_threshold,
    const int image_grid_cell_size_pixels,
    const int min_num_optimized_tracks_per_view,
    std::unordered_set<point3D_t>* tracks_to_optimize) {
  // Get the registered image ids.
  std::vector<image_t> vec_reg_image_ids = reconstruction.RegImageIds();
  std::unordered_set<image_t> reg_image_ids(vec_reg_image_ids.begin(),
                                            vec_reg_image_ids.end());

  return SelectGoodTracksForBundleAdjustment(
      reconstruction, reg_image_ids, long_track_length_threshold,
      image_grid_cell_size_pixels, min_num_optimized_tracks_per_view,
      tracks_to_optimize);
}

bool SelectGoodTracksForBundleAdjustment(
    const Reconstruction& reconstruction,
    const std::unordered_set<image_t>& image_ids,
    const int long_track_length_threshold,
    const int image_grid_cell_size_pixels,
    const int min_num_optimized_tracks_per_view,
    std::unordered_set<point3D_t>* tracks_to_optimize) {
  // Compute the track mean reprojection errors.
  std::unordered_map<point3D_t, TrackStatistics> track_statistics;
  ComputeTrackStatistics(reconstruction, image_ids, long_track_length_threshold,
                         &track_statistics);

  // For each image, divide the image into a grid and choose the highest quality
  // tracks from each grid cell. This encourages good spatial coverage of tracks
  // within each image.
  for (const image_t image_id : image_ids) {
    const Image& image = reconstruction.Image(image_id);

    // Select the best tracks from each grid cell in the image and add them to
    // the container of tracks to be optimized.
    SelectBestTracksFromEachImageGridCell(reconstruction, image,
                                          image_grid_cell_size_pixels,
                                          track_statistics, tracks_to_optimize);
  }

  // To this point, we have only added features that have as full spatial
  // coverage as possible within each image but we have not ensured that
  // each image is constrained by at least K features. So, we cycle through all
  // images again and add the top M tracks that have not already been added.
  for (const image_t image_id : image_ids) {
    const Image& image = reconstruction.Image(image_id);

    // If this view is not constrained by enough optimized tracks, add the top
    // ranked features until there are enough tracks constraining the view.
    SelectTopRankedTracksInView(reconstruction, track_statistics, image,
                                min_num_optimized_tracks_per_view,
                                tracks_to_optimize);
  }

  return true;
}

}  // namespace DAGSfM
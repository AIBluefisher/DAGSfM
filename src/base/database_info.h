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

#ifndef COLMAP_SRC_BASE_DATABASE_INFO_H_
#define COLMAP_SRC_BASE_DATABASE_INFO_H_

#include <rpc/msgpack.hpp>

#include "base/database.h"
#include "map_reduce/msgpack_adaptor.h"
#include "util/misc.h"

using namespace colmap;

namespace DAGSfM {

struct DatabaseInfo {
  std::vector<Camera> cameras;
  std::vector<Image> images;
  std::unordered_map<image_t, FeatureKeypoints> image_keypoints_mapper;
  std::unordered_map<ImagePair, FeatureMatches> images_matches_mapper;
  std::unordered_map<ImagePair, TwoViewGeometry> images_twoview_geometries;

  MSGPACK_DEFINE(cameras, images, image_keypoints_mapper, images_matches_mapper,
                 images_twoview_geometries);

  DatabaseInfo() {}

  DatabaseInfo(const DatabaseInfo& database_info) {
    cameras = database_info.cameras;
    images = database_info.images;
    image_keypoints_mapper = database_info.image_keypoints_mapper;
    images_matches_mapper = database_info.images_matches_mapper;
    images_twoview_geometries = database_info.images_twoview_geometries;
  }

  void Clear() {
    cameras.clear();
    images.clear();
    image_keypoints_mapper.clear();
    images_matches_mapper.clear();
    images_twoview_geometries.clear();
  }

  void Merge(const DatabaseInfo& database_info) {
    // LOG(INFO) << "Merging cameras";
    cameras.reserve(cameras.size() + database_info.cameras.size());
    std::unordered_set<camera_t> cameras_idx;
    for (const Camera& camera : cameras) {
      cameras_idx.insert(camera.CameraId());
    }

    for (const Camera& camera : database_info.cameras) {
      if (cameras_idx.count(camera.CameraId()) > 0) {
        continue;
      }
      cameras.emplace_back(camera);
    }

    // LOG(INFO) << "Merging images";
    images.reserve(images.size() + database_info.images.size());
    std::unordered_set<image_t> images_idx;
    for (const Image& image : images) {
      images_idx.insert(image.ImageId());
    }

    for (const Image& image : database_info.images) {
      if (images_idx.count(image.ImageId()) > 0) {
        continue;
      }
      images.emplace_back(image);
    }

    // LOG(INFO) << "Merging keypoints";
    image_keypoints_mapper.reserve(image_keypoints_mapper.size() +
                                   database_info.image_keypoints_mapper.size());
    for (const Image& image : database_info.images) {
      if (image_keypoints_mapper.count(image.ImageId()) > 0) {
        continue;
      }
      image_keypoints_mapper.emplace(
          image.ImageId(),
          database_info.image_keypoints_mapper.at(image.ImageId()));
    }

    // LOG(INFO) << "merging matches";
    images_matches_mapper.reserve(images_matches_mapper.size() +
                                  database_info.images_matches_mapper.size());
    for (const auto& image_pair : database_info.images_matches_mapper) {
      if (images_matches_mapper.count(image_pair.first) > 0) {
        continue;
      }
      images_matches_mapper.emplace(image_pair.first, image_pair.second);
    }

    // LOG(INFO) << "two view geometries";
    images_twoview_geometries.reserve(
        images_twoview_geometries.size() +
        database_info.images_twoview_geometries.size());
    for (const auto& image_pair : database_info.images_twoview_geometries) {
      if (images_twoview_geometries.count(image_pair.first) > 0) {
        continue;
      }
      images_twoview_geometries.emplace(image_pair.first, image_pair.second);
    }
  }

  bool LoadPartialDatabase(const std::string& database_path,
                           const std::vector<image_t>& image_ids) {
    if (!ExistsPath(database_path)) {
      LOG(ERROR) << "Cannot load database " << database_path;
      return false;
    }

    Database database(database_path);

    cameras = database.ReadAllCameras();

    std::unordered_set<image_t> image_set(image_ids.begin(), image_ids.end());
    std::vector<Image> all_images = database.ReadAllImages();
    for (const Image& image : all_images) {
      if (image_set.count(image.ImageId()) > 0) {
        images.emplace_back(image);
      }
    }

    std::vector<std::pair<image_t, image_t>> image_pairs;
    std::vector<int> num_inliers;
    database.ReadTwoViewGeometryNumInliers(&image_pairs, &num_inliers);

    image_keypoints_mapper.reserve(images.size());
    for (const Image& image : images) {
      FeatureKeypoints keypoints = database.ReadKeypoints(image.ImageId());
      image_keypoints_mapper.emplace(image.ImageId(), keypoints);
    }

    images_matches_mapper.reserve(image_pairs.size());
    for (const auto& image_pair : image_pairs) {
      if (image_set.count(image_pair.first) > 0 &&
          image_set.count(image_pair.second) > 0) {
        FeatureMatches matches =
            database.ReadMatches(image_pair.first, image_pair.second);
        images_matches_mapper.emplace(image_pair, matches);
      }
    }

    images_twoview_geometries.reserve(image_pairs.size());
    for (const auto& image_pair : image_pairs) {
      if (image_set.count(image_pair.first) > 0 &&
          image_set.count(image_pair.second) > 0) {
        TwoViewGeometry two_view_geometry =
            database.ReadTwoViewGeometry(image_pair.first, image_pair.second);
        images_twoview_geometries.emplace(image_pair, two_view_geometry);
      }
    }
    database.Close();
    return true;
  }

  bool LoadDatabase(const std::string& database_path) {
    if (!ExistsPath(database_path)) {
      return false;
    }

    Database database(database_path);

    cameras = database.ReadAllCameras();
    images = database.ReadAllImages();

    std::vector<std::pair<image_t, image_t>> image_pairs;
    std::vector<int> num_inliers;
    database.ReadTwoViewGeometryNumInliers(&image_pairs, &num_inliers);

    image_keypoints_mapper.reserve(images.size());
    for (const Image& image : images) {
      FeatureKeypoints keypoints = database.ReadKeypoints(image.ImageId());
      image_keypoints_mapper.emplace(image.ImageId(), keypoints);
    }

    images_matches_mapper.reserve(image_pairs.size());
    for (const auto& image_pair : image_pairs) {
      FeatureMatches matches =
          database.ReadMatches(image_pair.first, image_pair.second);
      images_matches_mapper.emplace(image_pair, matches);
    }

    images_twoview_geometries.reserve(image_pairs.size());
    for (const auto& image_pair : image_pairs) {
      TwoViewGeometry two_view_geometry =
          database.ReadTwoViewGeometry(image_pair.first, image_pair.second);
      images_twoview_geometries.emplace(image_pair, two_view_geometry);
    }

    database.Close();

    return true;
  }

  void ExportToDatabase(Database& database) {
    LOG(INFO) << "image size: " << images.size();
    LOG(INFO) << "camera size: " << cameras.size();
    LOG(INFO) << "keypoints size: " << image_keypoints_mapper.size();

    for (const auto& camera : cameras) {
      if (!database.ExistsCamera(camera.CameraId())) {
        database.WriteCamera(camera, true);
      }
    }

    for (const auto& image : images) {
      if (!database.ExistsImage(image.ImageId())) {
        database.WriteImage(image, true);
      }
    }

    for (const auto& it : image_keypoints_mapper) {
      if (!database.ExistsKeypoints(it.first)) {
        database.WriteKeypoints(it.first, it.second);
      }
    }

    for (const auto& it : images_matches_mapper) {
      if (!database.ExistsMatches(it.first.first, it.first.second)) {
        database.WriteMatches(it.first.first, it.first.second, it.second);
      }
    }

    for (const auto& it : images_twoview_geometries) {
      if (!database.ExistsInlierMatches(it.first.first, it.first.second)) {
        database.WriteTwoViewGeometry(it.first.first, it.first.second,
                                      it.second);
      }
    }
  }

  void UpdateImageIndex(
      const std::unordered_map<std::string, image_t>& global_image_name_to_id) {
    // LOG(INFO) << "Update images";
    std::unordered_map<image_t, image_t> local_to_global;

    for (Image& image : images) {
      const image_t local_id = image.ImageId();
      const std::string image_name = image.Name();
      const image_t global_id = global_image_name_to_id.at(image_name);
      image.SetImageId(global_id);
      image.SetCameraId(global_id);
      local_to_global.emplace(local_id, global_id);
    }

    for (Camera& camera : cameras) {
      const camera_t local_id = camera.CameraId();
      const camera_t global_id = local_to_global[local_id];
      camera.SetCameraId(global_id);
    }

    // LOG(INFO) << "update keypoints";
    std::unordered_map<image_t, FeatureKeypoints>
        updated_image_keypoints_mapper;
    for (auto& it : image_keypoints_mapper) {
      updated_image_keypoints_mapper.emplace(local_to_global[it.first],
                                             it.second);
    }
    updated_image_keypoints_mapper.swap(image_keypoints_mapper);

    // LOG(INFO) << "update matches";
    // LOG(INFO) << "matches size: " << images_matches_mapper.size();
    std::unordered_map<ImagePair, FeatureMatches> updated_images_matches_mapper;
    for (auto& it : images_matches_mapper) {
      updated_images_matches_mapper.emplace(
          ImagePair(local_to_global[it.first.first],
                    local_to_global[it.first.second]),
          it.second);
    }
    updated_images_matches_mapper.swap(images_matches_mapper);

    // LOG(INFO) << "update two view geometries";
    // LOG(INFO) << "two view geometries size: " <<
    // images_twoview_geometries.size();
    std::unordered_map<ImagePair, TwoViewGeometry>
        updated_images_twoview_geometries;
    for (auto& it : images_twoview_geometries) {
      updated_images_twoview_geometries.emplace(
          ImagePair(local_to_global[it.first.first],
                    local_to_global[it.first.second]),
          it.second);
    }
    updated_images_twoview_geometries.swap(images_twoview_geometries);
  }
};

}  // namespace DAGSfM

#endif
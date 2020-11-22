// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "controllers/automatic_reconstruction.h"

#include "base/undistortion.h"
#include "controllers/incremental_mapper_controller.h"
#include "feature/extraction.h"
#include "feature/matching.h"
#include "util/misc.h"
#include "util/option_manager.h"

namespace colmap {

AutomaticReconstructionController::AutomaticReconstructionController(
    const Options& options, ReconstructionManager* reconstruction_manager)
    : options_(options),
      reconstruction_manager_(reconstruction_manager),
      active_thread_(nullptr) {
  CHECK(ExistsDir(options_.workspace_path));
  CHECK(ExistsDir(options_.image_path));
  CHECK_NOTNULL(reconstruction_manager_);

  option_manager_.AddAllOptions();

  *option_manager_.image_path = options_.image_path;
  *option_manager_.database_path =
      JoinPaths(options_.workspace_path, "database.db");

  if (options_.data_type == DataType::VIDEO) {
    option_manager_.ModifyForVideoData();
  } else if (options_.data_type == DataType::INDIVIDUAL) {
    option_manager_.ModifyForIndividualData();
  } else if (options_.data_type == DataType::INTERNET) {
    option_manager_.ModifyForInternetData();
  } else {
    LOG(FATAL) << "Data type not supported";
  }

  CHECK(ExistsCameraModelWithName(options_.camera_model));

  if (options_.quality == Quality::LOW) {
    option_manager_.ModifyForLowQuality();
  } else if (options_.quality == Quality::MEDIUM) {
    option_manager_.ModifyForMediumQuality();
  } else if (options_.quality == Quality::HIGH) {
    option_manager_.ModifyForHighQuality();
  } else if (options_.quality == Quality::EXTREME) {
    option_manager_.ModifyForExtremeQuality();
  }

  option_manager_.sift_extraction->num_threads = options_.num_threads;
  option_manager_.sift_matching->num_threads = options_.num_threads;
  option_manager_.mapper->num_threads = options_.num_threads;

  ImageReaderOptions reader_options = *option_manager_.image_reader;
  reader_options.database_path = *option_manager_.database_path;
  reader_options.image_path = *option_manager_.image_path;
  if (!options_.mask_path.empty()) {
    reader_options.mask_path = options_.mask_path;
    option_manager_.image_reader->mask_path = options_.mask_path;
  }
  reader_options.single_camera = options_.single_camera;
  reader_options.camera_model = options_.camera_model;

  option_manager_.sift_extraction->use_gpu = options_.use_gpu;
  option_manager_.sift_matching->use_gpu = options_.use_gpu;

  option_manager_.sift_extraction->gpu_index = options_.gpu_index;
  option_manager_.sift_matching->gpu_index = options_.gpu_index;

  feature_extractor_.reset(new SiftFeatureExtractor(
      reader_options, *option_manager_.sift_extraction));

  exhaustive_matcher_.reset(new ExhaustiveFeatureMatcher(
      *option_manager_.exhaustive_matching, *option_manager_.sift_matching,
      *option_manager_.database_path));

  if (!options_.vocab_tree_path.empty()) {
    option_manager_.sequential_matching->loop_detection = true;
    option_manager_.sequential_matching->vocab_tree_path =
        options_.vocab_tree_path;
  }

  sequential_matcher_.reset(new SequentialFeatureMatcher(
      *option_manager_.sequential_matching, *option_manager_.sift_matching,
      *option_manager_.database_path));

  if (!options_.vocab_tree_path.empty()) {
    option_manager_.vocab_tree_matching->vocab_tree_path =
        options_.vocab_tree_path;
    vocab_tree_matcher_.reset(new VocabTreeFeatureMatcher(
        *option_manager_.vocab_tree_matching, *option_manager_.sift_matching,
        *option_manager_.database_path));
  }
}

void AutomaticReconstructionController::Stop() {
  if (active_thread_ != nullptr) {
    active_thread_->Stop();
  }
  Thread::Stop();
}

void AutomaticReconstructionController::Run() {
  if (IsStopped()) {
    return;
  }

  RunFeatureExtraction();

  if (IsStopped()) {
    return;
  }

  RunFeatureMatching();

  if (IsStopped()) {
    return;
  }

  if (options_.sparse) {
    RunSparseMapper();
  }

  if (IsStopped()) {
    return;
  }
}

void AutomaticReconstructionController::RunFeatureExtraction() {
  CHECK(feature_extractor_);
  active_thread_ = feature_extractor_.get();
  feature_extractor_->Start();
  feature_extractor_->Wait();
  feature_extractor_.reset();
  active_thread_ = nullptr;
}

void AutomaticReconstructionController::RunFeatureMatching() {
  Thread* matcher = nullptr;
  if (options_.data_type == DataType::VIDEO) {
    matcher = sequential_matcher_.get();
  } else if (options_.data_type == DataType::INDIVIDUAL ||
             options_.data_type == DataType::INTERNET) {
    Database database(*option_manager_.database_path);
    const size_t num_images = database.NumImages();
    if (options_.vocab_tree_path.empty() || num_images < 200) {
      matcher = exhaustive_matcher_.get();
    } else {
      matcher = vocab_tree_matcher_.get();
    }
  }

  CHECK(matcher);
  active_thread_ = matcher;
  matcher->Start();
  matcher->Wait();
  exhaustive_matcher_.reset();
  sequential_matcher_.reset();
  vocab_tree_matcher_.reset();
  active_thread_ = nullptr;
}

void AutomaticReconstructionController::RunSparseMapper() {
  const auto sparse_path = JoinPaths(options_.workspace_path, "sparse");
  if (ExistsDir(sparse_path)) {
    auto dir_list = GetDirList(sparse_path);
    std::sort(dir_list.begin(), dir_list.end());
    if (dir_list.size() > 0) {
      std::cout << std::endl
                << "WARNING: Skipping sparse reconstruction because it is "
                   "already computed"
                << std::endl;
      for (const auto& dir : dir_list) {
        reconstruction_manager_->Read(dir);
      }
      return;
    }
  }

  IncrementalMapperController mapper(
      option_manager_.mapper.get(), *option_manager_.image_path,
      *option_manager_.database_path, reconstruction_manager_);
  active_thread_ = &mapper;
  mapper.Start();
  mapper.Wait();
  active_thread_ = nullptr;

  CreateDirIfNotExists(sparse_path);
  reconstruction_manager_->Write(sparse_path, &option_manager_);
}

}  // namespace colmap

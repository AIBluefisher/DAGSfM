// Copyright (c) 2018 Yu Chen

#pragma once

#include "i23dSFM/sfm/pipelines/sfm_engine.hpp"
#include "i23dSFM/sfm/pipelines/global/GlobalSfM_rotation_averaging.hpp"
#include "i23dSFM/sfm/pipelines/global/GlobalSfM_translation_averaging.hpp"

#include "i23dSFM/sfm/pipelines/sfm_features_provider.hpp"
#include "i23dSFM/sfm/pipelines/sfm_matches_provider.hpp"
#include "i23dSFM/tracks/tracks.hpp"

#include "third_party/htmlDoc/htmlDoc.hpp"
#include "third_party/histogram/histogram.hpp"

namespace i23dSFM {
namespace sfm {

class HybridSfMReconstructionEngine : public ReconstructionEngine
{
public:
    HybridSfMReconstructionEngine(const SfM_Data & sfm_data, 
                                                        const std::string & soutDirectory,
                                                        const std::string & loggingFile = "");

    ~HybridSfMReconstructionEngine();

    void SetFeaturesProvider(Features_Provider * provider);
    void SetMatchesProvider(Matches_Provider * provider);

    virtual bool Process();

    void setInitialPair(const Pair & initialPair)
    {
        _initialpair = initialPair;
    }

    /// Initialize tracks
    bool InitLandmarkTracks();

    /// Select a candidate initial pair
    bool ChooseInitialPair(Pair & initialPairIndex) const;

    /// Compute the initial 3D seed (First camera t=0; R=Id, second estimated by 5 point algorithm)
    bool MakeInitialPair3D(const Pair & initialPair);

    /// Automatic initial pair selection (based on a 'baseline' computation score)
    bool AutomaticInitialPairChoice(Pair & initialPair) const;

    /**
     * Set the default lens distortion type to use if it is declared unknown
     * in the intrinsics camera parameters by the previous steps.
     *
     * It can be declared unknown if the type cannot be deduced from the metadata.
     */
    void SetUnknownCameraType(const cameras::EINTRINSIC camType)
    {
        _camType = camType;
    }

    void SetRotationAveragingMethod(ERotationAveragingMethod eRotationAveragingMethod);
    void SetTranslationAveragingMethod(ETranslationAveragingMethod _eTranslationAveragingMethod);

protected:
    /// Compute from relative rotations the global rotations of the camera poses
    bool Compute_Global_Rotations
    (
        const i23dSFM::rotation_averaging::RelativeRotations & vec_relatives_R,
        Hash_Map<IndexT, Mat3> & map_globalR
    );

    /// Compute/refine relative translations and compute global translations
    bool Compute_Global_Translations
    (
        const Hash_Map<IndexT, Mat3> & global_rotations,
        matching::PairWiseMatches & tripletWise_matches
    );

    /// Compute the initial structure of the scene
    bool Compute_Initial_Structure
    (
        matching::PairWiseMatches & tripletWise_matches
    );

    // Adjust the scene (& remove outliers)
    bool Adjust();


private:

    /// Return MSE (Mean Square Error) and a histogram of residual values.
    double ComputeResidualsHistogram(Histogram<double> * histo);

    /// List the images that the greatest number of matches to the current 3D reconstruction.
    bool FindImagesWithPossibleResection(std::vector<size_t> & vec_possible_indexes);

    /// Add a single Image to the scene and triangulate new possible tracks.
    bool Resection(const size_t imageIndex);

    /// Bundle adjustment to refine Structure; Motion and Intrinsics
    bool BundleAdjustment();

    /// Discard track with too large residual error
    size_t badTrackRejector(double dPrecision, size_t count = 0);

        /// Compute relative rotations
    void Compute_Relative_Rotations
    (
        i23dSFM::rotation_averaging::RelativeRotations & vec_relatives_R
    );


    //----
    //-- Data
    //----
    private:

    // HTML logger
    std::shared_ptr<htmlDocument::htmlDocumentStream> _htmlDocStream;
    std::string _sLoggingFile;

    // Parameter
    Pair _initialpair;
    cameras::EINTRINSIC _camType; // The camera type for the unknown cameras

    //-- Data provider
    Features_Provider  * _features_provider;
    Matches_Provider  * _matches_provider;

    // Temporary data
    i23dSFM::tracks::STLMAPTracks _map_tracks; // putative landmark tracks (visibility per 3D point)
    Hash_Map<IndexT, double> _map_ACThreshold; // Per camera confidence (A contrario estimated threshold error)

    std::set<size_t> _set_remainingViewId;     // Remaining camera index that can be used for resection

    // Parameter
    ERotationAveragingMethod _eRotationAveragingMethod;
    ETranslationAveragingMethod _eTranslationAveragingMethod;

    std::shared_ptr<Features_Provider> _normalized_features_provider;
};



}   // namespace sfm
}   // namespace i23dSFM
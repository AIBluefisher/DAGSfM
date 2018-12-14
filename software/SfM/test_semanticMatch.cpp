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

#include "i23dSFM/image/image.hpp"
#include "i23dSFM/features/features.hpp"
#include "i23dSFM/matching/regions_matcher.hpp"

#include "nonFree/sift/SIFT_describer.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include "third_party/vectorGraphics/svgDrawer.hpp"

#include <string>
#include <iostream>

using namespace i23dSFM;
using namespace i23dSFM::image;
using namespace i23dSFM::matching;
using namespace svg;
using namespace std;

int main() 
{
  Image<RGBColor> image;
  string jpg_filenameL = stlplus::folder_up(string(THIS_SOURCE_DIR))
    + "/imageData/StanfordMobileVisualSearch/Ace_0.png";
  string jpg_filenameR = stlplus::folder_up(string(THIS_SOURCE_DIR))
    + "/imageData/StanfordMobileVisualSearch/Ace_1.png";

  Image<unsigned char> imageL, imageR;
  ReadImage(jpg_filenameL.c_str(), &imageL);
  ReadImage(jpg_filenameR.c_str(), &imageR);

  //--
  // Detect regions thanks to an image_describer
  //--
  using namespace i23dSFM::features;
  std::unique_ptr<Image_describer> image_describer(new SIFT_Image_describer);
  std::map<IndexT, std::unique_ptr<features::Regions> > regions_perImage;
  image_describer->Describe(imageL, regions_perImage[0]);
  image_describer->Describe(imageR, regions_perImage[1]);

  const PointFeatures featsL = regions_perImage.at(0)->GetRegionsPositions();
  const PointFeatures featsR = regions_perImage.at(1)->GetRegionsPositions();

  // Show both images side by side
  {
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);
    string out_filename = "00_images.jpg";
    WriteImage(out_filename.c_str(), concat);
  }

  const SIFT_Regions* regionsL = dynamic_cast<SIFT_Regions*>(regions_perImage.at(0).get());
  const SIFT_Regions* regionsR = dynamic_cast<SIFT_Regions*>(regions_perImage.at(1).get());

  //- Draw features on the two image (side by side)
  {
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);

    //-- Draw features :
    for (size_t i=0; i < featsL.size(); ++i )  
    {
      const SIOPointFeature point = regionsL->Features()[i];
      DrawCircle(point.x(), point.y(), point.scale(), 255, &concat);
    }

    for (size_t i=0; i < featsR.size(); ++i )  
    {
      const SIOPointFeature point = regionsR->Features()[i];
      DrawCircle(point.x()+imageL.Width(), point.y(), point.scale(), 255, &concat);
    }
    const std::string out_filename = "01_features.jpg";
    WriteImage(out_filename.c_str(), concat);
  }

  //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
  std::vector<IndMatch> vec_PutativeMatches;
  {
    // Find corresponding points
    matching::DistanceRatioMatch(0.8, matching::BRUTE_FORCE_L2, *regions_perImage.at(0).get(),
                                                      *regions_perImage.at(1).get(), vec_PutativeMatches);

    // Draw correspondences after Nearest Neighbor ratio filter
    svgDrawer svgStream( imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));
    svgStream.drawImage(jpg_filenameL, imageL.Width(), imageL.Height());
    svgStream.drawImage(jpg_filenameR, imageR.Width(), imageR.Height(), imageL.Width());

    for (size_t i = 0; i < vec_PutativeMatches.size(); ++i) 
    {
      //Get back linked feature, draw a circle and link them by a line
      const SIOPointFeature L = regionsL->Features()[vec_PutativeMatches[i]._i];
      const SIOPointFeature R = regionsR->Features()[vec_PutativeMatches[i]._j];
      svgStream.drawLine(L.x(), L.y(), R.x()+imageL.Width(), R.y(), svgStyle().stroke("green", 2.0));
      svgStream.drawCircle(L.x(), L.y(), L.scale(), svgStyle().stroke("yellow", 2.0));
      svgStream.drawCircle(R.x()+imageL.Width(), R.y(), R.scale(),svgStyle().stroke("yellow", 2.0));
    }
    string out_filename = "02_siftMatches.svg";
    ofstream svgFile( out_filename.c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }

  // Display some statistics
  std::cout << featsL.size() << " Features on image A" << std::endl
   << featsR.size() << " Features on image B" << std::endl
   << vec_PutativeMatches.size() << " matches after matching with Distance Ratio filter" << std::endl;

  return EXIT_SUCCESS;
}

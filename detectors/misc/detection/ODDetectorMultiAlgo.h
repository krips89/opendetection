/*
Copyright (c) 2015, Kripasindhu Sarkar
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*///
// Created by sarkar on 06.08.15.
//

#ifndef OPENDETECTION_ODDETECTORMULTIALGO_H
#define OPENDETECTION_ODDETECTORMULTIALGO_H

#include <common/pipeline/ODDetector.h>
#include <common/pipeline/ODDetection.h>

namespace od
{

  /** \brief Class for running multiple Detection algorithms (with default parameters) on the same scene.
   *
   * Using this class one can do object detection/recognition using multiple algorithms and provide outcome of detections (eg. people detected by HOG, face detected by Cascade, bottle detected PnPRansac in the same image).
  *
  * \author Kripasindhu Sarkar
  *
  */
  class ODDetectorMultiAlgo2D : public ODDetector2D
  {
  public:
    ODDetectorMultiAlgo2D(std::string const &training_data_location_) : ODDetector2D(training_data_location_)
    { }


    typedef pcl::PointXYZRGBA PointT;



    ODDetections *detect(ODSceneImage *scene) ;
    ODDetections2D *detectOmni(ODSceneImage *scene);

    ODDetections* detect(ODScenePointCloud<PointT> *scene);
    ODDetections3D* detectOmni(ODScenePointCloud<PointT> *scene);


    void init();

  private:
    std::vector<ODDetector2D *> detectors_2d_;
    std::vector<ODDetector3D<PointT> *> detectors_3d_;
  };

  class ODDetectorMultiAlgo : public ODDetector
  {
  public:
    ODDetectorMultiAlgo(std::string const &training_data_location_) : ODDetector(training_data_location_)
    { }


    typedef pcl::PointXYZRGBA PointT;



    ODDetections *detect(ODSceneImage *scene) ;
    ODDetections2D *detectOmni(ODSceneImage *scene);

    ODDetections* detect(ODScenePointCloud<PointT> *scene);
    ODDetections3D* detectOmni(ODScenePointCloud<PointT> *scene);


    void init();

  private:
    std::vector<ODDetector2D *> detectors_2d_;
    std::vector<ODDetector3D<PointT> *> detectors_3d_;
  };

  /** \example objectdetector/od_multialgo_files.cpp
    * \example objectdetector/od_multialgo_pc.cpp
    */

}
#endif //OPENDETECTION_ODDETECTORMULTIALGO_H

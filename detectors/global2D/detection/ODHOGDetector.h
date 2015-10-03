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
// Created by sarkar on 15.07.15.
//

#ifndef OPENDETECTION_ODHOGMULTISCALEDETECTOR_H
#define OPENDETECTION_ODHOGMULTISCALEDETECTOR_H

#include "common/pipeline/ODDetector.h"
#include "common/pipeline/ODScene.h"
#include "common/utils/utils.h"
#include "common/utils/ODFeatureDetector2D.h"

#include <iostream>
#include <opencv2/opencv.hpp>

namespace od
{

  namespace g2d
  {
    /** \brief A linear classifier for HOG features.
     *
     * This class takes an image as an input, finds its HOG features using the parameters containing in the trained data, runs the linear classifier (present in the trained data) on the computed HOG features
     * and finally produces the classification output. Covers both multiscale detection (detectOmni()) and detection on a fixed scene (detect() - the scene is resized to the HOG window size).
     *
     * Use ODHOGTrainer for the training of new objects. By default this class provides two people detector: OD_DEFAULT_PEOPLE, the people detector from OpenCV and OD_DAIMLER_PEOPLE, the second detector available from OpenCV.
     * One can set any linear classifier, the linear weight vector through setSVMDetector() function but it is highly not recommended as the HOG parameters and the weight vector can be out of sync.
     * One must set appropriate HOG parameters after using setSVMDetector() function with which the weight vector was trained.
     *
     * \author Kripasindhu Sarkar
     *
     */


    class ODHOGDetector : public ODDetector2D
    {
    public:

      OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(SVMType, (OD_CUSTOM)(OD_DEFAULT_PEOPLE)(OD_DAIMLER_PEOPLE)(OD_FILE))


      ODHOGDetector(std::string const &trained_data_location_ = "", cv::Size winsize = cv::Size(64,128), cv::Size blocksize = cv::Size(16,16), cv::Size blockstride = cv::Size(8,8), cv::Size cellsize = cv::Size(8,8), float hitshreshold = 0.0):
                                                                      ODDetector2D(trained_data_location_),  winSize(winsize), blockSize(blocksize), blockStride(blockstride),
                                                                      cellSize(cellsize), hitThreshold(hitshreshold), hog_(winSize, blockSize, blockStride, cellSize, 9, 1, -1,
                                                                                                                           cv::HOGDescriptor::L2Hys, 0.2, false, cv::HOGDescriptor::DEFAULT_NLEVELS)
      {
        TRAINED_LOCATION_DENTIFIER_ = "HOG";
        TRAINED_DATA_ID_ = "hog.xml";
        metainfo_ = true;
        svmtype_ = OD_DEFAULT_PEOPLE;

        if (trained_data_location_ != "")
          svmtype_ = OD_FILE;
      }


      void init();
      void load(std::string filename);

      void setSVMFromFile(std::string fileName);

      void setSVMDetector(std::vector<float> svmdetector)
      {
        hog_.setSVMDetector(svmdetector);
      }

      ODDetections2D *detectOmni(ODSceneImage *scene);
      ODDetections *detect(ODSceneImage *scene);

      int detect(ODScene *scene, std::vector<ODDetection *> &detections)
      { }


      void setTrainedDataLocation(std::string trained_data_location_)
      {
        this->trained_data_location_ = trained_data_location_;
        this->svmtype_ = OD_FILE;
      }

      SVMType const &getSvmtype() const
      {
        return svmtype_;
      }

      void setSvmtype(SVMType const &svmtype_)
      {
        ODHOGDetector::svmtype_ = svmtype_;
      }

      cv::Size const &getWinSize() const
      {
        return winSize;
      }

      void setWinSize(cv::Size const &winSize)
      {
        ODHOGDetector::winSize = winSize;
      }

      cv::Size const &getBlockSize() const
      {
        return blockSize;
      }

      void setBlockSize(cv::Size const &blockSize)
      {
        ODHOGDetector::blockSize = blockSize;
      }

      cv::Size const &getBlockStride() const
      {
        return blockStride;
      }

      void setBlockStride(cv::Size const &blockStride)
      {
        ODHOGDetector::blockStride = blockStride;
      }

      cv::Size const &getCellSize() const
      {
        return cellSize;
      }

      void setCellSize(cv::Size const &cellSize)
      {
        ODHOGDetector::cellSize = cellSize;
      }

      float getHitThreshold() const
      {
        return hitThreshold;
      }

      void setHitThreshold(float hitThreshold)
      {
        ODHOGDetector::hitThreshold = hitThreshold;
      }

      void printParameters();

    protected:
      //properteis
      cv::Size winSize;
      cv::Size blockSize;
      cv::Size blockStride;
      cv::Size cellSize;

      float hitThreshold;

      cv::HOGDescriptor hog_;
      SVMType svmtype_;

    };

  /** \example objectdetector/od_image_hog.cpp
  *   \example objectdetector/od_image_hog_files.cpp
  *   \example apps/global2D/od_multihog_app.cpp
  *   This is an example of how to use the ODHOGDetector class.
  *
  *   More details about this example.
  */
  }
}


#endif //OPENDETECTION_ODHOGMULTISCALEDETECTOR_H
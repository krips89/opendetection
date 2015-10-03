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
// Created by sarkar on 16.06.15.
//

#ifndef OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H
#define OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H

#include <common/pipeline/ODDetector.h>
#include <common/pipeline/ODTrainer.h>
#include <common/utils/utils.h>

#include "opencv2/core.hpp"
#include "opencv2/face.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"



namespace od
{
  namespace g2d
  {


    /** \brief A facerecognizer based on EigenFace and FischerFace algorithms.
     *
     * Currently it just supports detection on fixed scene (detect()) and does not support multiscale detection. This is due to the fact that class for cascade classifier -
     * ODCascadeDetector supports multiscale detection and can be easily integrated with this recognizer - first by finding face using the Cascade and then applying this recognizer on that detected window.
     * This is faster than trying to perform recognition on each multiscale window.
     *
     * \author Kripasindhu Sarkar
     *
     */

    class ODFaceRecognizer : public ObjectDetector
    {
    public:

      OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(FaceRecogType, (OD_FACE_FISCHER)(OD_FACE_EIGEN))

      ODFaceRecognizer(FaceRecogType recogtype = OD_FACE_EIGEN, int num_components = 0, double threshold = DBL_MAX)
          : recogtype_(recogtype), num_components_(num_components), threshold_(threshold), im_height_(0), im_width_(0)
      {
        TRAINED_DATA_IDENTIFIER_ = "FACERECOG";
        TRAINED_DATA_EXT_ = "facerec.xml";
      }

      void init();

      void initTrainer();

      void initDetector();

      int train();

      ODDetections *detect(ODSceneImage *scene);


      FaceRecogType const &getRecogtype() const
      {
        return recogtype_;
      }

      void setRecogtype(FaceRecogType const &recogtype_)
      {
        ODFaceRecognizer::recogtype_ = recogtype_;
      }

      int getThreshold() const
      {
        return threshold_;
      }

      void setThreshold(int threshold_)
      {
        ODFaceRecognizer::threshold_ = threshold_;
      }

      int getNumComponents() const
      {
        return num_components_;
      }

      void setNumComponents(int num_components_)
      {
        ODFaceRecognizer::num_components_ = num_components_;
      }

    protected:
      cv::Ptr<cv::face::FaceRecognizer> cvrecognizer_;
      FaceRecogType recogtype_;

      int im_width_;
      int im_height_;
      int num_components_;
      double threshold_;


    private:
      static void read_csv(const std::string &filename, std::vector<cv::Mat> &images, std::vector<int> &labels, char separator = ';');

    };
    /** \example objectdetector/od_image_facerecog.cpp
      */
  }
}
#endif //OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H

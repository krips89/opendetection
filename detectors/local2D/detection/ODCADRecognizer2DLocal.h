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
*/
//
// Created by sarkar on 08.06.15.
//

#ifndef OPENDETECTION_SIMPLERANSACDETECTOR_H
#define OPENDETECTION_SIMPLERANSACDETECTOR_H

#include <detectors/local2D/ODImageLocalMatching.h>

#include "common/pipeline/ODDetector.h"
#include "common/pipeline/ODScene.h"
#include "common/utils/utils.h"
#include "common/utils/ODFeatureDetector2D.h"

#include <iostream>
#include <time.h>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>



#include "simple_ransac_detection/Utils.h"


class Model;
class PnpProblem;

namespace od
{

  namespace l2d
  {

    /** \brief Simple RANSAC based 3D object recognizer.
     *
     * A recognizer which uses local features like SIFT/SURF to perform object recognition.
     * Given a 'trained model' trained by ODCADRecogTrainerSnapshotBased or trained externally (manually augmenting features in 3D cad models), this class performs a complete detection
     * in an image. It first extracts 2D features from the scene, matches them with all the feature augmented models (the trained data) and in the end solves PnP under RANSAC.
     *
     * \author Kripasindhu Sarkar
     *
     */
    class ODCADRecognizer2DLocal : public ODImageLocalMatchingDetector
    {
    public:
      string const &getCameraIntrinsicFile() const
      {
        return camera_intrinsic_file;
      }

      void setCameraIntrinsicFile(string const &camera_intrinsic_file)
      {
        ODCADRecognizer2DLocal::camera_intrinsic_file = camera_intrinsic_file;
      }

      int getNumKeyPoints() const
      {
        return numKeyPoints;
      }

      void setNumKeyPoints(int numKeyPoints)
      {
        ODCADRecognizer2DLocal::numKeyPoints = numKeyPoints;
      }

      float getRatioTest() const
      {
        return ratioTest;
      }

      void setRatioTest(float ratioTest)
      {
        ODCADRecognizer2DLocal::ratioTest = ratioTest;
      }

      bool isFast_match() const
      {
        return fast_match;
      }

      void setFast_match(bool fast_match)
      {
        ODCADRecognizer2DLocal::fast_match = fast_match;
      }

      bool isUse_gpu() const
      {
        return use_gpu;
      }

      void setUse_gpu(bool use_gpu)
      {
        ODCADRecognizer2DLocal::use_gpu = use_gpu;
      }

      bool isUse_gpu_match() const
      {
        return use_gpu_match;
      }

      void setUse_gpu_match(bool use_gpu_match)
      {
        ODCADRecognizer2DLocal::use_gpu_match = use_gpu_match;
      }

      bool isMetainfo() const
      {
        return metainfo_;
      }

      void setMetainfo(bool metainfo)
      {
        ODCADRecognizer2DLocal::metainfo_ = metainfo;
      }

      int getIterationsCount() const
      {
        return iterationsCount;
      }

      void setIterationsCount(int iterationsCount)
      {
        ODCADRecognizer2DLocal::iterationsCount = iterationsCount;
      }

      float getReprojectionError() const
      {
        return reprojectionError;
      }

      void setReprojectionError(float reprojectionError)
      {
        ODCADRecognizer2DLocal::reprojectionError = reprojectionError;
      }

      double getConfidence() const
      {
        return confidence;
      }

      void setConfidence(double confidence)
      {
        ODCADRecognizer2DLocal::confidence = confidence;
      }

      int getMinInliers() const
      {
        return minInliers;
      }

      void setMinInliers(int minInliers)
      {
        ODCADRecognizer2DLocal::minInliers = minInliers;
      }

      int getPnpMethod() const
      {
        return pnpMethod;
      }

      void setPnpMethod(int pnpMethod)
      {
        ODCADRecognizer2DLocal::pnpMethod = pnpMethod;
      }

      ODCADRecognizer2DLocal(string const &trained_data_location_ = 0) : ODImageLocalMatchingDetector(
          trained_data_location_)
      {
        metainfo_ = true;

        camera_intrinsic_file = "Data/out_camera_data_lion_old.yml";         // mesh

        red = cv::Scalar(0, 0, 255);
        green = cv::Scalar(0, 255, 0);
        blue = cv::Scalar(255, 0, 0);
        yellow = cv::Scalar(0, 255, 255);


        numKeyPoints = 2000;      // number of detected keypoints
        ratioTest = 0.70f;          // ratio test
        fast_match = true;       // fastRobustMatch() or robustMatch()
        use_gpu = false;
        use_gpu_match = false;

        iterationsCount = 500;      // number of Ransac iterations.
        reprojectionError = 2.0;  // maximum allowed distance to consider it an inlier.
        confidence = 0.95;        // ransac successful confidence.

        minInliers = 30;    // Kalman threshold updating

        pnpMethod = cv::SOLVEPNP_EPNP;
        f_type_default = "SIFT";
        featureDetector = boost::make_shared<ODFeatureDetector2D>(f_type_default, use_gpu);
      }

      void parseParameterString(string parameter_string);

      void init();

      ODDetections* detect(ODSceneImage *scene);

      ODDetections3D* detectOmni(ODSceneImage *scene);

    protected:

      string camera_intrinsic_file;         // mesh

      cv::Scalar red;
      cv::Scalar green;
      cv::Scalar blue;
      cv::Scalar yellow;

// Robust Matcher parameters
      int numKeyPoints;      // number of detected keypoints
      float ratioTest;          // ratio test
      bool fast_match;       // fastRobustMatch() or robustMatch()
      bool use_gpu;
      bool use_gpu_match;

// RANSAC parameters
      int iterationsCount;      // number of Ransac iterations.
      float reprojectionError;  // maximum allowed distance to consider it an inlier.
      double confidence;        // ransac successful confidence.

// Kalman Filter parameters
      int minInliers;    // Kalman threshold updating

// PnP parameters
      int pnpMethod;

      //############NON-CONFIG PARAMETERS used for detection###########
      vector<string> model_names;
      vector<Model> models;
      PnPProblem pnp_detection;
      std::string f_type_default;
      boost::shared_ptr<ODFeatureDetector2D> featureDetector;

      bool detectSingleModel(ODSceneImage *scene, Model const &model, ODDetection3D *&pD, cv::Mat &frame_viz);
    };
    /** \example objectdetector/od_image_cadrecog_camera.cpp
     * \example objectdetector/od_image_cadrecog_files.cpp
     */
  }
}

#endif //OPENDETECTION_SIMPLERANSACDETECTOR_H

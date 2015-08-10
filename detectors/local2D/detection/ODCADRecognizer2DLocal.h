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
    //--model=Data/Chicken/Mesh.xml --mesh=Data/Chicken/Mesh.ply
    // --use_gpu --fast --method=1 --error=2 --confidence=0.9 --iterations=500 --inliers=30 --model=Data/Chicken/Mesh.xml --mesh=Data/Chicken/Mesh.ply --test_images=./Data/Chicken/Chicken/*.JPG --camera_intrinsic_file=Data/out_camera_dataset_101.yml
    // --use_gpu --fast --method=1 --error=2 --confidence=0.9 --iterations=500 --inliers=30 --test_images=./Data/Lion/test_images/IMG_*.JPG --camera_intrinsic_file=Data/out_camera_data_lion_old.yml
    //  --fast --method=1 --error=2 --confidence=0.9 --iterations=500 --inliers=30 --test_images=./Data/Lion/Lion2/IMG_*.JPG --camera_intrinsic_file=Data/out_camera_dataset_101.yml
    //--fast --method=1 --error=2 --confidence=0.9 --iterations=500 --inliers=10 --model=Data/Totem/Param.SIFT.xml --mesh=Data/Totem/Param.ply --test_images=./Data/Totem/Totem/IMG_0251.JPG --camera_intrinsic_file=Data/out_camera_dataset_101.yml

    /** \brief Simple ransac based 3D object detector; the details will be explained later
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
        return metainfo;
      }

      void setMetainfo(bool metainfo)
      {
        ODCADRecognizer2DLocal::metainfo = metainfo;
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

      ODCADRecognizer2DLocal(string const &training_data_location_ = 0) : ODImageLocalMatchingDetector(
          training_data_location_)
      {
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
      bool metainfo;

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

  }
}

#endif //OPENDETECTION_SIMPLERANSACDETECTOR_H

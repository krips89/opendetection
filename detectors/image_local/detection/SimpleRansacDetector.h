//
// Created by sarkar on 08.06.15.
//

#ifndef OPENDETECTION_SIMPLERANSACDETECTOR_H
#define OPENDETECTION_SIMPLERANSACDETECTOR_H

#include <iostream>
#include <detectors/image_local/ODImageLocalMatching.h>

#include "common/pipeline/ODDetector.h"
#include "common/pipeline/ODScene.h"

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
  class SimpleRansacDetector : public ODImageLocalMatchingDetector
  {
  public:
    string const &getCameraIntrinsicFile() const
    {
      return camera_intrinsic_file;
    }

    void setCameraIntrinsicFile(string const &camera_intrinsic_file)
    {
      SimpleRansacDetector::camera_intrinsic_file = camera_intrinsic_file;
    }

    int getNumKeyPoints() const
    {
      return numKeyPoints;
    }

    void setNumKeyPoints(int numKeyPoints)
    {
      SimpleRansacDetector::numKeyPoints = numKeyPoints;
    }

    float getRatioTest() const
    {
      return ratioTest;
    }

    void setRatioTest(float ratioTest)
    {
      SimpleRansacDetector::ratioTest = ratioTest;
    }

    bool isFast_match() const
    {
      return fast_match;
    }

    void setFast_match(bool fast_match)
    {
      SimpleRansacDetector::fast_match = fast_match;
    }

    bool isUse_gpu() const
    {
      return use_gpu;
    }

    void setUse_gpu(bool use_gpu)
    {
      SimpleRansacDetector::use_gpu = use_gpu;
    }

    bool isUse_gpu_match() const
    {
      return use_gpu_match;
    }

    void setUse_gpu_match(bool use_gpu_match)
    {
      SimpleRansacDetector::use_gpu_match = use_gpu_match;
    }

    bool isMetainfo() const
    {
      return metainfo;
    }

    void setMetainfo(bool metainfo)
    {
      SimpleRansacDetector::metainfo = metainfo;
    }

    int getIterationsCount() const
    {
      return iterationsCount;
    }

    void setIterationsCount(int iterationsCount)
    {
      SimpleRansacDetector::iterationsCount = iterationsCount;
    }

    float getReprojectionError() const
    {
      return reprojectionError;
    }

    void setReprojectionError(float reprojectionError)
    {
      SimpleRansacDetector::reprojectionError = reprojectionError;
    }

    double getConfidence() const
    {
      return confidence;
    }

    void setConfidence(double confidence)
    {
      SimpleRansacDetector::confidence = confidence;
    }

    int getMinInliers() const
    {
      return minInliers;
    }

    void setMinInliers(int minInliers)
    {
      SimpleRansacDetector::minInliers = minInliers;
    }

    int getPnpMethod() const
    {
      return pnpMethod;
    }

    void setPnpMethod(int pnpMethod)
    {
      SimpleRansacDetector::pnpMethod = pnpMethod;
    }

    SimpleRansacDetector(string const &training_data_location_ = 0) : ODImageLocalMatchingDetector(
        training_data_location_)
    {
      camera_intrinsic_file = "Data/out_camera_data_lion_old.yml";         // mesh

      red = cv::Scalar(0, 0, 255);
      green = cv::Scalar(0,255,0);
      blue = cv::Scalar(255,0,0);
      yellow = cv::Scalar(0,255,255);


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

    }

    void parseParameterString(string parameter_string);

    void init();

    int detect(ODScene *scene, vector<ODDetection *> &detections);
    int detect(ODSceneImage *scene, vector<ODDetection3D *> &detections);

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

    bool detectSingleModel(ODSceneImage *scene, Model const &model, ODDetection3D * &pD);
  };





  class FrameGenerator
  {
  public:
    FrameGenerator(cv::CommandLineParser parser)
    {
      //some hardcoded default values (TEST ONE IMAGES), the program will run even without any arguments
      cameraID = 0;
      curr_image = -1;
      string test_images = "Data/Lion/test_images/IMG_20150226_102852.jpg";
      image_list = myglob(test_images);

      inputType = IMAGE_LIST;

      exhausted = false;
      cout << "yo " << parser.get<int>("cam_id") << endl;

      //parsing input from the arguments now
      if(parser.get<string>("test_images").size() > 0) {
        test_images = parser.get<string>("test_images");
        inputType = IMAGE_LIST;
        image_list = myglob(test_images);
      }
      if(parser.get<string>("video").size() > 0) {
        video_read_path = parser.get<string>("video");
        inputType = VIDEO_FILE;
        inputCapture.open(video_read_path);
        if(!inputCapture.isOpened()) {
          cout << "FATAL: Cannot open video capture!";
          exhausted = true;
        }
      }
      if(parser.has("cam_id"))      //MAX PREFERENCE
      {
        cameraID = !parser.get<int>("cam_id") ? parser.get<int>("cam_id") : cameraID;
        inputType = CAMERA;
        inputCapture.open(cameraID);
        if(!inputCapture.isOpened()) {
          cout << "FATAL: Cannot open video capture!";
          exhausted = true;
        }
      }


    }

    cv::Mat getNextFrame()
    {
      cv::Mat result;
      if(inputType == IMAGE_LIST) {
        curr_image++;

        //sleep(3); //wait for 3 seconds

        if(curr_image == image_list.size() - 1)
          exhausted = true;
        cout << "Frame: " << image_list[curr_image] << endl;


//      KFeatureDetector fd("SIFT", true);
//      cv::Mat descriptors; vector<KeyPoint> kps;
        //fd.findSiftGPUDescriptors(image_list[curr_image].c_str(), descriptors, kps);

        result = cv::imread(image_list[curr_image], cv::IMREAD_COLOR);
//      Mat result1 = imread(image_list[curr_image], IMREAD_GRAYSCALE);
//      Mat temp;
//      cv::cvtColor(result, temp, cv::COLOR_BGR2GRAY);

        //2
//      fd.findSiftGPUDescriptors(result1, descriptors, kps );
//      fd.findSiftGPUDescriptors(temp, descriptors, kps );
//      fd.findSiftGPUDescriptors(result, descriptors, kps );

      } else {
        if(inputCapture.isOpened()) {
          //as real time as possible
          inputCapture.read(result);
        } else exhausted = false;
      }

      return result;
    }

  public:
    enum InputType
    {
      INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST
    };

    string input;

    int cameraID;
    vector<string> image_list;
    string video_read_path;
    int curr_image;
    cv::VideoCapture inputCapture;

    InputType inputType;
    bool exhausted;


  private:
    string patternToUse;


  };



}

#endif //OPENDETECTION_SIMPLERANSACDETECTOR_H

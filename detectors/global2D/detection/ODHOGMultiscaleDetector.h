//
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
    /** \brief ODHOGMultiscaleDetector: A linear classifier detector for HOG features
   * User can set any linear classifier through setSVMDetector function or one of the default available detectors like OD_DEFAULT_PEOPLE
   * This class will then find HOG features in the scene and use the liniar classification model as an input to classify the scene
   * It covers both simple region detection and omni detection (multiscale mitilocation detection on the entire scene)
   *
   *
   * \author Kripasindhu Sarkar
   *
   */
    class ODHOGMultiscaleDetector : ODDetector
    {
    public:

      OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(SVMType, (OD_CUSTOM)(OD_DEFAULT_PEOPLE)(OD_DAIMLER_PEOPLE))

      ODHOGMultiscaleDetector()
      {
        metainfo_ = true;
        svmtype_ = OD_DEFAULT_PEOPLE;
      }

      void init()
      {
        switch(svmtype_)
        {
          case OD_DEFAULT_PEOPLE:
            hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
            break;
          case OD_DAIMLER_PEOPLE:
            hog_.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
            break;
            //dont set anything for custom, it is to be set by the user
        }
      }

      void setSVMDetector(std::vector<float> svmdetector)
      {
        hog_.setSVMDetector(svmdetector);
      }

      ODDetections2D *detectOmni(ODSceneImage *scene);

      int detect(ODScene *scene, vector<ODDetection *> &detections)
      { }

    private:
      cv::HOGDescriptor hog_;
      SVMType svmtype_;
    };

  }
}
#endif //OPENDETECTION_ODHOGMULTISCALEDETECTOR_H
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

  /** \brief ODHOGMultiscaleDetector: A linear classifier detector for HOG features
   * User can set any linear classifier through setSVMDetector function or one of the default available detectors like OD_DEFAULT_PEOPLE
   * This class will then find HOG features in the scene and use the liniar classification model as an input to classify the scene
   * It covers both simple region detection and omni detection (multiscale mitilocation detection on the entire scene)
   *
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODHOGMultiscaleDetector: ODDetector
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
      switch (svmtype_)
      {
        case OD_DEFAULT_PEOPLE: hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector()); break;
        case OD_DAIMLER_PEOPLE: hog_.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector()); break;
          //dont set anything for custom, it is to be set by the user
      }
    }

    void setSVMDetector(std::vector<float> svmdetector)
    {
      hog_.setSVMDetector(svmdetector);
    }

    ODDetections2D* detect(ODSceneImage *scene)
    {

      vector<cv::Rect> found, found_filtered;
      hog_.detectMultiScale(scene->getCVImage(), found, 0, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);

      size_t i, j;
      for(i = 0; i < found.size(); i++)
      {
        cv::Rect r = found[i];
        for(j = 0; j < found.size(); j++)
          if(j != i && (r & found[j]) == r)
            break;
        if(j == found.size())
          found_filtered.push_back(r);
      }

      //always create detections
      ODDetections2D * detections = new ODDetections2D;

      cv::Mat viz = scene->getCVImage().clone();
      for(i = 0; i < found_filtered.size(); i++)
      {

        ODDetection2D * detections2D = new ODDetection2D;
        detections2D->setBoundingBox(found_filtered[i]);
        detections2D->setId("PEOPLE");
        detections2D->setType(ODDetection::OD_DETECTION_CLASS);
        detections->push_back(detections2D);

        if(metainfo_)
        {
          cv::Rect r = found_filtered[i];
          r.x += cvRound(r.width * 0.1);
          r.width = cvRound(r.width * 0.8);
          r.y += cvRound(r.height * 0.06);
          r.height = cvRound(r.height * 0.9);
          rectangle(viz, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
        }
      }

      detections->setMetainfoImage(viz);

      return detections;
    }

    int detect(ODScene *scene, vector<ODDetection *> &detections) {}

  private:
    cv::HOGDescriptor hog_;
    SVMType svmtype_;
  };

}
#endif //OPENDETECTION_ODHOGMULTISCALEDETECTOR_H
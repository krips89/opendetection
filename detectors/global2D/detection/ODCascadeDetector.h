//
// Created by sarkar on 17.07.15.
//

#ifndef OPENDETECTION_ODCASCADEDETECTOR_H
#define OPENDETECTION_ODCASCADEDETECTOR_H

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
    /** \brief A class for detection using Cascade classifiers.
     * Given a scene and a cascade classifier, this class performs a classification and returns detections. The training is not supported in OD currently but is compatible to the cascade training of OpenCV.
     * Train your cascade classifiers using OpenCV's *opencv_traincascade* utility (http://docs.opencv.org/master/dc/d88/tutorial_traincascade.html#gsc.tab=0). It is a great tool for traning your cascade.
     * Paste the generated xml in training_data_location_/CASCADE/*.cascade.xml to use your trained cascade.
     *
     * \author Kripasindhu Sarkar
     *
     */

    class ODCascadeDetector : public ODDetector2D
    {
    public:

      ODCascadeDetector(std::string const &training_data_location_ = "", double scaleFactor = 1.1, int minNeighbors = 3, int flags = 0, cv::Size minSize = cv::Size(), cv::Size maxSize = cv::Size())
          : ODDetector2D(training_data_location_), scaleFactor_(scaleFactor), minNeighbors_(minNeighbors), minSize_(minSize), maxSize_(maxSize)
      {
        TRAINED_DATA_IDENTIFIER_ = "CASCADE";
        TRAINED_DATA_EXT_ = "cascade.xml";
        metainfo_ = true;
      }

      void init()
      {
        haar_cascade_ = boost::make_shared<cv::CascadeClassifier>(FileUtils::getFirstFile(getSpecificTrainingDataLocation(), TRAINED_DATA_EXT_));
      }

      ODDetections2D *detectOmni(ODSceneImage *scene);
      ODDetections* detect(ODSceneImage *scene);

    private:
      boost::shared_ptr<cv::CascadeClassifier> haar_cascade_;

      double scaleFactor_;
      int minNeighbors_;
      cv::Size minSize_;
      cv::Size maxSize_;

    };
    /** \example objectdetector/od_image_cascade.cpp
     *  \example objectdetector/od_image_cascade_files.cpp
     */
  }

}

#endif //OPENDETECTION_ODCASCADEDETECTOR_H

//
// Created by sarkar on 08.06.15.
//

#ifndef OPENDETECTION_ODDETECTOR_H
#define OPENDETECTION_ODDETECTOR_H

#include "ObjectDetector.h"
#include "ODScene.h"
#include "ODDetection.h"
#include "ObjectDetector.h"
#include "ODAlgorithmBase.h"
#include <iostream>

namespace od
{
  /** \brief The main detection class; all Detector derives from this
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODDetector: public ODDetectorCommon
  {
  public:

    ODDetector(std::string const &training_data_location_) : ODDetectorCommon(training_data_location_)
    { }

    virtual ODDetections* detect(ODScene *scene){}

    bool metainfo_;

  };


  class ODDetector2D: public ODDetector
  {
  public:
    ODDetector2D(std::string const &training_data_location_) : ODDetector(training_data_location_)
    { }

    ODDetections* detect(ODScene *scene)
    {
      return detect(dynamic_cast<ODSceneImage *>(scene));
    }

    virtual ODDetections* detect(ODSceneImage *scene) = 0;
    virtual ODDetections2D* detectOmni(ODSceneImage *scene) = 0;
  };

  template<typename PointT = pcl::PointXYZRGBA>
  class ODDetector3D: public ODDetector
  {
  public:
    ODDetector3D(std::string const &training_data_location_) : ODDetector(training_data_location_)
    { }

    virtual ODDetections* detect(ODScenePointCloud<PointT> *scene) = 0;
    virtual ODDetections3D* detectOmni(ODScenePointCloud<PointT> *scene) = 0;
  };

  class ODDetector2DComplete: public ODDetector
  {
  public:
    ODDetector2DComplete(std::string const &training_data_location_) : ODDetector(training_data_location_)
    { }

    virtual ODDetections* detect(ODSceneImage *scene) = 0;
    virtual ODDetections3D* detectOmni(ODSceneImage *scene) = 0;
  };

}
#endif //OPENDETECTION_ODDETECTOR_H

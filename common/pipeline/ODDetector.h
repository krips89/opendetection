//
// Created by sarkar on 08.06.15.
//

#ifndef OPENDETECTION_ODDETECTOR_H
#define OPENDETECTION_ODDETECTOR_H

//#include "ObjectDetector.h"
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
  class ODDetector
  {
  public:

    ODDetector(std::string const &training_data_location_) : training_data_location_(training_data_location_)
    { }


    virtual ODDetection* detect(ODScene *scene) {}

    virtual void init() = 0;
    virtual ODDetections* detectOmni(ODScene *scene) {}

    std::string getTrainingInputLocation() const
    {
      return training_input_location_;
    }

    void setTrainingInputLocation(std::string training_input_location_)
    {
      this->training_input_location_ = training_input_location_;
    }

    std::string getTrainingDataLocation() const
    {
      return training_data_location_;
    }

    void setTrainingDataLocation(std::string training_data_location_)
    {
      this->training_data_location_ = training_data_location_;
    }

    string getSpecificTrainingDataLocation()
    {
      return training_data_location_ + "/" + "TD_" + TRAINED_DATA_IDENTIFIER_;
    }

    bool metainfo_;
  protected:
    std::string training_input_location_, training_data_location_;
    std::string TRAINED_DATA_EXT_, TRAINED_DATA_IDENTIFIER_;

  };


  class ODDetector2D: public ODDetector
  {
  public:
    ODDetector2D(std::string const &training_data_location_) : ODDetector(training_data_location_)
    { }

    virtual ODDetections* detect(ODSceneImage *scene) {}
    virtual ODDetections2D* detectOmni(ODSceneImage *scene) {}
  };

  template<typename PointT = pcl::PointXYZRGBA>
  class ODDetector3D: public ODDetector
  {
  public:
    ODDetector3D(std::string const &training_data_location_) : ODDetector(training_data_location_)
    { }

    virtual ODDetections* detect(ODScenePointCloud<PointT> *scene) {}
    virtual ODDetections3D* detectOmni(ODScenePointCloud<PointT> *scene) {}
  };

  class ODDetector2DComplete: public ODDetector
  {
  public:
    ODDetector2DComplete(std::string const &training_data_location_) : ODDetector(training_data_location_)
    { }

    virtual ODDetections* detect(ODSceneImage *scene) {}
    virtual ODDetections3D* detectOmni(ODSceneImage *scene) {}
  };

}
#endif //OPENDETECTION_ODDETECTOR_H

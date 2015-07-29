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
  class ODDetector: public ODAlgorithmBase
  {
  public:

    ODDetector()
    { }

    ODDetector(std::string const &training_data_location_) : training_data_location_(training_data_location_)
    { }

    virtual int detect(ODScene *scene, std::vector<ODDetection *> &detections) {};

    virtual ODDetection* detect(ODScene *scene) {}

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

    bool metainfo_;
  protected:
    std::string training_input_location_, training_data_location_;

  };

}
#endif //OPENDETECTION_ODDETECTOR_H

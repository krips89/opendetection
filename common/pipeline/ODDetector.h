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

using namespace std;

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

    ODDetector(string const &training_data_location_) : training_data_location_(training_data_location_)
    { }

    virtual int detect(ODScene *scene, vector<ODDetection *> &detections) = 0;

    string getTrainingDataLocation() const
    {
      return training_data_location_;
    }

    void setTrainingDataLocation(string training_data_location_)
    {
      this->training_data_location_ = training_data_location_;
    }


  protected:
    string training_data_location_;
  };

}
#endif //OPENDETECTION_ODDETECTOR_H

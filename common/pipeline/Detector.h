//
// Created by sarkar on 08.06.15.
//

#ifndef OPENDETECTION_DETECTOR_H
#define OPENDETECTION_DETECTOR_H

//#include "ObjectDetector.h"
#include "Scene.h"
#include <iostream>

using namespace std;

namespace od
{
  class Detector
  {
  public:

    Detector(string const &training_input_location_, string const &training_data_location_) : training_input_location_(
        training_input_location_), training_data_location_(training_data_location_)
    { }

    int detect(Scene scene);

    string getTrainingInputLocation() const
    {
      return training_input_location_;
    }

    void setTrainingInputLocation(string training_input_location_)
    {
      this->training_input_location_ = training_input_location_;
    }

    string getTrainingDataLocation() const
    {
      return training_data_location_;
    }

    void setTrainingDataLocation(string training_data_location_)
    {
      this->training_data_location_ = training_data_location_;
    }


  protected:
    string training_input_location_, training_data_location_;
  };

}
#endif //OPENDETECTION_DETECTOR_H

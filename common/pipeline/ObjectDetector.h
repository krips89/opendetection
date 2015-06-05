//
// Created by sarkar on 03.06.15.
//

#ifndef OPENDETECTION_OBJECTDETECTOR_H
#define OPENDETECTION_OBJECTDETECTOR_H

#include <jmorecfg.h>
#include "Scene.h"
#include <string>

using namespace std;


namespace od
{

  enum DetectionMethod {
    PC_GLOBAL_FEATUREMATCHING,
    PC_LOCAL_CORRESPONDENCE_GROUPING,
    IMAGE_LOCAL_SIMPLE,
    IMAGE_GLOBAL_DENSE,
    IMAGE_GLOBAL_CLASSIFICATION,
  };


  class ObjectDetector {
  public:

    DetectionMethod const &getDetectionMethod() const
    {
      return detection_method_;
    }

    void setDetectionMethod(DetectionMethod const &detection_method_)
    {
      this->detection_method_ = detection_method_;
    }

    bool getAlwaysTrain() const
    {
      return always_train_;
    }

    void setAlwaysTrain(boolean always_train_)
    {
      this->always_train_ = always_train_;
    }

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

    int train();

    int detect(Scene const & scene);


  protected:
    DetectionMethod detection_method_;
    bool always_train_;
    string training_input_location_, training_data_location_;
  };

  //this class can be of different structure from ObjectDetector and should be defined seperately.
  // Subclassing ObjDetector as currently it requires all the information of ObjDetector
  class ODAlgorithmBase: public ObjectDetector
  {
  };

}

#endif //OPENDETECTION_OBJECTDETECTOR_H

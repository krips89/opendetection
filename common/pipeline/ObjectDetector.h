/*
Copyright (c) 2015, Kripasindhu Sarkar
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by sarkar on 03.06.15.
//

#ifndef OPENDETECTION_OBJECTDETECTOR_H
#define OPENDETECTION_OBJECTDETECTOR_H

#include <string>
#include <vector>
#include "ODDetection.h"


namespace od
{

  class ODScene;
  class ODDetection;

  enum DetectionMethod {
    PC_GLOBAL_FEATUREMATCHING,
    PC_LOCAL_CORRESPONDENCE_GROUPING,
    IMAGE_LOCAL_SIMPLE,
    IMAGE_GLOBAL_DENSE,
    IMAGE_GLOBAL_CLASSIFICATION,
  };

  /** \brief The common class for detectors. Both Trainers and Detectors drerives from this and therefore, all the common data/functionalities of Trainers and Detectors should go here.
  *
  *
  * \author Kripasindhu Sarkar
  *
  */

  class ODDetectorCommon
  {
  public:

    ODDetectorCommon( std::string const &trained_data_location_="") : trained_data_location_(trained_data_location_)
    {
      std::string clasname = typeid(this).name();
      TRAINED_DATA_ID_ = clasname;
      std::transform(clasname.begin(), clasname.end(), clasname.begin(), ::toupper);
      TRAINED_LOCATION_DENTIFIER_  = clasname;
    }

    virtual void init() = 0;


    /** \brief Gets/Sets the directory containing the data for training. The trainer uses the data from directory for training. Detectors can use this location to get additional information in its detection algirhtms as well.
      */
    std::string getTrainingInputLocation() const
    {
      return training_input_location_;
    }

    /** \brief Gets/Sets the directory containing the data for training. The trainer uses the data from directory for training. Detectors can use this location to get additional information in its detection algirhtms as well.
      */
    void setTrainingInputLocation(std::string training_input_location_)
    {
      this->training_input_location_ = training_input_location_;
    }

    /** \brief Gets/Sets the base directory for trained data. This should be same for all Trainers and Detectors and can be considered as the 'database' of trained data. Trainers uses one of its
     * subdirectories based on its type to store algo specific trained data. The corresponding Detector would use the same directory to fetch the trained data for online detection.
      */
    std::string getTrainedDataLocation() const
    {
      return trained_data_location_;
    }


    /** \brief The base directory for trained data. This should be same for all Trainers and Detectors and can be considered as the 'database' of trained data. Trainers uses one of its
     * subdirectories based on its type to store algo specific trained data. The corresponding Detector would use the same directory to fetch the trained data for online detection.
     */
    virtual void setTrainedDataLocation(std::string trained_data_location_)
    {
      this->trained_data_location_ = trained_data_location_;
    }


    /** \brief Gets the specific directory for a Trainer or a Detector inside trained_data_location_.
      */
    std::string getSpecificTrainingDataLocation()
    {
      return trained_data_location_ + "/" + "TD_" + TRAINED_LOCATION_DENTIFIER_;
    }

    std::string getSpecificTrainingData()
    {
      return getSpecificTrainingDataLocation() + "/" + TRAINED_DATA_ID_;
    }

    std::string const &getTrainedDataID() const
    {
      return TRAINED_DATA_ID_;
    }

    void setTrainedDataID(std::string const &trainedDataID)
    {
      ODDetectorCommon::TRAINED_DATA_ID_ = trainedDataID;
    }

  protected:
    std::string training_input_location_, trained_data_location_;
    std::string TRAINED_DATA_ID_, TRAINED_LOCATION_DENTIFIER_;
  };

  /** \brief This is the main class for object detection and recognition.
   *
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ObjectDetector {
  public:


    ObjectDetector()
    { }

    DetectionMethod const &getMethod() const
    {
      return method_;
    }

    void setDetectionMethod(DetectionMethod const &detection_method_)
    {
      this->method_ = detection_method_;
    }

    bool getAlwaysTrain() const
    {
      return always_train_;
    }

    void setAlwaysTrain(bool always_train_)
    {
      this->always_train_ = always_train_;
    }

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

    std::string getSpecificTrainingDataLocation()
    {
      return training_data_location_ + "/" + "TD_" + TRAINED_DATA_IDENTIFIER_;
    }

    virtual void init() = 0;

    virtual void initDetector(){}
    virtual void initTrainer(){}

    virtual int train() = 0;


    virtual int detect(ODScene *scene, std::vector<ODDetection *> detections) {}

    virtual ODDetection* detect(ODScene *scene) {}
    virtual ODDetections* detectOmni(ODScene *scene) {}

  protected:
    DetectionMethod method_;
    bool always_train_;
    bool trained_;
    std::string training_input_location_, training_data_location_;

    std::string TRAINED_DATA_EXT_, TRAINED_DATA_IDENTIFIER_;
  };

}

#endif //OPENDETECTION_OBJECTDETECTOR_H

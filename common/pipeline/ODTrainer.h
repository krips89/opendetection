//
// Created by sarkar on 08.06.15.
//

#ifndef OPENDETECTION_TRAINER_H
#define OPENDETECTION_TRAINER_H

#include<iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include "ODAlgorithmBase.h"
#include "ObjectDetector.h"


namespace bf = boost::filesystem;

namespace od
{
  /** \brief The base class for all trainers. All trainers derives from this and implement the function train().
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODTrainer: public ODDetectorCommon
  {
  public:

    ODTrainer(std::string const &training_input_location ="", std::string const &training_data_location="") : ODDetectorCommon(training_data_location)
    {
      training_input_location_ = training_data_location;
    }

    virtual int train() = 0;

  };

}
#endif //OPENDETECTION_TRAINER_H

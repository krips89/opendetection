//
// Created by sarkar on 08.06.15.
//

#ifndef OPENDETECTION_TRAINER_H
#define OPENDETECTION_TRAINER_H

#include<iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include "ODAlgorithmBase.h"


namespace bf = boost::filesystem;

namespace od
{
  /** \brief Maining Trainer class; all trainers derives from this
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODTrainer: public ODAlgorithmBase
  {
  public:

    ODTrainer(std::string const &training_input_location_ ="", std::string const &training_data_location_="") : training_input_location_(
        training_input_location_), training_data_location_(training_data_location_)
    { }

    virtual int train() = 0;

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





  protected:
    std::string training_input_location_, training_data_location_;
  };

}
#endif //OPENDETECTION_TRAINER_H

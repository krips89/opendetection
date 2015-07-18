//
// Created by sarkar on 16.06.15.
//

#ifndef OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGTRAINER_H
#define OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGTRAINER_H

#include <common/pipeline/ODTrainer.h>
#include <iostream>

namespace od
{
/** \brief ODPointCloudGlobalMatchingTrainer
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODPointCloudGlobalMatchingTrainer : public ODTrainer
  {

  public:
    ODPointCloudGlobalMatchingTrainer(std::string const &training_input_location_ = "", std::string const &training_data_location_ = "") : ODTrainer(training_input_location_, training_data_location_)
    {
      desc_name = "esf";
    }

    int train();

    std::string const &getDescName() const
    {
      return desc_name;
    }

    void setDescName(std::string const &desc_name)
    {
      ODPointCloudGlobalMatchingTrainer::desc_name = desc_name;
    }

  protected:
    std::string desc_name;
  };
}

#endif //OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGTRAINER_H

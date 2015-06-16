//
// Created by sarkar on 16.06.15.
//

#ifndef OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGTRAINER_H
#define OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGTRAINER_H

#include <common/pipeline/ODTrainer.h>

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
    ODPointCloudGlobalMatchingTrainer(string const &training_input_location_, string const &training_data_location_) : ODTrainer(training_input_location_, training_data_location_)
    { }

    int train();
  };
}

#endif //OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGTRAINER_H

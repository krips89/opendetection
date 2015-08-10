//
// Created by sarkar on 16.06.15.
//

#ifndef OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGTRAINER_H
#define OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGTRAINER_H

#include <common/pipeline/ODTrainer.h>
#include <iostream>

namespace od
{
  namespace g3d
  {
/** \brief ODCADDetectTrainer3DGlobal
   *
   * \author Kripasindhu Sarkar
   *
   */
    class ODCADDetectTrainer3DGlobal : public ODTrainer
    {

    public:
      ODCADDetectTrainer3DGlobal(std::string const &training_input_location_ = "", std::string const &training_data_location_ = "") : ODTrainer(
          training_input_location_, training_data_location_)
      {
        desc_name = "esf";
        TRAINED_DATA_IDENTIFIER_ = "GLOBAL3DVFH";
      }

      int train();

      std::string const &getDescName() const
      {
        return desc_name;
      }

      void setDescName(std::string const &desc_name)
      {
        ODCADDetectTrainer3DGlobal::desc_name = desc_name;
      }

    protected:
      std::string desc_name;
    };
  }
}

#endif //OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGTRAINER_H

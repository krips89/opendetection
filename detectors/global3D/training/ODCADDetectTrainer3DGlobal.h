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
/** \brief Training class for the detector ODCADDetector3DGlobal.
   *
   * This class uses PCL 3d_recognition_framework in the background for the training of 3D CAD models (in PLY format) and should be used with ODCADDetector3DGlobal for their detection in a pointcloud.
   * In the training_input_location_ the CAD models should be arranged in the following structure:
   *
   * - <Database dir>
        - <Category 1 dir>
          - ...
          - ...
        - <Category 2 dir>
          - ...
          - ...
        - ...

    For example:

    - /home/user/DB
      - /home/user/DB/bottles
        - /home/user/DB/bottles/milk.ply
        - /home/user/DB/bottles/wine.ply
      - /home/user/DB/fruit
        - /home/user/DB/fruit/apple.ply
        - /home/user/DB/fruit/banana.ply


    After the training use the detector class ODCADDetector3DGlobal.
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

      void init() {};

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

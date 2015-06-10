//
// Created by sarkar on 05.06.15.
//

#ifndef OPENDETECTION_ODIMAGELOCALMATCHINGSIMPLE_H
#define OPENDETECTION_ODIMAGELOCALMATCHINGSIMPLE_H

#include <common/pipeline/Scene.h>
#include <common/pipeline/Trainer.h>
#include <common/pipeline/Detector.h>
#include "common/pipeline/ObjectDetector.h"
#include "detectors/image_local/training/SnapshotCorrMaker.h"

namespace od
{

  class ODImageLocalMatching : public ODAlgorithmBase
  {

  public:

    enum TrainingMethod {
      TRAINING_SNAPSHOT_BASED_CORRESPONDENCES
    };
    enum DetectionMethod {
      DETECTION_SIMPLE_RANSAC,
      DETECTION_MOPED
    };

    ODImageLocalMatching(TrainingMethod const &training_method = TRAINING_SNAPSHOT_BASED_CORRESPONDENCES,
                               DetectionMethod const &detection_method = DETECTION_MOPED)
    {

    }

    void init()
    {
      //init training
      if (training_method_ == TRAINING_SNAPSHOT_BASED_CORRESPONDENCES)
      {
        trainer_ = new SnapshotCorrMaker(training_input_location_, training_data_location_);
      }
      //init online detector
      detector_ = new Detector(training_input_location_, training_data_location_);
    }

    int train()
    {
      return trainer_->train();
    }
    int detect(ImageScene const & scene)
    {
      detector_->detect(scene);
    }
    int detect(Scene const & scene) {return 0;}

  protected:


  };


}
#endif //OPENDETECTION_ODIMAGELOCALMATCHINGSIMPLE_H

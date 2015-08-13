//
// Created by sarkar on 05.06.15.
//

#ifndef OPENDETECTION_ODIMAGELOCALMATCHINGSIMPLE_H
#define OPENDETECTION_ODIMAGELOCALMATCHINGSIMPLE_H

#include "common/pipeline/ODScene.h"
#include "common/pipeline/ODTrainer.h"
#include "common/pipeline/ODDetector.h"
#include "common/pipeline/ObjectDetector.h"


namespace od
{

  namespace l2d
  {
    /** \brief ODImageLocalMatchingTrainer
   *
   * \author Kripasindhu Sarkar
   *
   */
    class ODImageLocalMatchingTrainer : public ODTrainer
    {

    public:
      ODImageLocalMatchingTrainer(std::string const &training_input_location_, std::string const &training_data_location_) : ODTrainer(training_input_location_, training_data_location_)
      {
        TRAINED_DATA_IDENTIFIER_ = "FEATCORR";
        TRAINED_DATA_EXT_ = "corr.xml";
      }
    };

    /** \brief ODImageLocalMatchingDetector
   *
   * \author Kripasindhu Sarkar
   *
   */
    class ODImageLocalMatchingDetector : public ODDetector2DComplete
    {

    public:
      ODImageLocalMatchingDetector(std::string const &training_data_location_) : ODDetector2DComplete(training_data_location_)
      {
        TRAINED_DATA_IDENTIFIER_ = "FEATCORR";
        TRAINED_DATA_EXT_ = "corr.xml";
      }
    };

    /** \brief ODImageLocalMatching
   *
   * \author Kripasindhu Sarkar
   *
   */
    class ODImageLocalMatching : public ObjectDetector
    {

    public:

      ODImageLocalMatchingTrainer *getTrainer() const
      {
        return trainer_;
      }

      void setTrainer(ODImageLocalMatchingTrainer *trainer_)
      {
        ODImageLocalMatching::trainer_ = trainer_;
      }

      ODImageLocalMatchingDetector *getDetector() const
      {
        return detector_;
      }

      void setDetector(ODImageLocalMatchingDetector *detector_)
      {
        ODImageLocalMatching::detector_ = detector_;
      }


      ODImageLocalMatching()
      {
        TRAINED_DATA_EXT_ = "corr.xml";
      }

      void init()
      { }


      int train()
      {
        return trainer_->train();
      }

      int detect(ODScene *scene, std::vector<ODDetection *> detections)
      {
        //detector_->detect(scene, detections);
        return 1;
      }


    protected:

      ODImageLocalMatchingTrainer *trainer_;
      ODImageLocalMatchingDetector *detector_;

    };

  }
}
#endif //OPENDETECTION_ODIMAGELOCALMATCHINGSIMPLE_H

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
        TRAINED_LOCATION_DENTIFIER_ = "FEATCORR";
        TRAINED_DATA_ID_ = "corr.xml";
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
        TRAINED_LOCATION_DENTIFIER_ = "FEATCORR";
        TRAINED_DATA_ID_ = ".xml";
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

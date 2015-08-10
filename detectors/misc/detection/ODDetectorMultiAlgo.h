//
// Created by sarkar on 06.08.15.
//

#ifndef OPENDETECTION_ODDETECTORMULTIALGO_H
#define OPENDETECTION_ODDETECTORMULTIALGO_H

#include <common/pipeline/ODDetector.h>
#include <common/pipeline/ODDetection.h>

namespace od
{


  class ODDetectorMultiAlgo : public ODDetector2D
  {
  public:
    ODDetectorMultiAlgo(string const &training_data_location_) : ODDetector2D(training_data_location_)
    { }


    ODDetections *detect(ODSceneImage *scene) ;

    ODDetections2D *detectOmni(ODSceneImage *scene);

    void init();

  private:
    std::vector<ODDetector2D *> detectors_;
  };



}
#endif //OPENDETECTION_ODDETECTORMULTIALGO_H

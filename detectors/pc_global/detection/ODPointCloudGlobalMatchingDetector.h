//
// Created by sarkar on 16.06.15.
//

#ifndef OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGDETECTOR_H
#define OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGDETECTOR_H

#include <common/pipeline/ODDetector.h>

namespace od
{
/** \brief ODPointCloudGlobalMatchingDetector
  *
  * \author Kripasindhu Sarkar
  *
  */
  class ODPointCloudGlobalMatchingDetector : public ODDetector
  {

  public:
    ODPointCloudGlobalMatchingDetector(string const &training_data_location_) : ODDetector(training_data_location_)
    { }

    void parseParameterString(string parameter_string);

    void init();

    int detect(ODScene *scene, vector<ODDetection *> &detections);
    int detect(ODSceneImage *scene, vector<ODDetection3D *> &detections);

  };

}

#endif //OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGDETECTOR_H

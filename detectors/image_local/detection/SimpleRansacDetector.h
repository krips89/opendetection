//
// Created by sarkar on 08.06.15.
//

#ifndef OPENDETECTION_SIMPLERANSACDETECTOR_H
#define OPENDETECTION_SIMPLERANSACDETECTOR_H

#include <iostream>

#include "common/pipeline/Detector.h"
#include "common/pipeline/Scene.h"

using namespace std;

namespace od
{

  class SimpleRansacDetector : public Detector
  {
  public:
    SimpleRansacDetector(string const &training_input_location_, string const &training_data_location_): Detector (training_input_location_, training_data_location_)
    { }
    int detect(ImageScene scene);
  };

}
#endif //OPENDETECTION_SIMPLERANSACDETECTOR_H

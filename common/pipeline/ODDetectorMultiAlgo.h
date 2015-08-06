//
// Created by sarkar on 06.08.15.
//

#ifndef OPENDETECTION_ODDETECTORMULTIALGO_H
#define OPENDETECTION_ODDETECTORMULTIALGO_H

#include "ODDetector.h"
#include "ODDetection.h"


class ODDetectorMultiAlgo: public ODDetector
{
  ODDetection* detect(ODScene *scene)
  { }

  ODDetections* detectOmni(ODScene *scene) {}
};


#endif //OPENDETECTION_ODDETECTORMULTIALGO_H

//
// Created by sarkar on 06.08.15.
//

#include "ODDetectorMultiAlgo.h"
#include "detectors/global2D/detection/ODCascadeDetector.h"
#include "detectors/global2D/detection/ODHOGMultiscaleDetector.h"
#include "detectors/global2D/ODFaceRecognizer.h"

#include "detectors/local2D/detection/SimpleRansacDetector.h"


using namespace std;
using namespace od::g2d;
using namespace od::l2d;

namespace od
{
  ODDetections *ODDetectorMultiAlgo::detect(ODSceneImage *scene)
  {
    return NULL;
  }

  ODDetections2D *ODDetectorMultiAlgo::detectOmni(ODSceneImage *scene)
  {
    //make a list of different algorithms
    //vector<ODDetector *> detectors = {new ODCascadeDetector(training_data_location_), new ODHOGMultiscaleDetector(training_data_location_), new SimpleRansacDetector(training_data_location_)};
    vector<ODDetector2D *> detectors;
    detectors.push_back(new ODCascadeDetector(training_data_location_));
    detectors.push_back(new ODHOGMultiscaleDetector(training_data_location_));
    //  detectors.push_back(new SimpleRansacDetector(training_data_location_));


    ODDetections2D * detections_all = new ODDetections2D;
    for (int i = 0; i < detectors.size(); i++)
    {
      ODDetections * detections_individual = detectors[i]->detectOmni(scene);
      detections_all->append(detections_individual);
    }

    return detections_all;
  }
}
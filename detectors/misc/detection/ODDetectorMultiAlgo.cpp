//
// Created by sarkar on 06.08.15.
//

#include "ODDetectorMultiAlgo.h"
#include "detectors/global2D/detection/ODCascadeDetector.h"
#include "detectors/global2D/detection/ODHOGDetector.h"
#include "detectors/global2D/ODFaceRecognizer.h"

#include "detectors/local2D/detection/ODCADRecognizer2DLocal.h"


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
    ODDetections2D * detections_all = new ODDetections2D;
    for (int i = 0; i < detectors_.size(); i++)
    {
      ODDetections2D * detections_individual = detectors_[i]->detectOmni(scene);
      detections_all->append(detections_individual);
    }

    return detections_all;
  }

  void ODDetectorMultiAlgo::init()
  {
    //make a list of different algorithms
    //vector<ODDetector *> detectors = {new ODCascadeDetector(training_data_location_), new ODHOGDetector(training_data_location_), new ODCADRecognizer2DLocal(training_data_location_)};
    detectors_.push_back(new ODCascadeDetector(training_data_location_));
    detectors_.push_back(new ODHOGDetector(training_data_location_));
    //  detectors.push_back(new ODCADRecognizer2DLocal(training_data_location_));


    for (int i = 0; i < detectors_.size(); i++)
    {
      detectors_[i]->init();
    }
  }
}
#include "common/pipeline/ObjectDetector.h"
#include "detectors/image_local/ODImageLocalMatching.h"



int main(int argc, char *argv[])
{
  od::ODImageLocalMatching detector;
  detector.setTrainingInputLocation(argv[1]);
  detector.setTrainingDataLocation(argv[2]);
  detector.setTrainingMethod(od::ODImageLocalMatching::TRAINING_SNAPSHOT_BASED_CORRESPONDENCES);
  detector.setDetectionMethod(od::ODImageLocalMatching::DETECTION_MOPED);
  detector.init();
  detector.train();


  return 0;
}
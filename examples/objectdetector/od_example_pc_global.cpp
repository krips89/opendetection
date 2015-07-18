//
// Created by sarkar on 16.06.15.
//

/** \brief Example of the usage of global 3D pipeline
   *
   * \author Kripasindhu Sarkar
   *
   */


#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"
#include "detectors/global3D/ODPointCloudGlobalMatching.h"


int main(int argc, char *argv[])
{

  string training_input_dir(argv[1]), trained_data_dir(argv[2]), pointcloud_file(argv[3]);

  //trainer
  od::ODTrainer *trainer = new od::ODPointCloudGlobalMatchingTrainer(training_input_dir, trained_data_dir);
  trainer->train();


  //detector
  od::ODPointCloudGlobalMatchingDetector<> *detector = new od::ODPointCloudGlobalMatchingDetector<>();
  detector->setTrainingInputLocation(training_input_dir);
  detector->setTrainingDataLocation(trained_data_dir);
  detector->init();


  //Get a scene
  od::ODScenePointCloud<> scene(pointcloud_file);
  vector<od::ODDetection3D *> detections;

  //Detect
  detector->detect(&scene, detections);

  //feedback
  for(int i = 0; i < detections.size(); i++)
  {
    detections[i]->printSelf();
  }

  return 0;
}

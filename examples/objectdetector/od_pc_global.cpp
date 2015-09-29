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

  std::string training_input_dir(argv[1]), trained_data_dir(argv[2]), pointcloud_file(argv[3]);

  //trainer
  od::ODTrainer *trainer = new od::g3d::ODCADDetectTrainer3DGlobal(training_input_dir, trained_data_dir);
  trainer->train();


  //detector
  od::g3d::ODCADDetector3DGlobal<> *detector = new od::g3d::ODCADDetector3DGlobal<>();
  detector->setTrainingInputLocation(training_input_dir);
  detector->setTrainedDataLocation(trained_data_dir);
  detector->init();


  //Get a scene
  od::ODScenePointCloud<> *scene = new od::ODScenePointCloud<>(pointcloud_file);

  od::ODDetections3D * detections = detector->detectOmni(scene);

  //feedback
  for(int i = 0; i < detections->size(); i++)
  {
    detections->at(i)->printSelf();
  }

  return 0;
}

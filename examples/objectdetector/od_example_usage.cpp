#include <detectors/image_local/training/SnapshotCorrTrainer.h>
#include <detectors/image_local/detection/SimpleRansacDetector.h>

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"
#include "detectors/image_local/ODImageLocalMatching.h"



int main(int argc, char *argv[])
{

  string training_input_dir(argv[1]), trained_data_dir(argv[2]), image_file(argv[3]);

  //trainer
  od::SnapshotCorrTrainer *trainer = new od::SnapshotCorrTrainer(training_input_dir, trained_data_dir);
  //trainer->train();


  //detector
  od::SimpleRansacDetector *detector = new od::SimpleRansacDetector(trained_data_dir);
  //set commandline options type inputs
  detector->parseParameterString("--use_gpu --fast --method=1 --error=2 --confidence=0.9 --iterations=500 --inliers=20 --metainfo");
  //set some other inputs
  detector->setCamera_intrinsic_file("image_local_scenes/camera_webcam_fixed.xml");
  //init
  detector->init();


  //Get a scene
  od::ODSceneImage scene(image_file);
  vector<od::ODDetection3D *> detections;

  //Detect
  detector->detect(&scene, detections);

  //feedback
  for(int i = 0; i < detections.size(); i++)
  {
    cv::namedWindow("Overlay" + toString(i), cv::WINDOW_NORMAL);
    detections[i]->printSelf();
    cv::imshow("Overlay" + toString(i), detections[i]->metainfo_image_);
  }
  cv::waitKey();


  return 0;
}
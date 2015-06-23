#include <detectors/image_local/training/SnapshotCorrTrainer.h>
#include <detectors/image_local/detection/SimpleRansacDetector.h>
#include "common/utils/ODFrameGenerator.h"

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"
#include "detectors/image_local/ODImageLocalMatching.h"


using namespace od;

void visualizeDetection(vector<od::ODDetection3D *> detections)
{
  //feedback
  for(int i = 0; i < detections.size(); i++)
  {
    cv::namedWindow("Overlay" + toString(i), cv::WINDOW_NORMAL);
    detections[i]->printSelf();
    cv::imshow("Overlay" + toString(i), detections[i]->metainfo_image_);
  }
  cv::waitKey();
}

int main(int argc, char *argv[])
{
  string training_input_dir(argv[1]), trained_data_dir(argv[2]), image_files(argv[3]);

  //trainer
  od::SnapshotCorrTrainer *trainer = new od::SnapshotCorrTrainer(training_input_dir, trained_data_dir);
  //trainer->train();

  //detector
  od::SimpleRansacDetector *detector = new od::SimpleRansacDetector(trained_data_dir);
  //set commandline options type inputs
  detector->parseParameterString("--use_gpu --fast --method=1 --error=2 --confidence=0.9 --iterations=500 --inliers=20 --metainfo");
  detector->setCameraIntrinsicFile("image_local_scenes/camera_webcam_fixed.xml");   //set some other inputs
  detector->init();

  //get scenes
  od::ODFrameGenerator<od::ODSceneImage, od::FILE_LIST> frameGenerator(image_files);
  while(frameGenerator.isValid())
  {
    od::ODSceneImage * scene = frameGenerator.getNextFrame();

    //Detect
    vector<od::ODDetection3D *> detections;
    detector->detect(scene, detections);

    visualizeDetection(detections);
  }

  return 0;
}
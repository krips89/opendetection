#include <detectors/image_local/training/SnapshotCorrTrainer.h>
#include <detectors/image_local/detection/SimpleRansacDetector.h>
#include "common/utils/ODFrameGenerator.h"

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"
#include "detectors/image_local/ODImageLocalMatching.h"


using namespace od;

int main(int argc, char *argv[])
{
  string training_input_dir(argv[1]), trained_data_dir(argv[2]);

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
  od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_DEVICE> frameGenerator("0");
  //GUI
  cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
  while(frameGenerator.isValid() && cv::waitKey(30) != 27)
  {
    od::ODSceneImage * scene = frameGenerator.getNextFrame();
    cv::imshow("Overlay", scene->getCVImage());

    //Detect
    vector<od::ODDetection3D *> detections;
    detector->detect(scene, detections);

    if(detections.size() > 0)
      cv::imshow("Overlay", detections[0]->metainfo_image_); //only showing the first detection
    else
      cv::imshow("Overlay", scene->getCVImage());
  }

  return 0;
}
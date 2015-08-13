
/** \brief Example of the usage of local 2d pipeline with simple ransac based pose detector
   *
   * \author Kripasindhu Sarkar
   *
   */

#include "detectors/local2D/training/ODCADRecogTrainerSnapshotBased.h"
#include "detectors/local2D/detection/ODCADRecognizer2DLocal.h"
#include "common/utils/ODFrameGenerator.h"

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"
#include "detectors/local2D/ODImageLocalMatching.h"


using namespace od;

int main(int argc, char *argv[])
{
  std::string training_input_dir(argv[1]), trained_data_dir(argv[2]);

  //trainer
  od::l2d::ODCADRecogTrainerSnapshotBased *trainer = new od::l2d::ODCADRecogTrainerSnapshotBased(training_input_dir, trained_data_dir);
  //trainer->train();

  //detector
  od::l2d::ODCADRecognizer2DLocal *detector = new od::l2d::ODCADRecognizer2DLocal(trained_data_dir);
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

    //Detect
    ODDetections3D *detections =  detector->detectOmni(scene);

    if(detections->size() > 0)
      cv::imshow("Overlay", detections->getMetainfoImage()); //only showing the first detection
    else
      cv::imshow("Overlay", scene->getCVImage());
  }

  return 0;
}
/** \brief Example of the usage of cascade detector
   *
   * \author Kripasindhu Sarkar
   *
   */

#include <detectors/global2D/detection/ODCascadeDetector.h>
#include "common/utils/ODFrameGenerator.h"

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"


using namespace od;

int main(int argc, char *argv[])
{
  string trained_cascade(argv[1]);

  //detector
  od::g2d::ODCascadeDetector *detector = new od::g2d::ODCascadeDetector;
  detector->setTrainingDataLocation(trained_cascade);
  detector->init();

  //get scenes
  od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_DEVICE> frameGenerator("0");
  //GUI
  cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
  while(frameGenerator.isValid() && cv::waitKey(20) != 27)
  {
    od::ODSceneImage * scene = frameGenerator.getNextFrame();

    //Detect
    ODDetections2D *detections =  detector->detectOmni(scene);

    if(detections->size() > 0)
      cv::imshow("Overlay", detections->getMetainfoImage()); //only showing the first detection
    else
      cv::imshow("Overlay", scene->getCVImage());

    delete scene;
  }

  return 0;
}
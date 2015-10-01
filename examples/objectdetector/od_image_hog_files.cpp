/** \brief Example of the usage of cascade detector
   *
   * \author Kripasindhu Sarkar
   *
   */

#include <detectors/global2D/detection/ODCascadeDetector.h>
#include <detectors/global2D/detection/ODHOGDetector.h>
//#include "detectors/global2D/detection/ODHOGDetector.h"
#include "common/utils/ODFrameGenerator.h"

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"


using namespace od;

int main(int argc, char *argv[])
{
  std::string trained_hog(argv[1]), images(argv[2]);

  //detector
  od::g2d::ODHOGDetector *detector = new od::g2d::ODHOGDetector;
  detector->setTrainedDataLocation(trained_hog);
  //detector->setSvmtype(g2d::ODHOGDetector::OD_DAIMLER_PEOPLE);
  detector->init();

  //get scenes
  od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_FILE_LIST> frameGenerator(images);
  //GUI
  cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
  while(frameGenerator.isValid() && cv::waitKey(1000) != 27)
  {
    od::ODSceneImage * scene = frameGenerator.getNextFrame();

    //Detect
    ODDetections2D *detections =  detector->detectOmni(scene);

    if(detections->size() > 0)
      cv::imshow("Overlay", detections->renderMetainfo(*scene).getCVImage());
    else
      cv::imshow("Overlay", scene->getCVImage());

    delete scene;
  }

  return 0;
}
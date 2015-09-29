
#include <detectors/misc/detection/ODDetectorMultiAlgo.h>
#include <opencv2/highgui.hpp>
#include "common/utils/ODFrameGenerator.h"

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"


using namespace od;

int main(int argc, char *argv[])
{
  std::string trained_data_dir(argv[1]), query_images(argv[2]);;

  //detector
  od::ODDetectorMultiAlgo2D *detector = new od::ODDetectorMultiAlgo2D(trained_data_dir);
  detector->init();

  //get scenes
  od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_FILE_LIST> frameGenerator(query_images);
  //GUI
  cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
  while(frameGenerator.isValid() && cv::waitKey(3000) != 27)
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

/** \brief Example of the usage of HOG detector
   *
   * \author Kripasindhu Sarkar
   *
   */


#include <detectors/global2D/training/ODHOGTrainer.h>
#include "detectors/global2D/detection/ODHOGDetector.h"
#include "common/utils/ODFrameGenerator.h"

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"


using namespace od;

int main(int argc, char *argv[])
{

  std::string pos_samples(argv[1]), neg_samples(argv[2]), trained_data_dir(argv[3]), test_images(argv[4]);

  //trainer
  od::g2d::ODHOGTrainer *trainer = new od::g2d::ODHOGTrainer("", trained_data_dir);
  trainer->setPosSamplesDir(pos_samples);
  trainer->setNegSamplesDir(neg_samples);
  trainer->setNOFeaturesNeg(10);
  trainer->setTrainHardNegetive(true);
  trainer->train();


  od::g2d::ODHOGDetector *detector = new od::g2d::ODHOGDetector;
  detector->setTrainedDataLocation(trained_data_dir);
  detector->init();

  //get scenes
  od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_FILE_LIST> frameGenerator(test_images);
  //GUI
  cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
  while(frameGenerator.isValid() && cv::waitKey(2000) != 27)
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
/** \brief Example of the usage of FaceRecognizer
   *
   * \author Kripasindhu Sarkar
   *
   */


#include <detectors/global2D/detection/ODHOGMultiscaleDetector.h>
#include <detectors/global2D/ODFaceRecognizer.h>

#include "common/utils/ODFrameGenerator.h"
#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"


using namespace od;

int main(int argc, char *argv[])
{
  string training_input_csv(argv[1]), trained_xml(argv[2]), query_images(argv[3]);
  //detector
  g2d::ODFaceRecognizer * objdetector = new g2d::ODFaceRecognizer;
  objdetector->setTrainingInputLocation(training_input_csv);
  objdetector->setTrainingDataLocation(trained_xml);
  objdetector->setRecogtype(g2d::ODFaceRecognizer::OD_FACE_FISCHER);
  objdetector->initTrainer();
  objdetector->train();


  objdetector->initDetector();

  //get scenes
  od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_FILE_LIST> frameGenerator(query_images);
  //GUI
  cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
  while(frameGenerator.isValid() && cv::waitKey(2000) != 27)
  {
    od::ODSceneImage * scene = frameGenerator.getNextFrame();

    //Detect
    ODDetections2D *detections =  objdetector->detect(scene);
    (*detections)[0]->printSelf();

    cv::imshow("Overlay", scene->getCVImage());

    delete scene;
  }

  return 0;
}
#include "detectors/local2D/training/ODCADRecogTrainerSnapshotBased.h"
#include "detectors/local2D/detection/ODCADRecognizer2DLocal.h"
#include "common/utils/ODFrameGenerator.h"

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"
#include "detectors/local2D/ODImageLocalMatching.h"
///home/sarkar/work/opencv/real_time_pose_estimation/Data/Lion/test_images/IMG_*.JPG /home/sarkar/work/opencv/face  /home/sarkar/models/MODELS_DATA/Lion/test_images/Lion_old/original/out_camera_data_lion_old.xml /home/sarkar/models/MODELS_DATA/Lion/test_images/Lion_old/original/ransac_detection_txtmap.txt


using namespace od;
using namespace std;

int main(int argc, char *argv[])
{
  std::string imagespath(argv[1]), modelsPath(argv[2]), camerapath(argv[3]), outputfile(argv[4]);
  ofstream logfile(outputfile.c_str());

  //detector
  od::l2d::ODCADRecognizer2DLocal *detector = new od::l2d::ODCADRecognizer2DLocal(modelsPath);
  //set commandline options type inputs
  detector->parseParameterString("--use_gpu --method=1 --error=2 --confidence=0.5 --iterations=1000 --inliers=6 --metainfo");
  detector->setCameraIntrinsicFile(camerapath);   //set some other inputs
  detector->init();

  //get scenes
  od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_FILE_LIST> frameGenerator(imagespath);
  //GUI
  //cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
  while(frameGenerator.isValid() && cv::waitKey(1000) != 27)
  {
    od::ODSceneImage * scene = frameGenerator.getNextFrame();
    //cv::imshow("Overlay", scene->getCVImage());

    //Detect
    ODDetections3D *detections =  detector->detectOmni(scene);

    if(detections->size() > 0)
    {
      logfile << scene->getPath() << endl;
      logfile << detections->size() << endl;

      for (int i = 0; i < detections->size(); i++)
      {
        ODDetection3D *detection = detections->at(i);
        detection->printSelf();
        logfile << detection->getId() << endl;
      }

      //cv::imshow("Overlay", detections->getMetainfoImage());
    }
    else
      //cv::imshow("Overlay", scene->getCVImage());



    delete scene;
    delete detections;

  }

  return 0;
}
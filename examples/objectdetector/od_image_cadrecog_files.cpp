/*
Copyright (c) 2015, Kripasindhu Sarkar
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/#include "detectors/local2D/training/ODCADRecogTrainerSnapshotBased.h"
#include "detectors/local2D/detection/ODCADRecognizer2DLocal.h"
#include "common/utils/ODFrameGenerator.h"

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"
#include "detectors/local2D/ODImageLocalMatching.h"


using namespace od;

int main(int argc, char *argv[])
{
  std::string training_input_dir(argv[1]), trained_data_dir(argv[2]), query_images(argv[3]);

  //trainer
  od::l2d::ODCADRecogTrainerSnapshotBased *trainer = new od::l2d::ODCADRecogTrainerSnapshotBased(training_input_dir, trained_data_dir);
  //trainer->train();

  //detector
  od::l2d::ODCADRecognizer2DLocal *detector = new od::l2d::ODCADRecognizer2DLocal(trained_data_dir);
  //set commandline options type inputs
  detector->parseParameterString("--use_gpu --method=1 --error=2 --confidence=0.8 --iterations=500 --inliers=20 --metainfo");
  detector->setCameraIntrinsicFile("/home/sarkar/models/opendetection_local/image_local_scenes/camera_householdnew.xml");   //set some other inputs
  detector->init();

  //get scenes
  od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_FILE_LIST> frameGenerator(query_images);
  //GUI
  cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
  while(frameGenerator.isValid() && cv::waitKey(30) != 27)
  {
    od::ODSceneImage * scene = frameGenerator.getNextFrame();
    cv::imshow("Overlay", scene->getCVImage());

    //Detect
    ODDetections3D *detections =  detector->detectOmni(scene);

    if(detections->size() > 0)
      cv::imshow("Overlay", detections->getMetainfoImage());
    else
      cv::imshow("Overlay", scene->getCVImage());


    delete scene;

  }

  return 0;
}
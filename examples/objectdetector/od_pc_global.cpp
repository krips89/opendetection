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
*///
// Created by sarkar on 16.06.15.
//

/** \brief Example of the usage of global 3D pipeline
   *
   * \author Kripasindhu Sarkar
   *
   */


#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"
#include "detectors/global3D/ODPointCloudGlobalMatching.h"


int main(int argc, char *argv[])
{

  std::string training_input_dir(argv[1]), trained_data_dir(argv[2]), pointcloud_file(argv[3]);

  //trainer
  od::ODTrainer *trainer = new od::g3d::ODCADDetectTrainer3DGlobal(training_input_dir, trained_data_dir);
  trainer->train();


  //detector
  od::g3d::ODCADDetector3DGlobal<> *detector = new od::g3d::ODCADDetector3DGlobal<>();
  detector->setTrainingInputLocation(training_input_dir);
  detector->setTrainedDataLocation(trained_data_dir);
  detector->init();


  //Get a scene
  od::ODScenePointCloud<> *scene = new od::ODScenePointCloud<>(pointcloud_file);

  od::ODDetections3D * detections = detector->detectOmni(scene);

  //feedback
  for(int i = 0; i < detections->size(); i++)
  {
    detections->at(i)->printSelf();
  }

  return 0;
}

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
*/
#include <detectors/misc/detection/ODDetectorMultiAlgo.h>
#include <opencv2/highgui.hpp>
#include "common/utils/ODFrameGenerator.h"

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"


/** \brief Example of the usage of Multi algorithm detector for point cloud
   *
   * \author Kripasindhu Sarkar
   *
   */


#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"
#include "detectors/global3D/ODPointCloudGlobalMatching.h"
#include "common/utils/ODFrameGenerator.h"


int main(int argc, char *argv[])
{

  std::string training_input_dir(argv[1]), trained_data_dir(argv[2]);


  //detector
  od::ODDetectorMultiAlgo *detector = new od::ODDetectorMultiAlgo(trained_data_dir);
  detector->setTrainingInputLocation(training_input_dir);
  detector->init();


  //GUI and feedback
  od::ODScenePointCloud<pcl::PointXYZRGBA> *frame;

  pcl::visualization::PCLVisualizer vis ("kinect");
  size_t previous_cluster_size = 0;


  od::ODFrameGenerator<od::ODScenePointCloud<pcl::PointXYZRGBA>, od::GENERATOR_TYPE_DEVICE> frameGenerator;
  while(frameGenerator.isValid())
  {

    frame = frameGenerator.getNextFrame();

    //remove previous point clouds and text and add new ones in the visualizer
    vis.removePointCloud ("frame");
    vis.addPointCloud<pcl::PointXYZRGBA> (frame->getPointCloud(), "frame");
    for (size_t i = 0; i < previous_cluster_size; i++)
    {
      std::stringstream cluster_name;
      cluster_name << "cluster_" << i;
      vis.removePointCloud (cluster_name.str ());
      vis.removeText3D (cluster_name.str() + "_txt");

      cluster_name << "_ply_model_";
      vis.removeShape (cluster_name.str ());
    }

    //Detect

    od::ODDetections3D * detections = detector->detectOmni(frame);


    //add all the detections in the visualizer with its id as text
    for (size_t i = 0; i < detections->size (); i++)
    {
      std::stringstream cluster_name;
      cluster_name << "cluster_" << i;
      pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> random_handler (detections->at(i)->getMetainfoCluster());
      vis.addPointCloud<pcl::PointXYZ> (detections->at(i)->getMetainfoCluster(), random_handler, cluster_name.str ());

      pcl::PointXYZ pos; pos.x = detections->at(i)->getLocation()[0]; pos.y = detections->at(i)->getLocation()[1]; pos.z = detections->at(i)->getLocation()[2];
      vis.addText3D (detections->at(i)->getId(), pos, 0.015f, 1, 0, 1, cluster_name.str() + "_txt", 0);
    }
    previous_cluster_size = detections->size ();


    vis.spinOnce ();
  }

  return 0;
}

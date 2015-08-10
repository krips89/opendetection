
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

  string training_input_dir(argv[1]), trained_data_dir(argv[2]);


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

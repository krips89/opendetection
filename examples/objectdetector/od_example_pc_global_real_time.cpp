//
// Created by sarkar on 16.06.15.
//

/** \brief Example of the usage of global pipeline
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

  //trainer
  od::ODTrainer *trainer = new od::ODPointCloudGlobalMatchingTrainer(training_input_dir, trained_data_dir);
  trainer->train();


  //detector
  od::ODPointCloudGlobalMatchingDetector<pcl::PointXYZRGBA> *detector = new od::ODPointCloudGlobalMatchingDetector<pcl::PointXYZRGBA>();
  detector->setTrainingInputLocation(training_input_dir);
  detector->setTrainingDataLocation(trained_data_dir);
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
    vector<od::ODDetection3D *> detections;
    detector->detect(frame, detections);


    //add all the detections in the visualizer with its id as text
    for (size_t i = 0; i < detections.size (); i++)
    {
      std::stringstream cluster_name;
      cluster_name << "cluster_" << i;
      pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> random_handler (detections[i]->getMetainfoCluster());
      vis.addPointCloud<pcl::PointXYZ> (detections[i]->getMetainfoCluster(), random_handler, cluster_name.str ());

      pcl::PointXYZ pos; pos.x = detections[i]->getLocation()[0]; pos.x = detections[i]->getLocation()[0]; pos.x = detections[i]->getLocation()[0];
      vis.addText3D (detections[i]->getId(), pos, 0.015f, 1, 0, 1, cluster_name.str() + "_txt", 0);
    }
    previous_cluster_size = detections.size ();


    vis.spinOnce ();
  }

  return 0;
}

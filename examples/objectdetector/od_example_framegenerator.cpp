//
// Created by sarkar on 16.06.15.
//

/** \brief Example of the usage of FrameGenarator class for capturing kinect point cloud
   *
   * \author Kripasindhu Sarkar
   *
   */

#include <boost/smart_ptr/make_shared_array.hpp>
#include "common/utils/ODFrameGenerator.h"


int main(int argc, char *argv[])
{
  //GUI and feedback
  pcl::visualization::PCLVisualizer vis ("kinect");

  od::ODScenePointCloud<pcl::PointXYZRGBA> *frame;
  od::ODFrameGenerator<od::ODScenePointCloud<pcl::PointXYZRGBA>, od::DEVICE> frameGenerator("");
  while(frameGenerator.isValid())
  {

    frame = frameGenerator.getNextFrame();

    vis.removePointCloud ("frame");
    vis.addPointCloud<pcl::PointXYZRGBA> (frame->getPointCloud(), "frame");

    vis.spinOnce ();
  }
  return 0;
}

//
// Created by sarkar on 19.06.15.
//

#include "ODFrameGenerator.h"

namespace od
{
  template class ODFrameGenerator<ODScenePointCloud<pcl::PointXYZRGBA> , DEVICE>;
  //template ODFrameGenerator<>::ODFrameGenerator(std::string input);
}
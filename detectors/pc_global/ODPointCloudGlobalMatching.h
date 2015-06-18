//
// Created by sarkar on 16.06.15.
//

#ifndef OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H
#define OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H

#include <common/pipeline/ODDetector.h>
#include <common/pipeline/ODTrainer.h>
#include <detectors/pc_global/training/ODPointCloudGlobalMatchingTrainer.h>
#include <detectors/pc_global/detection/ODPointCloudGlobalMatchingDetector.h>

namespace od
{





  /** \brief ODPointCloudGlobalMatching: global feature based object detection in point cloud
   *
   * \author Kripasindhu Sarkar
   *
   */

  class ODPointCloudGlobalMatching: ObjectDetector
  {

    ODPointCloudGlobalMatching()
    {
    }

    void init()
    { }


    int train()
    {
      return trainer_->train();
    }

    int detect(ODScene *scene, vector<ODDetection *> detections)
    {
      detector_->detect(scene, detections);
    }

  protected:
    std::string desc_name;
    ODPointCloudGlobalMatchingTrainer *trainer_;
    ODPointCloudGlobalMatchingDetector<pcl::PointXYZ> *detector_;
  };
}
#endif //OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H

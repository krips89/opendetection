//
// Created by sarkar on 16.06.15.
//

#ifndef OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H
#define OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H

#include <common/pipeline/ODDetector.h>
#include <common/pipeline/ODTrainer.h>
#include "detectors/global3D/training/ODCADDetectTrainer3DGlobal.h"
#include "detectors/global3D/detection/ODCADDetector3DGlobal.h"

namespace od
{

  namespace g3d
  {
    /** \brief ODPointCloudGlobalMatching: global feature based object detection in point cloud
   *
   * \author Kripasindhu Sarkar
   *
   */

    class ODPointCloudGlobalMatching : ObjectDetector
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

      int detect(ODScene *scene, std::vector<ODDetection *> detections)
      {
        //detector_->detect(scene, detections);
        return 0;
      }

    protected:
      std::string desc_name;
      ODCADDetectTrainer3DGlobal *trainer_;
      ODCADDetector3DGlobal<pcl::PointXYZ> *detector_;
    };
  }
}
#endif //OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H

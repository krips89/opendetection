//
// Created by sarkar on 16.06.15.
//

#ifndef OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGDETECTOR_H
#define OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGDETECTOR_H

#include <common/pipeline/ODDetector.h>




#include <pcl/pcl_macros.h>
#include <pcl/apps/3d_rec_framework/pipeline/global_nn_classifier.h>
#include <pcl/apps/3d_rec_framework/pc_source/mesh_source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/vfh_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/esf_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/cvfh_estimator.h>
//#include <pcl/apps/3d_rec_framework/tools/openni_frame_source.h>
#include <pcl/apps/3d_rec_framework/utils/metrics.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/console/parse.h>

namespace od
{
  namespace g3d
  {
/** \brief ODCADDetector3DGlobal
  *
  * \author Kripasindhu Sarkar
  *
  */
    template<typename PointT = pcl::PointXYZRGBA>
    class ODCADDetector3DGlobal : public ODDetector3D<PointT>
    {

    public:
      ODCADDetector3DGlobal(string const &training_data_location = "", string const &training_input_location = "") : ODDetector3D<PointT>(training_data_location),
                                                                          NN(2), desc_name("esf")
      {
        this->TRAINED_DATA_IDENTIFIER_ = "GLOBAL3DVFH";
        this->training_input_location_ = training_input_location;
      }

      void init();

      ODDetections *detect(ODScenePointCloud<PointT> *scene);

      ODDetections3D *detectOmni(ODScenePointCloud<PointT> *scene);

      int getNN() const
      {
        return NN;
      }

      void setNN(int NN)
      {
        ODCADDetector3DGlobal::NN = NN;
      }

      string const &getDescName() const
      {
        return desc_name;
      }

      void setDescName(string const &desc_name)
      {
        ODCADDetector3DGlobal::desc_name = desc_name;
      }

    protected:
      int NN;
      string desc_name;
      boost::shared_ptr<pcl::rec_3d_framework::GlobalClassifier<pcl::PointXYZ> > global_;

    };
  }
}

#include "ODCADDetector3DGlobal.hpp"
#endif //OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGDETECTOR_H

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
  /** \brief Detector based on 3D global features like VFH, ESF, CVFH etc.
    *
    * This class uses PCL 3d_recognition_framework in the background for the detection. First train your data using ODCADDetectTrainer3DGlobal and use this class for the detection.
    * This detection will assume the presence of a plane (like a table top) in the pointcloud. It segments the point cloud assuming the presence of a plane and using a simple Euclidian segmentation.
    * After that it finds the global features of each segmented scene match them with the trained data thereby performing a clssification. Read the documentation of ODCADDetectTrainer3DGlobal to know how
    * the training data should be arranged and trained to get meaningful detection.
    *
    * This class provides a detection for each segmented scenes which matches them the best. So the number of positive detection is same as the number of possible segmented scene in the point cloud.
    *
    * \author Kripasindhu Sarkar
    *
    */
    template<typename PointT = pcl::PointXYZRGBA>
    class ODCADDetector3DGlobal : public ODDetector3D<PointT>
    {

    public:
      ODCADDetector3DGlobal(std::string const &training_data_location = "", std::string const &training_input_location = "") : ODDetector3D<PointT>(training_data_location),
                                                                          NN(2), desc_name("esf")
      {
        this->TRAINED_LOCATION_DENTIFIER_ = "GLOBAL3DVFH";
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

      std::string const &getDescName() const
      {
        return desc_name;
      }

      void setDescName(std::string const &desc_name)
      {
        ODCADDetector3DGlobal::desc_name = desc_name;
      }

    protected:
      int NN;
      std::string desc_name;
      boost::shared_ptr<pcl::rec_3d_framework::GlobalClassifier<pcl::PointXYZ> > global_;

    };
    /** \example objectdetector/od_pc_global_real_time.cpp
     *  \example objectdetector/od_pc_global_files.cpp
     */
  }
}

#include "ODCADDetector3DGlobal.hpp"
#endif //OPENDETECTION_ODPOINTCLOUDGLOBALMATCHINGDETECTOR_H

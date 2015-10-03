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
// Created by sarkar on 06.08.15.
//

#include "ODDetectorMultiAlgo.h"
#include "detectors/global2D/detection/ODCascadeDetector.h"
#include "detectors/global2D/detection/ODHOGDetector.h"
#include "detectors/global2D/ODFaceRecognizer.h"

#include "detectors/local2D/detection/ODCADRecognizer2DLocal.h"


//3D detectors
#include "detectors/global3D/detection/ODCADDetector3DGlobal.h"


using namespace std;
using namespace od::g2d;
using namespace od::g3d;
using namespace od::l2d;

namespace od
{

  //BASED ON 2D SCENE
  ODDetections *ODDetectorMultiAlgo2D::detect(ODSceneImage *scene)
  {
    ODDetections * detections_all = new ODDetections;
    for (int i = 0; i < detectors_2d_.size(); i++)
    {
      ODDetections * detections_individual = detectors_2d_[i]->detect(scene);
      detections_all->append(detections_individual);
    }

    return detections_all;
  }

  ODDetections2D *ODDetectorMultiAlgo2D::detectOmni(ODSceneImage *scene)
  {
    ODDetections2D * detections_all = new ODDetections2D;
    for (int i = 0; i < detectors_2d_.size(); i++)
    {
      ODDetections2D * detections_individual = detectors_2d_[i]->detectOmni(scene);
      detections_all->append(detections_individual);
    }

    return detections_all;
  }






  void ODDetectorMultiAlgo2D::init()
  {
    //make a list of different algorithms
    //vector<ODDetector *> detectors = {new ODCascadeDetector(trained_data_location_), new ODHOGDetector(trained_data_location_), new ODCADRecognizer2DLocal(trained_data_location_)};
    detectors_2d_.push_back(new ODCascadeDetector(trained_data_location_));
    detectors_2d_.push_back(new ODHOGDetector(trained_data_location_));
    //  detectors.push_back(new ODCADRecognizer2DLocal(trained_data_location_));

    for (int i = 0; i < detectors_2d_.size(); i++)
    {
      detectors_2d_[i]->init();
    }
  }




  /////############BASED ON 3D SCENE#####################

  void ODDetectorMultiAlgo::init()
  {
      //3D
    detectors_3d_.push_back(new ODCADDetector3DGlobal<PointT>(trained_data_location_, training_input_location_));

    for (int i = 0; i < detectors_3d_.size(); i++)
    {
      detectors_3d_[i]->init();
    }
  }


  ODDetections* ODDetectorMultiAlgo::detect(ODScenePointCloud<PointT> *scene)
  {
    ODDetections * detections_all = new ODDetections;
    for (int i = 0; i < detectors_3d_.size(); i++)
    {
      ODDetections * detections_individual = detectors_3d_[i]->detect(scene);
      detections_all->append(detections_individual);
    }

    return detections_all;
  }

  ODDetections3D* ODDetectorMultiAlgo::detectOmni(ODScenePointCloud<PointT> *scene)
  {
    ODDetections3D * detections_all = new ODDetections3D;
    for (int i = 0; i < detectors_3d_.size(); i++)
    {
      ODDetections3D * detections_individual = detectors_3d_[i]->detectOmni(scene);
      detections_all->append(detections_individual);
    }

    return detections_all;
  }
}
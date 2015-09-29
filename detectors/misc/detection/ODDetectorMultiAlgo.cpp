//
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
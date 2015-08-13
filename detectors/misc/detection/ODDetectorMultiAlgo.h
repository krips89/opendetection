//
// Created by sarkar on 06.08.15.
//

#ifndef OPENDETECTION_ODDETECTORMULTIALGO_H
#define OPENDETECTION_ODDETECTORMULTIALGO_H

#include <common/pipeline/ODDetector.h>
#include <common/pipeline/ODDetection.h>

namespace od
{


  class ODDetectorMultiAlgo : public ODDetector
  {
  public:
    ODDetectorMultiAlgo(std::string const &training_data_location_) : ODDetector(training_data_location_)
    { }


    typedef pcl::PointXYZRGBA PointT;



    ODDetections *detect(ODSceneImage *scene) ;
    ODDetections2D *detectOmni(ODSceneImage *scene);

    ODDetections* detect(ODScenePointCloud<PointT> *scene);
    ODDetections3D* detectOmni(ODScenePointCloud<PointT> *scene);


    void init();

  private:
    std::vector<ODDetector2D *> detectors_2d_;
    std::vector<ODDetector3D<PointT> *> detectors_3d_;
  };



}
#endif //OPENDETECTION_ODDETECTORMULTIALGO_H

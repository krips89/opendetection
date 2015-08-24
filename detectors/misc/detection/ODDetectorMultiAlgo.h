//
// Created by sarkar on 06.08.15.
//

#ifndef OPENDETECTION_ODDETECTORMULTIALGO_H
#define OPENDETECTION_ODDETECTORMULTIALGO_H

#include <common/pipeline/ODDetector.h>
#include <common/pipeline/ODDetection.h>

namespace od
{

  /** \brief Class for running multiple Detection algorithms (with default parameters) on the same scene.
   *
   * Using this class one can do object detection/recognition using multiple algorithms and provide outcome of detections (eg. people detected by HOG, face detected by Cascade, bottle detected PnPRansac in the same image).
  *
  * \author Kripasindhu Sarkar
  *
  */
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

  /** \example objectdetector/od_multialgo_files.cpp
    * \example objectdetector/od_multialgo_pc.cpp
    */

}
#endif //OPENDETECTION_ODDETECTORMULTIALGO_H

//
// Created by sarkar on 16.06.15.
//

#ifndef OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H
#define OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H

#include <common/pipeline/ODDetector.h>
#include <common/pipeline/ODTrainer.h>
#include <common/utils/utils.h>
#include <detectors/global3D/training/ODPointCloudGlobalMatchingTrainer.h>
#include <detectors/global3D/detection/ODPointCloudGlobalMatchingDetector.h>

#include "opencv2/core.hpp"
#include "opencv2/face.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"



namespace od
{

  /** \brief ODFaceRecognizer: global feature based facerecognizer
   *
   * Supports EigenFace and FischerFace currently. As an global detector, it does not perform a multiscale omni detection. It just says if a given scene represents a face.
   * To search for a face in an entire scene, you need to use a onnidetector which performs this task on multiscale on each locatoin
   *
   * \author Kripasindhu Sarkar
   *
   */

  class ODFaceRecognizer: public ObjectDetector
  {
  public:

    OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(FaceRecogType, (OD_FACE_FISCHER)(OD_FACE_EIGEN))
    ODFaceRecognizer(FaceRecogType recogtype = OD_FACE_EIGEN, int num_components = 0, double threshold = DBL_MAX):recogtype_(recogtype), num_components_(num_components), threshold_(threshold), im_height_(0), im_width_(0)
    {
    }

    void init();

    void initTrainer();

    void initDetector();

    int train();

    ODDetections2D* detect(ODSceneImage *scene);


    FaceRecogType const &getRecogtype() const
    {
      return recogtype_;
    }

    void setRecogtype(FaceRecogType const &recogtype_)
    {
      ODFaceRecognizer::recogtype_ = recogtype_;
    }

    int getThreshold() const
    {
      return threshold_;
    }

    void setThreshold(int threshold_)
    {
      ODFaceRecognizer::threshold_ = threshold_;
    }

    int getNumComponents() const
    {
      return num_components_;
    }

    void setNumComponents(int num_components_)
    {
      ODFaceRecognizer::num_components_ = num_components_;
    }

  protected:
    cv::Ptr<cv::face::FaceRecognizer> cvrecognizer_;
    FaceRecogType recogtype_;

    int im_width_;
    int im_height_;
    int num_components_;
    double threshold_;


  private:
    static void read_csv(const string& filename, vector<cv::Mat>& images, vector<int>& labels, char separator = ';');

  };
}
#endif //OPENDETECTION_ODPOINTCLOUDGLOBALMATCHING_H

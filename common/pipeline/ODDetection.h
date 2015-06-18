//
// Created by sarkar on 12.06.15.
//

#ifndef OPENDETECTION_ODDETECTION_H
#define OPENDETECTION_ODDETECTION_H

#include "iostream"
#include "common/utils/utils.h"
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

using namespace std;

namespace od
{

  /** \brief Detection: This class contains detection information; like if it is a type of recognition or a classification etc.
   * It also contains info like poses etc depending on the method of object detection
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODDetection
  {
  public:


    OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(DetectionType, (OD_DETECTION_RECOG)(OD_DETECTION_CLASS))

    void printSelf()
    {
      cout << "--Detection-- \nType: " << enumToString(type_) << endl;
      cout << "ID: " << id_ << endl;
    }

    DetectionType const &getType() const
    {
      return type_;
    }

    void setType(DetectionType const &type_)
    {
      ODDetection::type_ = type_;
    }

    string const &getId() const
    {
      return id_;
    }

    void setId(string const &id_)
    {
      ODDetection::id_ = id_;
    }

    DetectionType type_;
    string id_;
  };

  /** \brief Detection in 2D
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODDetection2D : public ODDetection
  {
  public:
    ODDetection2D()
    {
      location_ = Eigen::Vector3d::UnitZ();
    }

    Eigen::Vector3d location_;
    cv::Mat metainfo_image;
  };

  /** \brief Detection in 3D
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODDetection3D : public ODDetection
  {
  public:
    Eigen::Vector4d const &getLocation_() const
    {
      return location_;
    }

    void setLocation(Eigen::Vector4d const &location_)
    {
      ODDetection3D::location_ = location_;
    }
    void setLocation(cv::Mat const &location_)
    {
      cv::cv2eigen(location_, this->location_);
    }

    Eigen::Matrix3Xd const &getPose() const
    {
      return orientation_;
    }

    void setPose(Eigen::Matrix3Xd const &pose_)
    {
      ODDetection3D::orientation_ = pose_;
    }
    void setPose(cv::Mat const & pose_cv)
    {
      this->orientation_ = Eigen::Map<Eigen::Matrix3d>(pose_cv.clone().ptr<double>());
    }

    double getScale() const
    {
      return scale_;
    }

    void setScale(double scale)
    {
      ODDetection3D::scale_ = scale;
    }

    cv::Mat const &getMetainfoImage() const
    {
      return metainfo_image_;
    }

    void setMetainfoImage(cv::Mat const &metainfo_image)
    {
      ODDetection3D::metainfo_image_ = metainfo_image;
    }

    ODDetection3D()
    {
      location_ = Eigen::Vector4d::UnitW();
      orientation_.setIdentity();
      scale_ = 1;

    }

    void printSelf()
    {
      ODDetection::printSelf();
      cout << "Location: " << location_ << endl;
      cout << "Pose: " << orientation_ << endl;
      cout << "Scale: " << scale_ << endl;
    }

    Eigen::Vector4d location_;
    Eigen::Matrix3Xd orientation_;
    double scale_;
    cv::Mat metainfo_image_;
  };
}
#endif //OPENDETECTION_ODDETECTION_H

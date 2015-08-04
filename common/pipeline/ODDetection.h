//
// Created by sarkar on 12.06.15.
//

#ifndef OPENDETECTION_ODDETECTION_H
#define OPENDETECTION_ODDETECTION_H

#include "iostream"
#include "common/utils/utils.h"
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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


    OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(DetectionType, (OD_DETECTION_RECOG)(OD_DETECTION_CLASS)(OD_DETECTION_NULL))

    ODDetection(DetectionType const &type_ = OD_DETECTION_NULL, string const &id_ = "", double confidence_ = 1) : type_(type_), id_(id_),
                                                                                     confidence_(confidence_)
    { }

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


    double getConfidence() const
    {
      return confidence_;
    }

    void setConfidence(double confidence_)
    {
      ODDetection::confidence_ = confidence_;
    }

  private:
    DetectionType type_;
    string id_;
    double confidence_;
  };

  /** \brief Detection in 2D
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODDetection2D : public ODDetection
  {
  public:

    ODDetection2D(DetectionType const &type_ = OD_DETECTION_NULL, string const &id_ = "", double confidence_ = 1) : ODDetection(type_, id_, confidence_)
    {
      location_ = Eigen::Vector3d::UnitZ();
    }

    Eigen::Vector3d const &getLocation() const
    {
      return location_;
    }

    void setLocation(Eigen::Vector3d const &location_)
    {
      ODDetection2D::location_ = location_;
    }

    cv::Rect const &getBoundingBox() const
    {
      return bounding_box_;
    }

    void setBoundingBox(cv::Rect const &bounding_box_)
    {
      ODDetection2D::bounding_box_ = bounding_box_;
    }

    cv::Mat const &getMetainfoImage() const
    {
      return metainfo_image_;
    }

    void setMetainfoImage(cv::Mat const &metainfo_image_)
    {
      ODDetection2D::metainfo_image_ = metainfo_image_;
    }

    Eigen::Vector3d location_;
    cv::Rect bounding_box_;
    cv::Mat metainfo_image_;
  };

  /** \brief Detection in 3D
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODDetection3D : public ODDetection
  {
  public:
    Eigen::Vector4d const &getLocation() const
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

    typename pcl::PointCloud<pcl::PointXYZ>::Ptr const &getMetainfoCluster() const
    {
      return metainfo_cluster_;
    }

    void setMetainfoCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr const &metainfo_cluster_)
    {
      ODDetection3D::metainfo_cluster_ = metainfo_cluster_;
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
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr metainfo_cluster_;
  };


  class ODDetections
  {
  public:

    ODDetections (int n = 0): detections_(n)
    {
    }

    virtual ~ODDetections()
    {
      for (int i = 0; i < this->size(); i++)
        delete detections_[i];
      detections_.resize(0);
    }

    int size() { return detections_.size(); }

    void push_back(ODDetection* detection)
    {
      detections_.push_back(detection);
    }

    ODDetection * operator[](int i) { return detections_[i]; }
    ODDetection * at(int i) { return (*this)[i]; }

    cv::Mat const &getMetainfoImage() const
    {
      return metainfo_image_;
    }

    void setMetainfoImage(cv::Mat const &metainfo_image_)
    {
      this->metainfo_image_ = metainfo_image_.clone();
    }

    typename pcl::PointCloud<pcl::PointXYZ>::Ptr const &getMetainfoCluster() const
    {
      return metainfo_cluster_;
    }

    void setMetainfoCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr const &metainfo_cluster_)
    {
      this->metainfo_cluster_ = metainfo_cluster_;
    }

  protected:
    std::vector<ODDetection*> detections_;
    cv::Mat metainfo_image_;
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr metainfo_cluster_;
  };


  class ODDetections2D: public ODDetections
  {
  public:

    ODDetection2D * operator[](int i) { return static_cast<ODDetection2D *>(detections_[i]); }
    ODDetection2D * at(int i) { return static_cast<ODDetection2D *>(detections_[i]); }

  };

  class ODDetections3D: public ODDetections
  {
  public:

    ODDetection3D * operator[](int i) { return static_cast<ODDetection3D *>(detections_[i]); }
    ODDetection3D * at(int i) { return static_cast<ODDetection3D *>(detections_[i]); }
  };
}
#endif //OPENDETECTION_ODDETECTION_H

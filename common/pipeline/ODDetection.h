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
*/
//
// Created by sarkar on 12.06.15.
//

#ifndef OPENDETECTION_ODDETECTION_H
#define OPENDETECTION_ODDETECTION_H

#include "iostream"
#include "common/utils/utils.h"
#include "ODScene.h"
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/imgproc.hpp>

namespace od
{

  /** \brief The base class of all the detection.
   *
   * This is the base class of all the detection classes containing the detection information. All the ODDetector s return a collection of this class (in the form of ODDetections).
   * Supports two modes: recognition (with type OD_DETECTION_RECOG) and classification (with type OD_DETECTION_CLASS). Along with the type, ODDetector sets an ID to identify what class or what instance of recognition is detected/recognied.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODDetection
  {
  public:


    OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(DetectionType, (OD_DETECTION_RECOG)(OD_DETECTION_CLASS)(OD_DETECTION_NULL))

    virtual ~ODDetection()
    { }

    ODDetection(DetectionType const &type_ = OD_DETECTION_NULL, std::string const &id_ = "", double confidence_ = 1) : type_(type_), id_(id_),
                                                                                     confidence_(confidence_)
    { }

    void printSelf()
    {
      std::cout << "--Detection-- \nType: " << enumToString(type_) << std::endl;
      std::cout << "ID: " << id_ << std::endl;
    }

    DetectionType const &getType() const
    {
      return type_;
    }

    void setType(DetectionType const &type_)
    {
      ODDetection::type_ = type_;
    }

    std::string const &getId() const
    {
      return id_;
    }

    void setId(std::string const &id_)
    {
      ODDetection::id_ = id_;
    }


    /** \brief Get/Set the confidence of the detection. ODDetector can use this to provide confidence amnong several detections.
      */
    double getConfidence() const
    {
      return confidence_;
    }

    /** \brief Get/Set the confidence of the detection. ODDetector can use this to provide confidence amnong several detections.
      */
    void setConfidence(double confidence_)
    {
      ODDetection::confidence_ = confidence_;
    }

  private:
    DetectionType type_;
    std::string id_;
    double confidence_;
  };

  /** \brief Detection for 2D with 2D location information
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODDetection2D : public virtual ODDetection
  {
  public:

    virtual ~ODDetection2D()
    { }

    ODDetection2D(DetectionType const &type_ = OD_DETECTION_NULL, std::string const &id_ = "", double confidence_ = 1) : ODDetection(type_, id_, confidence_)
    {
      location_2d_ = Eigen::Vector3d::UnitZ();
    }

    Eigen::Vector3d const &getLocation() const
    {
      return location_2d_;
    }

    void setLocation(Eigen::Vector3d const &location_)
    {
      ODDetection2D::location_2d_ = location_;
    }

    cv::Rect const &getBoundingBox() const
    {
      return bounding_box_2d_;
    }

    void setBoundingBox(cv::Rect const &bounding_box_)
    {
      ODDetection2D::bounding_box_2d_ = bounding_box_;
    }

    cv::Mat const &getMetainfoImage() const
    {
      return metainfo_image_;
    }

    void setMetainfoImage(cv::Mat const &metainfo_image_)
    {
      ODDetection2D::metainfo_image_ = metainfo_image_;
    }


    Eigen::Vector3d location_2d_;
    cv::Rect bounding_box_2d_;
    cv::Mat metainfo_image_;
  };

  /** \brief Detection in 3D with 3D location information.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODDetection3D : public virtual ODDetection
  {
  public:
    virtual ~ODDetection3D()
    { }

    Eigen::Vector4d const &getLocation() const
    {
      return location_3d_;
    }

    void setLocation(Eigen::Vector4d const &location_)
    {
      ODDetection3D::location_3d_ = location_;
    }
    void setLocation(cv::Mat const &location_)
    {
      cv::cv2eigen(location_, this->location_3d_);
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

    ODDetection3D(DetectionType const &type_ = OD_DETECTION_NULL, std::string const &id_ = "", double confidence_ = 1) : ODDetection(type_, id_, confidence_)
    {
      location_3d_ = Eigen::Vector4d::UnitW();
      orientation_.setIdentity();
      scale_ = 1;
    }

    void printSelf()
    {
      ODDetection::printSelf();
      std::cout << "Location: " << location_3d_ << std::endl;
      std::cout << "Pose: " << orientation_ << std::endl;
      std::cout << "Scale: " << scale_ << std::endl;
    }

    Eigen::Vector4d location_3d_;
    Eigen::Matrix3Xd orientation_;
    double scale_;
    cv::Mat metainfo_image_;
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr metainfo_cluster_;
  };

  /** \brief Detection in 2D with complete information.
  *
  * \author Kripasindhu Sarkar
  *
  */
  class ODDetectionComplete: public ODDetection2D, public ODDetection3D
  {
  };

  /** \brief The container class for ODDetection
  *
  * \author Kripasindhu Sarkar
  *
  */
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

    void append(ODDetections* detections)
    {
      detections_.insert(detections_.end(), detections->detections_.begin(), detections->detections_.end());
      //note the meta information of the appended detections are lost here.
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




/** \brief The container class for ODDetection2D returned by ODDetector2D
  *
  * \author Kripasindhu Sarkar
  *
  */
  class ODDetections2D: public ODDetections
  {
  public:

    /** \brief Draws rectangles over the input image using the bounding box information present in all the 2D detections. This is a quick function to render and verify the detections made.
      */
    ODSceneImage renderMetainfo(ODSceneImage input)
    {

      //picking up random colors for different detection algorithm, if exist
      /*std::map<std::string, cv::Scalar> color_map;
      for(int i = 0; i < detections_.size(); i++)
      {
        if(color_map.find(detections_[i]->getId()) == color_map.end())
         color_map[detections_[i]->getId()] = CV_RGB(rand()%255, rand()%255, rand()%255);
      }*/

      cv::Mat image = input.getCVImage().clone();
      for(int i = 0; i < detections_.size(); i++)
      {
        ODDetection2D * detection = dynamic_cast<ODDetection2D *>(detections_[i]);
        cv::rectangle(image, detection->bounding_box_2d_, getHashedColor(detections_[i]->getId(), 100), 2);
      }
      return ODSceneImage(image);
    }


    ODDetection2D * operator[](int i) { return dynamic_cast<ODDetection2D *>(detections_[i]); }
    ODDetection2D * at(int i) { return dynamic_cast<ODDetection2D *>(detections_[i]); }

  };

  /** \brief The container class for ODDetection3D returned by ODDetector3D
  *
  * \author Kripasindhu Sarkar
  *
  */
  class ODDetections3D: public ODDetections
  {
  public:

    /*ODSceneImage renderMetainfo(ODSceneImage input)
    {
      //picking up random colors for different detection algorithm, if exist
      std::map<std::string, cv::Scalar> color_map;
      for(int i = 0; i < detections_.size(); i++)
      {
        if(color_map.find(detections_[i]->getId()) == color_map.end())
          color_map[detections_[i]->getId()] = CV_RGB(rand()%255, rand()%255, rand()%255);
      }

      cv::Mat image = input.getCVImage().clone();
      for(int i = 0; i < detections_.size(); i++)
      {
        ODDetections3D * detection = dynamic_cast<ODDetections3D *>(detections_[i]);
        cv::rectangle(image, detection->bounding_box_2d_, color_map[detections_[i]->getId()], 2);
      }
      return ODSceneImage(image);
    }*/


    ODDetection3D * operator[](int i) { return dynamic_cast<ODDetection3D *>(detections_[i]); }
    ODDetection3D * at(int i) { return dynamic_cast<ODDetection3D *>(detections_[i]); }
  };

  /** \brief The container class for ODDetectionComplete returned by ODDetector2DComplete
 *
 * \author Kripasindhu Sarkar
 *
 */
  class ODDetectionsComplete: public ODDetections
  {
  public:

    ODDetectionComplete * operator[](int i) { return dynamic_cast<ODDetectionComplete *>(detections_[i]); }
    ODDetectionComplete * at(int i) { return dynamic_cast <ODDetectionComplete *>(detections_[i]); }
  };

}
#endif //OPENDETECTION_ODDETECTION_H

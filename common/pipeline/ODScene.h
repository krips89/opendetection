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
// Created by sarkar on 10.06.15.
//

#ifndef OPENDETECTION_SCENE_H
#define OPENDETECTION_SCENE_H

//#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/smart_ptr/make_shared_object.hpp>

namespace od
{
  /** \brief Base class for Scenes. This contains information about the scenes. Scenes can be image scenes or point cloud scenes
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODScene
  {
  public:

    virtual void *getData() = 0;

    std::string const &getPath() const
    {
      return path_;
    }

  protected:
    std::string path_;
  };

  /** \brief Class for Image Scene.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODSceneImage : public ODScene
  {
  public:

    std::vector<cv::KeyPoint> const &getKeypoints() const
    {
      return keypoints_;
    }

    void setKeypoints(std::vector<cv::KeyPoint> const &keypoints_)
    {
      ODSceneImage::keypoints_ = keypoints_;
    }

    cv::Mat const &getDescriptors() const
    {
      return descriptors_;
    }

    void setDescriptors(cv::Mat const &descriptors_)
    {
      ODSceneImage::descriptors_ = descriptors_;
      is_trained_ = true;
    }

    ODSceneImage(cv::Mat const &cvimage):is_trained_(false)
    {
      this->cvimage_ = cvimage.clone();
    }

    ODSceneImage(std::string const &path):is_trained_(false)
    {
      this->cvimage_ = cv::imread(path);
      this->path_ = path;
    }

    cv::Mat getCVImage()
    {
      return cvimage_;
    }

    void *getData()
    {
      return &cvimage_;
    }

  protected:
    cv::Mat cvimage_;

    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
    bool is_trained_;

  };


  /** \brief Class for 3D scene containing point cloud.
    *
    * \author Kripasindhu Sarkar
    *
    */
  template <typename PointType = pcl::PointXYZRGBA>
  class ODScenePointCloud : public ODScene
  {

  public:
    typedef typename pcl::PointCloud<PointType>::Ptr PointCloudPtr;

    ODScenePointCloud(PointCloudPtr const &point_cloud)
    {
      point_cloud_ = point_cloud;
    }

    ODScenePointCloud(std::string point_cloud_file): point_cloud_(new pcl::PointCloud<PointType>())
    {
      if (pcl::io::loadPCDFile<PointType> (point_cloud_file, *point_cloud_ ) == -1)
      {
        std::cout<<"ERROR: Couldn't read the file "<< point_cloud_file <<std::endl;
      }
      this->path_ = point_cloud_file;
    }

    ODScenePointCloud(): point_cloud_(new pcl::PointCloud<PointType>())
    {}

    PointCloudPtr const &getPointCloud() const
    {
      return point_cloud_;
    }

    PointCloudPtr &getPointCloudRef() const
    {
      return point_cloud_;
    }

    void setPointCloud(PointCloudPtr const &point_cloud_)
    {
      ODScenePointCloud::point_cloud_ = point_cloud_;
    }

    void * getData() { return (void *)point_cloud_.get(); }
  protected:
    PointCloudPtr point_cloud_;
  };
}

#endif //OPENDETECTION_SCENE_H

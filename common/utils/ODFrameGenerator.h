//
// Created by sarkar on 19.06.15.
//

#ifndef OPENDETECTION_ODFRAMEGENERATOR_H
#define OPENDETECTION_ODFRAMEGENERATOR_H

/** \brief The FrameGenerator class for capturing and reading Scenes conveniently
   *
   * \author Kripasindhu Sarkar
   *
   */

#include "common/pipeline/ODDetection.h"
#include "common/pipeline/ODScene.h"
#include <iostream>
#include <opencv2/videoio.hpp>
#include <pcl/apps/3d_rec_framework/tools/openni_frame_source.h>

namespace od
{
  enum GeneratorType
  {
    FILE_LIST, DEVICE
  };

  template<typename SceneT, GeneratorType TYPE>
  class ODFrameGenerator
  {
  public:

    ODFrameGenerator(std::string input);

    SceneT * getNextFrame();

    bool isValid();

  protected:
    int cameraID_;
    std::vector<std::string> file_list_;
    string video_read_path_;
    int curr_image_;
    cv::VideoCapture inputCapture_;
    bool exhausted_;
  };

  template<typename SceneT>
  class ODFrameGenerator<SceneT, FILE_LIST>
  {
  public:
    ODFrameGenerator(std::string const input = "")
    {
      file_list_ = myglob(input);
      curr_image_ = -1;
      exhausted_ = false;
    }

    SceneT * getNextFrame()
    {
      if (exhausted_)
      {
        cout << "Files Exhausted!";
        return NULL;
      }
      curr_image_++;
      if(curr_image_ == file_list_.size() - 1)
        exhausted_ = true;

      cout << "Frame: " << file_list_[curr_image_] << endl;
      return new SceneT(file_list_[curr_image_]);
    }
    bool isValid() {return !exhausted_;}

    std::vector<std::string> file_list_;
    bool exhausted_;
    int curr_image_;
  };

  template<>
  class ODFrameGenerator<ODSceneImage, DEVICE>
  {
  public:
    ODFrameGenerator(std::string input = "")
    {
      input_capture_.open(atoi(input.c_str()));
      if (!input_capture_.isOpened()) { cout << "FATAL: Cannot open video capture!";}
    }

    ODSceneImage * getNextFrame()
    {
      cv::Mat frame; input_capture_.read(frame);
      return new ODSceneImage(frame);
    }
    bool isValid() {return input_capture_.isOpened();}

    cv::VideoCapture input_capture_;
  };

  /*
  template<>
  class ODFrameGenerator<ODScenePointCloud<pcl::PointXYZRGBA> , DEVICE>
  {
  public:


    ODFrameGenerator(std::string input = "")
    {

      camera_ = new OpenNIFrameSource::OpenNIFrameSource(input);

      keyboard_cb = boost::bind (&OpenNIFrameSource::OpenNIFrameSource::onKeyboardEvent, camera_, _1);
    }

    boost::function<void(pcl::visualization::KeyboardEvent const &)> const &getKeyboard_cb() const
    {
      return keyboard_cb;
    }

    ODScenePointCloud<pcl::PointXYZRGBA> * getNextFrame()
    {
      OpenNIFrameSource::PointCloudPtr frame = camera_->snap();

      return new ODScenePointCloud<pcl::PointXYZRGBA>(frame);
    }
    bool isValid() {return camera_->isActive();}

    OpenNIFrameSource::OpenNIFrameSource *camera_;
    boost::function<void (const pcl::visualization::KeyboardEvent&)> keyboard_cb;
  };
*/


  template<>
  class ODFrameGenerator<ODScenePointCloud<pcl::PointXYZRGBA> , DEVICE>
  {
  public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    /* A simple class for capturing data from an OpenNI camera */
    ODFrameGenerator(std::string input = "") : grabber_(input), most_recent_frame_(), frame_counter_(0), active_(true)
    {
      boost::function<void(const PointCloudConstPtr&)> frame_cb = boost::bind (&ODFrameGenerator<ODScenePointCloud<pcl::PointXYZRGBA> , DEVICE>::onNewFrame, this, _1);
      grabber_.registerCallback (frame_cb);
      grabber_.start ();
      boost::this_thread::sleep (boost::posix_time::seconds(5));
    }

    ~ODFrameGenerator()
    {
      // Stop the grabber when shutting down
      grabber_.stop ();
    }

    ODScenePointCloud<pcl::PointXYZRGBA> * getNextFrame()
    {
      OpenNIFrameSource::PointCloudPtr frame = snap();

      return new ODScenePointCloud<pcl::PointXYZRGBA>(frame);
    }

    bool isValid() {return isActive();}

    const PointCloudPtr snap ()
    {
      return (most_recent_frame_);
    }

    bool isActive ()
    {
      return active_;
    }
    void onKeyboardEvent (const pcl::visualization::KeyboardEvent & event)
    {
      // When the spacebar is pressed, trigger a frame capture
      mutex_.lock ();
      if (event.keyDown () && event.getKeySym () == "e")
      {
        active_ = false;
      }
      mutex_.unlock ();
    }

  protected:
    void onNewFrame (const PointCloudConstPtr &cloud)
    {
      mutex_.lock ();
      ++frame_counter_;
      most_recent_frame_ = boost::make_shared < PointCloud > (*cloud); // Make a copy of the frame
      mutex_.unlock ();
    }

    pcl::OpenNIGrabber grabber_;
    PointCloudPtr most_recent_frame_;
    int frame_counter_;
    boost::mutex mutex_;
    bool active_;

  };

}
//
#endif //OPENDETECTION_ODFRAMEGENERATOR_H

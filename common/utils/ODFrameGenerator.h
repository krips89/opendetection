//
// Created by sarkar on 19.06.15.
//

#ifndef OPENDETECTION_ODFRAMEGENERATOR_H
#define OPENDETECTION_ODFRAMEGENERATOR_H

#include "common/pipeline/ODDetection.h"
#include "common/pipeline/ODScene.h"
#include <iostream>
#include <opencv2/videoio.hpp>
#include <pcl/apps/3d_rec_framework/tools/openni_frame_source.h>

namespace od
{
  enum GeneratorType
  {
    GENERATOR_TYPE_FILE_LIST, GENERATOR_TYPE_DEVICE
  };



  /** \brief The FrameGenerator class for capturing and reading Scenes conveniently.
   * Templated with two parameters - SceneType identifying a Scene class, and TYPE identifying the type of input. After the instantiation with a correct TYPE, use the function
   * getNextFrame() to get an instance of next scene of SceneType. getNextFrame() returns valid scenes until all scenes matched are exhausted - the time  when 'isValid()' is false.
   *
   * \tparam SceneT One of the Scene classes - ODSceneImage or ODScenePointCloud
   * \tparam TYPE TYPE can be GENERATOR_TYPE_FILE_LIST which means you provide the list of scene files to be returned by the FrameGenerator in the constructor (for eg. \/home/username/pics/*.jpg)
   * Or it can be GENERATOR_TYPE_DEVICE which picks up the webcam or the kinect based on the SceneType.
 * \author Kripasindhu Sarkar
 *
 */
  template<typename SceneT, GeneratorType TYPE>
  class ODFrameGenerator
  {
  public:

    ODFrameGenerator(std::string input = "");
    ODFrameGenerator(int input = 0){}

    SceneT * getNextFrame();

    bool isValid();

  protected:
    int cameraID_;
    std::vector<std::string> file_list_;
    std::string video_read_path_;
    int curr_image_;
    cv::VideoCapture inputCapture_;
    bool exhausted_;
  };


  template<typename SceneT>
  class ODFrameGenerator<SceneT, GENERATOR_TYPE_FILE_LIST>
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
    std::string currentFile()
    {
      return file_list_[curr_image_];
    }
    std::vector<std::string> file_list_;
    bool exhausted_;
    int curr_image_;
  };

  template<>
  class ODFrameGenerator<ODSceneImage, GENERATOR_TYPE_DEVICE>
  {
  public:
    ODFrameGenerator(std::string input = "")
    {
      input_capture_.open(input);
      if (!input_capture_.isOpened()) { cout << "FATAL: Cannot open video capture!";}
    }

    ODFrameGenerator(int input = 0)
    {
      input_capture_.open(input);
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
  class ODFrameGenerator<ODScenePointCloud<pcl::PointXYZRGBA> , GENERATOR_TYPE_DEVICE>
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
  class ODFrameGenerator<ODScenePointCloud<pcl::PointXYZRGBA> , GENERATOR_TYPE_DEVICE>
  {
  public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    /* A simple class for capturing data from an OpenNI camera */
    ODFrameGenerator(std::string input = "") : grabber_(input), most_recent_frame_(), frame_counter_(0), active_(true)
    {
      boost::function<void(const PointCloudConstPtr&)> frame_cb = boost::bind (&ODFrameGenerator<ODScenePointCloud<pcl::PointXYZRGBA> , GENERATOR_TYPE_DEVICE>::onNewFrame, this, _1);
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

//
// Created by sarkar on 10.06.15.
//

#ifndef OPENDETECTION_SCENE_H
#define OPENDETECTION_SCENE_H

#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

using namespace std;

namespace od
{
  /** \brief This contains information about the scenes. Scenes can be image scene or point cloud scene
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODScene
  {
    virtual void * getData() = 0;
  };

  /** \brief Image scene
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODSceneImage : public ODScene
  {
  public:

    ODSceneImage(cv::Mat const &cvimage)
    {
      this->cvimage_ = cvimage.clone();
    }

    ODSceneImage(string const &path)
    {
      this->cvimage_ = cv::imread(path);
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
  };

}

#endif //OPENDETECTION_SCENE_H

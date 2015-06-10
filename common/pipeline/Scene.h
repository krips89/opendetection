//
// Created by sarkar on 10.06.15.
//

#ifndef OPENDETECTION_SCENE_H
#define OPENDETECTION_SCENE_H

#include <opencv2/core/core.hpp>

namespace od
{
  class Scene
  {

  };

  class ImageScene: public Scene
  {
  public:

    ImageScene(cv::Mat const &cvimage)
    {
      this->cvimage_ = cvimage.clone();
    }

  protected:
    cv::Mat cvimage_;
  };

}

#endif //OPENDETECTION_SCENE_H

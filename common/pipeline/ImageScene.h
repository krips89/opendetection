//
// Created by sarkar on 05.06.15.
//

#ifndef OPENDETECTION_IMAGESCENE_H
#define OPENDETECTION_IMAGESCENE_H

#include "ObjectDetector.h"

#include <opencv2/core/core.hpp>

namespace od
{

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
#endif //OPENDETECTION_IMAGESCENE_H

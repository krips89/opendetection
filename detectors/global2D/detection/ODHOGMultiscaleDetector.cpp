//
// Created by sarkar on 15.07.15.
//

#include "ODHOGMultiscaleDetector.h"

namespace od
{
  namespace g2d
  {


    ODDetections2D *ODHOGMultiscaleDetector::detectOmni(ODSceneImage *scene)
    {

      vector<cv::Rect> found, found_filtered;
      hog_.detectMultiScale(scene->getCVImage(), found, 0, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);

      size_t i, j;
      for(i = 0; i < found.size(); i++)
      {
        cv::Rect r = found[i];
        for(j = 0; j < found.size(); j++)
          if(j != i && (r & found[j]) == r)
            break;
        if(j == found.size())
          found_filtered.push_back(r);
      }

      //always create detections
      ODDetections2D *detections = new ODDetections2D;

      cv::Mat viz = scene->getCVImage().clone();
      for(i = 0; i < found_filtered.size(); i++)
      {

        ODDetection2D *detections2D = new ODDetection2D;
        detections2D->setBoundingBox(found_filtered[i]);
        detections2D->setId("PEOPLE");
        detections2D->setType(ODDetection::OD_DETECTION_CLASS);
        detections->push_back(detections2D);

        if(metainfo_)
        {
          cv::Rect r = found_filtered[i];
          r.x += cvRound(r.width * 0.1);
          r.width = cvRound(r.width * 0.8);
          r.y += cvRound(r.height * 0.06);
          r.height = cvRound(r.height * 0.9);
          rectangle(viz, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
        }
      }
    }


  }
}

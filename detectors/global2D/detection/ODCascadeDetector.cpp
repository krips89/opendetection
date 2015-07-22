//
// Created by sarkar on 17.07.15.
//

#include "ODCascadeDetector.h"

using namespace cv;

namespace od
{

  namespace g2d
  {
    ODDetections2D *ODCascadeDetector::detectOmni(ODSceneImage *scene)
    {
      Mat gray;
      cvtColor(scene->getCVImage(), gray, CV_BGR2GRAY);
      // Find the faces in the frame:
      vector<Rect_<int> > faces;
      haar_cascade_->detectMultiScale(gray, faces, scaleFactor_, minNeighbors_, 0, minSize_, maxSize_);

      //always create detections
      ODDetections2D *detections = new ODDetections2D;
      cv::Mat viz = scene->getCVImage().clone();

      for(int i = 0; i < faces.size(); i++)
      {
        // Process face by face:
        Rect face_i = faces[i];

        ODDetection2D *detection2D = new ODDetection2D(ODDetection::OD_DETECTION_CLASS, "FACE", 1);
        detection2D->setBoundingBox(face_i);
        detections->push_back(detection2D);

        if(metainfo_)
        {
          rectangle(viz, face_i, CV_RGB(0, 255, 0), 1);
        }
      }
      detections->setMetainfoImage(viz);

      return detections;
    }
  }
}
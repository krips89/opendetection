//
// Created by sarkar on 17.07.15.
//

#include "ODCascadeDetector.h"

using namespace cv;
using namespace std;

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

    ODDetections* ODCascadeDetector::detect(ODSceneImage *scene)
    {
      //always create detections
      ODDetections *detections = new ODDetections;

      Mat gray;
      cvtColor(scene->getCVImage(), gray, CV_BGR2GRAY);
      // Find the faces in the frame:
      vector<Rect_<int> > faces;


      cv::Size imsize = gray.size();
      //hack for single detection,
      //note: maxsize = minsize = size of input image for single window detection
      //todo: implement in some other way of fast single detection; currently this will work, but maynot be fast
      haar_cascade_->detectMultiScale(gray, faces, 5, minNeighbors_, 0, gray.size(), gray.size());
      if (faces.size() > 0)
      {
        ODDetection *detection = new ODDetection(ODDetection::OD_DETECTION_CLASS, "FACE", 1);
        detections->push_back(detection);
      }
      return detections;
    }

  }
}
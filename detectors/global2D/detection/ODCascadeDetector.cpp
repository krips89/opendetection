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
*///
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
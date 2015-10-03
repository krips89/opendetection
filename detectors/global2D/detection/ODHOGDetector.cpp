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
// Created by sarkar on 15.07.15.
//

#include "ODHOGDetector.h"

using namespace std;

namespace od
{
  namespace g2d
  {

    void ODHOGDetector::init()
    {

      switch(svmtype_)
      {
        case OD_DEFAULT_PEOPLE:
          hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
          cout << "HOG TYPE: OpenCV Default People" << endl;
          //hog_.save(getSpecificTrainingDataLocation() + "/defaultpeople." + TRAINED_DATA_EXT_);
          break;
        case OD_DAIMLER_PEOPLE:
          hog_.winSize = cv::Size(48, 96);
          hitThreshold =1.2;
          hog_.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
          cout << "HOG TYPE: OpenCV Daimler People" << endl;
          //hog_.save(getSpecificTrainingDataLocation() + "/daimlerpeople." + TRAINED_DATA_EXT_);
          break;
        case OD_FILE:
          string hogfile = FileUtils::getFirstFile(getSpecificTrainingDataLocation(), TRAINED_DATA_ID_);
          load(hogfile);
          cout << "HOG TYPE: Custom HOG features loaded from: " << hogfile << endl;
          break;
          //dont set anything for custom, it is to be set by the user by setSVMDetector
      }

      printParameters();
    }

    void ODHOGDetector::load(std::string filename)
    {
      cv::FileStorage fs(filename, cv::FileStorage::READ);
      fs["hitThreshold"] >> hitThreshold;
      cv::FileNode fn = fs[cv::FileStorage::getDefaultObjectName(filename)];
      hog_.read(fn);
    }

    ODDetections2D *ODHOGDetector::detectOmni(ODSceneImage *scene)
    {
      //always create a detection
      ODDetections2D *detections = new ODDetections2D;


      vector<cv::Rect> found, found_filtered;
      hog_.detectMultiScale(scene->getCVImage(), found, hitThreshold, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);


      cv::Mat viz = scene->getCVImage().clone();
      for(int i = 0; i < found.size(); i++)
      {
        ODDetection2D *detection2D = new ODDetection2D;
        detection2D->setBoundingBox(found[i]);
        detection2D->setId("PEOPLE");
        detection2D->setType(ODDetection::OD_DETECTION_CLASS);
        detections->push_back(detection2D);


        if(metainfo_)
        {
          cv::Rect r = found[i];
          r.x += cvRound(r.width * 0.1);
          r.width = cvRound(r.width * 0.8);
          r.y += cvRound(r.height * 0.06);
          r.height = cvRound(r.height * 0.9);
          rectangle(viz, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
        }
      }
      detections->setMetainfoImage(viz);

      return detections;
    }

    ODDetections *ODHOGDetector::detect(ODSceneImage *scene)
    {
      //always create a detection
      ODDetections *detections = new ODDetections;

      cv::Mat scaledwindow;
      cv::resize(scene->getCVImage(), scaledwindow, hog_.winSize);

      std::vector<cv::Point> foundLocations;

      hog_.detect(scene->getCVImage(), foundLocations, hitThreshold);
      if (!foundLocations.empty())
      {
        ODDetection2D *detection2D = new ODDetection2D;
        detection2D->setId("PEOPLE");
        detection2D->setType(ODDetection::OD_DETECTION_CLASS);
        detections->push_back(detection2D);
      }

      return detections;
    }

    void ODHOGDetector::setSVMFromFile(std::string fileName)
    {
      vector<float> descriptor_vector;
      printf("Reading descriptor vector from file '%s'\n", fileName.c_str());
      string separator = " "; // Use blank as default separator between single features

      ifstream File;
      float percent;
      File.open(fileName.c_str(), ios::in);
      if (File.good() && File.is_open()) {

        double d;
        while(File >> d)
        {
          //cout << d << " ";
          descriptor_vector.push_back(d);
        }
        File.close();
      }

      hog_.setSVMDetector(descriptor_vector);
    }

    void ODHOGDetector::printParameters()
    {
      cout << "winSize: " << winSize << endl;
      cout << "blockSize: " << blockSize << endl;
      cout << "blockStride: " << blockStride << endl;
      cout << "cellSize: " << cellSize << endl;
      cout << "hitThreshold: " << hitThreshold << endl;
    }
  }
}

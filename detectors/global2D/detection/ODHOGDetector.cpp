//
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

          FileUtils::createTrainingDir(getSpecificTrainingDataLocation());
          hog_.save(getSpecificTrainingDataLocation() + "/defaultpeople." + TRAINED_DATA_EXT_);
          break;
        case OD_DAIMLER_PEOPLE:
          hog_.winSize = cv::Size(48, 96);
          hog_.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());

          FileUtils::createTrainingDir(getSpecificTrainingDataLocation());
          hog_.save(getSpecificTrainingDataLocation() + "/daimlerpeople." + TRAINED_DATA_EXT_);
          break;
        case OD_FILE:
          hog_.load(FileUtils::getFirstFile(getSpecificTrainingDataLocation(), TRAINED_DATA_EXT_));
          break;
          //dont set anything for custom, it is to be set by the user by setSVMDetector
      }
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

      hog_.detect(scene->getCVImage(), foundLocations);
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

  }
}

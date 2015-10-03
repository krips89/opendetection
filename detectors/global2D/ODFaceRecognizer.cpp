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
// Created by sarkar on 16.07.15.
//

#include "ODFaceRecognizer.h"

using namespace cv;
using namespace std;

namespace od
{
  namespace g2d
  {
    void ODFaceRecognizer::init()
    {
      switch(recogtype_)
      {
        case OD_FACE_FISCHER:
          cvrecognizer_ = cv::face::createFisherFaceRecognizer(num_components_, threshold_);
          break;
        case OD_FACE_EIGEN:
          cvrecognizer_ = cv::face::createEigenFaceRecognizer(num_components_, threshold_);
          break;
        default:
          std::cout << "FATAL: FACETYPE NOT FOUND!";
      }
    }

    void ODFaceRecognizer::initTrainer()
    {
      init();
    }

    void ODFaceRecognizer::initDetector()
    {
      if(!trained_)
      {
        init();

        //get models in the directory
        std::vector<std::string> files;
        FileUtils::getFilesInDirectoryRec(getSpecificTrainingDataLocation(), TRAINED_DATA_EXT_, files);

        if (files.size() == 0)
        {
          std::cout << "FATAL: Trained data not found" << endl;
          exit(1);
        }

        //choose the first
        cvrecognizer_->load(files[0]);
      }
    }

    int ODFaceRecognizer::train()
    {
      vector<cv::Mat> images;
      vector<int> labels;
      try
      {
        read_csv(training_input_location_, images, labels);
      } catch(cv::Exception &e)
      {
        cerr << "Error opening file \"" << training_input_location_ << "\". Reason: " << e.msg << endl;
        exit(1);
      }
      cvrecognizer_->train(images, labels);
      FileUtils::createTrainingDir(getSpecificTrainingDataLocation());

      cvrecognizer_->save(getSpecificTrainingDataLocation() + "/trained." + TRAINED_DATA_EXT_);
      trained_ = true;

      //the training set has atleast one image
      im_width_ = images[0].cols;
      im_height_ = images[0].rows;
    }

    ODDetections *ODFaceRecognizer::detect(ODSceneImage *scene)
    {
      cv::Mat face_edited;
      cvtColor(scene->getCVImage(), face_edited, CV_BGR2GRAY);

      if(trained_)
      {
        cv::resize(face_edited.clone(), face_edited, cv::Size(im_width_, im_height_), 1.0, 1.0, cv::INTER_CUBIC);
      }

      int label = 100;
      double confidence;
      cvrecognizer_->predict(face_edited, label, confidence);

      //fill in the detection
      ODDetection2D *detection = new ODDetection2D(ODDetection::OD_DETECTION_CLASS, toString(label), confidence);
      ODDetections2D *detections = new ODDetections2D;
      detections->push_back(detection);
      return detections;
    }


    void ODFaceRecognizer::read_csv(const string &filename, vector<cv::Mat> &images, vector<int> &labels, char separator)
    {
      std::ifstream file(filename.c_str(), ifstream::in);
      if(!file)
      {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
      }
      string line, path, classlabel;
      while(getline(file, line))
      {
        stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty())
        {
          images.push_back(cv::imread(path, 0));
          labels.push_back(atoi(classlabel.c_str()));
        }
      }
    }
    
  }
}
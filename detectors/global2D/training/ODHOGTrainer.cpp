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
// Created by sarkar on 13.08.15.
//


#include "ODHOGTrainer.h"


#include <fstream>
#include <algorithm>
#include <iterator>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//binding class for svmlight
#include "common/bindings/svmlight.h"

#define TRAINHOG_SVM_TO_TRAIN SVMlight

using namespace std;
using namespace cv;

namespace od
{
  namespace g2d
  {

    static void storeCursor(void)
    {
      printf("\033[s");
    }

    static void resetCursor(void)
    {
      printf("\033[u");
    }

    void ODHOGTrainer::saveDescriptorVectorToFile(vector<float> &descriptorVector, vector<unsigned int> &_vectorIndices, string fileName)
    {
      printf("Saving descriptor vector to file '%s'\n", fileName.c_str());
      string separator = " "; // Use blank as default separator between single features
      fstream File;
      float percent;
      File.open(fileName.c_str(), ios::out);
      if(File.good() && File.is_open())
      {
        printf("Saving %lu descriptor vector features:\t", descriptorVector.size());
        storeCursor();
        for(int feature = 0; feature < descriptorVector.size(); ++feature)
        {
          if((feature % 10 == 0) || (feature == (descriptorVector.size() - 1)))
          {
            percent = ((1 + feature) * 100 / descriptorVector.size());
            printf("%4u (%3.0f%%)", feature, percent);
            fflush(stdout);
            resetCursor();
          }
          File << descriptorVector.at(feature) << separator;
        }
        printf("\n");
        File << endl;
        File.flush();
        File.close();
      }
    }

    void ODHOGTrainer::calculateFeaturesFromInput(const string &imageFilename, vector<float> &featureVector, HOGDescriptor &hog)
    {

      Mat imageDataorig, imageData;
      imageDataorig = imread(imageFilename, 0);


      if(imageDataorig.empty())
      {
        featureVector.clear();
        printf("Error: HOG image '%s' is empty, features calculation skipped!\n", imageFilename.c_str());
        return;
      }

      vector<Point> locations;
      locations.push_back(start_hog_pos);
      hog.compute(imageDataorig, featureVector, winStride, trainingPadding, locations);
      //cout << "Desc size :" << featureVector.size();
      //cout << ": Expected size :" << hog.getDescriptorSize() << endl;
      imageDataorig.release(); // Release the image again after features are extracted
    }

    void ODHOGTrainer::detectTrainingSetTest(const HOGDescriptor &hog, const double hitThreshold, const vector<string> &posFileNames, const vector<string> &negFileNames)
    {
      unsigned int truePositives = 0;
      unsigned int trueNegatives = 0;
      unsigned int falsePositives = 0;
      unsigned int falseNegatives = 0;
      vector<Point> foundDetection;
      // Walk over positive training samples, generate images and detect
      for(vector<string>::const_iterator posTrainingIterator = posFileNames.begin();
          posTrainingIterator != posFileNames.end(); ++posTrainingIterator)
      {
        const Mat imageData = imread(*posTrainingIterator, 0);
        hog.detect(imageData, foundDetection, hitThreshold, winStride, trainingPadding);
        if(foundDetection.size() > 0)
        {
          ++truePositives;
          falseNegatives += foundDetection.size() - 1;
        } else
        {
          ++falseNegatives;
        }
      }
      // Walk over negative training samples, generate images and detect
      for(vector<string>::const_iterator negTrainingIterator = negFileNames.begin();
          negTrainingIterator != negFileNames.end(); ++negTrainingIterator)
      {
        const Mat imageData = imread(*negTrainingIterator, 0);
        hog.detect(imageData, foundDetection, hitThreshold, winStride, trainingPadding);
        if(foundDetection.size() > 0)
        {
          falsePositives += foundDetection.size();
        } else
        {
          ++trueNegatives;
        }
      }

      printf("Results:\n\tTrue Positives: %u\n\tTrue Negatives: %u\n\tFalse Positives: %u\n\tFalse Negatives: %u\n",
             truePositives, trueNegatives, falsePositives, falseNegatives);
    }


    void ODHOGTrainer::calculateFeaturesFromImageLoc(const Mat &imageData, vector<float> &featureVector, const HOGDescriptor &hog, Point startpos)
    {

      vector<Point> locations;
      locations.push_back(startpos);
      hog.compute(imageData, featureVector, winStride, trainingPadding, locations);
    }

    void ODHOGTrainer::handleNegetivefile(string const imageFilename, HOGDescriptor &hog, fstream &file)
    {

      Mat imageData;
      imageData = imread(imageFilename, 0);

      //cout << "Image size : " << imageData.size() << "random points : " << endl;
      //get hog at random 10 image location
      for(int i = 0; i < nofeatures_neg; i++)
      {
        Point feat_loc(rand() % (imageData.cols - winSize.width), rand() % (imageData.rows - winSize.height));
        //cout << feat_loc << endl;

        //use this p for finding feature
        vector<float> featureVector;
        calculateFeaturesFromImageLoc(imageData, featureVector, hog, feat_loc);

        if(!featureVector.empty())
        {
          /* Put positive or negative sample class to file,
       * true=positive, false=negative,
       * and convert positive class to +1 and negative class to -1 for SVMlight
       */
          file << "-1";
          // Save feature vector components
          for(unsigned int feature = 0; feature < featureVector.size(); ++feature)
          {
            file << " " << (feature + 1) << ":" << featureVector.at(feature);
          }
          file << endl;
        }
      }

    }

    void ODHOGTrainer::createHardTrainingData(const HOGDescriptor &hog, const double hitThreshold, const vector<string> &negFileNames)
    {
      fstream file;
      file.open(featuresFile.c_str(), std::fstream::app);
      if(!file.is_open())
      {
        cout << "ERROR opening previous feature file! HARD training failed!" << endl;
      }
      cout << "Appending HARD negetive features to " + featuresFile << endl;
      //cvNamedWindow("hardneg", WINDOW_AUTOSIZE);

      // Walk over negative training samples, generate images and detect
      int counter = 0;
      for(vector<string>::const_iterator negTrainingIterator = negFileNames.begin();
          negTrainingIterator != negFileNames.end(); ++negTrainingIterator)
      {

        const Mat imageData = imread(*negTrainingIterator, 0);


        vector<Rect> foundLocations;
        hog.detectMultiScale(imageData, foundLocations, hitThreshold);


        for(int i = 0; i < foundLocations.size(); i++)
        {
          counter++;
          //cout << "## hard examples ## FALSE POS FOUND : including the descriptor in training set" << endl;

          Mat negimg(imageData(foundLocations[i]));
          //imshow("hardneg", negimg); waitKey(2000);

          Mat resized_neg;
          resize(negimg, resized_neg, winSize);
          // imshow("hardneg", resized_neg); waitKey(4000);

          vector<float> featureVector;
          calculateFeaturesFromImageLoc(resized_neg, featureVector, hog, Point(0, 0));

          if(!featureVector.empty())
          {
            file << "-1";
            for(unsigned int feature = 0; feature < featureVector.size(); ++feature)
            {
              file << " " << (feature + 1) << ":" << featureVector.at(feature);
            }
            file << endl;
          }
        }
      }
      cout << "Wrote " << counter << " HARD negetive features" << endl;

      //CLOSE THE APPENDED DATA FILE
      file.close();
    }

    double ODHOGTrainer::trainWithSVMLight(string svmModelFile, string svmDescriptorFile, vector<float> &descriptorVector)
    {
      //training takes featurefile as input, produces hitthreshold and vector as output
      printf("Calling %s\n", TRAINHOG_SVM_TO_TRAIN::getInstance()->getSVMName());
      TRAINHOG_SVM_TO_TRAIN::getInstance()->read_problem(const_cast<char *> (featuresFile.c_str()));
      TRAINHOG_SVM_TO_TRAIN::getInstance()->train(); // Call the core libsvm training procedure
      printf("Training done, saving model file!\n");
      TRAINHOG_SVM_TO_TRAIN::getInstance()->saveModelToFile(svmModelFile);

      printf("Generating representative single HOG feature vector using svmlight!\n");

      descriptorVector.resize(0);
      vector<unsigned int> descriptorVectorIndices;
      // Generate a single detecting feature vector (v1 | b) from the trained support vectors, for use e.g. with the HOG algorithm
      TRAINHOG_SVM_TO_TRAIN::getInstance()->getSingleDetectingVector(descriptorVector, descriptorVectorIndices);
      // And save the precious to file system
      saveDescriptorVectorToFile(descriptorVector, descriptorVectorIndices, svmDescriptorFile);
      // Detector detection tolerance threshold
      return hitThreshold = TRAINHOG_SVM_TO_TRAIN::getInstance()->getThreshold();
    }

    int ODHOGTrainer::train()
    {

      vector<string> positiveTrainingImages;
      vector<string> negativeTrainingImages;
      vector<string> validExtensions;
      validExtensions.push_back(".jpg");
      validExtensions.push_back(".png");
      validExtensions.push_back(".ppm");

      FileUtils::getFilesInDirectoryRec(posSamplesDir, validExtensions, positiveTrainingImages);
      FileUtils::getFilesInDirectoryRec(negSamplesDir, validExtensions, negativeTrainingImages);
      cout << "No of positive Training Files: " << positiveTrainingImages.size() << endl;
      cout << "No of neg Training Files: "<< negativeTrainingImages.size() << endl;


      if( positiveTrainingImages.size() + negativeTrainingImages.size() == 0)
      {
        printf("No training sample files found, nothing to do!\n");
        return EXIT_SUCCESS;
      }

      /// @WARNING: This is really important, some libraries (e.g. ROS) seems to set the system locale which takes decimal commata instead of points which causes the file input parsing to fail
      setlocale(LC_ALL, "C"); // Do not use the system locale
      setlocale(LC_NUMERIC, "C");
      setlocale(LC_ALL, "POSIX");


      printf("Reading files, generating HOG features and save them to file '%s':\n", featuresFile.c_str());
      float percent;


      fstream File;
      File.open(featuresFile.c_str(), std::fstream::out);
      if(File.is_open())
      {

        // Iterate over POS IMAGES
        for(unsigned long currentFile = 0; currentFile < positiveTrainingImages.size(); ++currentFile)
        {

          vector<float> featureVector;
          const string currentImageFile = positiveTrainingImages.at(currentFile);


          calculateFeaturesFromInput(currentImageFile, featureVector, hog_);
          if(!featureVector.empty())
          {
            /* Put positive or negative sample class to file,
            * true=positive, false=negative,
            * and convert positive class to +1 and negative class to -1 for SVMlight
            */
            File << "+1";
            // Save feature vector components
            for(unsigned int feature = 0; feature < featureVector.size(); ++feature)
            {
              File << " " << (feature + 1) << ":" << featureVector.at(feature);
            }
            File << endl;
          }
        }


        // Iterate over NEG IMAGES
        for(unsigned long currentFile = 0; currentFile < negativeTrainingImages.size(); ++currentFile)
        {
          const string currentImageFile = negativeTrainingImages.at(currentFile);
          handleNegetivefile(currentImageFile, hog_, File);
        }

        printf("\n");
        File.flush();
        File.close();

      } else
      {
        printf("Error opening file '%s'!\n", featuresFile.c_str());
        return EXIT_FAILURE;
      }


      //train them with SVM
      vector<float> descriptorVector;
      hitThreshold = trainWithSVMLight(svmModelFile, descriptorVectorFile, descriptorVector);

      // Pseudo test our custom detecting vector
      hog_.setSVMDetector(descriptorVector);
      printf(
          "Testing training phase using training set as test set (just to check if training is ok - no detection quality conclusion with this!)\n");
      detectTrainingSetTest(hog_, hitThreshold, positiveTrainingImages, negativeTrainingImages);



      if(train_hard_negetive_)
      {
        cout << "Preparing for training HARD negetive windows" << endl;
        //create hard training examples
        createHardTrainingData(hog_, hitThreshold, negativeTrainingImages);
        //train again
        hitThreshold = trainWithSVMLight(svmModelHard, descriptorVectorHard, descriptorVector);

        // Pseudo test our custom detecting vector
        hog_.setSVMDetector(descriptorVector);
        printf(
            "Testing training phase using training set as test set after HARD EXAMPLES (just to check if training is ok - no detection quality conclusion with this!)\n");
        detectTrainingSetTest(hog_, hitThreshold, positiveTrainingImages, negativeTrainingImages);

      }

      save(getSpecificTrainingDataLocation() + "/odtrained." + TRAINED_DATA_ID_);

      return EXIT_SUCCESS;
    }


    void ODHOGTrainer::readDescriptorsFromFile(string fileName, vector<float> &descriptor_vector)
    {
      printf("Reading descriptor vector from file '%s'\n", fileName.c_str());
      string separator = " "; // Use blank as default separator between single features

      ifstream File;
      float percent;
      File.open(fileName.c_str(), ios::in);
      if(File.good() && File.is_open())
      {

        double d;
        while(File >> d)
        {
          //cout << d << " ";
          descriptor_vector.push_back(d);
        }
        File.close();
      }
    }

    void ODHOGTrainer::save(std::string filename)
    {
      FileStorage fs(filename, FileStorage::WRITE);
      fs << "hitThreshold" << hitThreshold;
      hog_.write(fs, FileStorage::getDefaultObjectName(filename));
    }

  }
}

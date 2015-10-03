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

#ifndef OPENDETECTION_ODHOGTRAINER_H
#define OPENDETECTION_ODHOGTRAINER_H

#include <opencv2/objdetect.hpp>
#include "common/pipeline/ODTrainer.h"
#include "common/utils/utils.h"




namespace od
{
  namespace g2d
  {
    /** \brief Class for training HOG based detector.
     *
     * Use ODHOGDetector after training with this class. This is the training class for training HOG based detector. SVMlight is used here to train linear SVM on the HOG features. It supports the usage of multiple random windows in negetive training images
     * to increase the number of negetive features by the function 'setNOFeaturesNeg'. It also supports "Hard negetive" training which collects all the false positive
     * windows after initial training to retrain and obtain a new feature vector. Use the function 'setTrainHardNegetive' to enable this feature.
     *
     * \author Kripasindhu Sarkar
     */

    class ODHOGTrainer : public ODTrainer
    {

    public:
      ODHOGTrainer(std::string const &training_input_location_ = "", std::string const &trained_data_location_ = "", cv::Size winsize = cv::Size(64,128), cv::Size blocksize = cv::Size(16,16), cv::Size blockstride = cv::Size(8,8), cv::Size cellsize = cv::Size(8,8), float hitshreshold = 0.0):
          ODTrainer(training_input_location_, trained_data_location_),  winSize(winsize), blockSize(blocksize), blockStride(blockstride),
          cellSize(cellsize), hog_(winSize, blockSize, blockStride, cellSize, 9)
      {

        TRAINED_LOCATION_DENTIFIER_ = "HOG";
        TRAINED_DATA_ID_ = "hog.xml";

        //algo parameter init
        trainingPadding = cv::Size(0, 0);
        start_hog_pos = cv::Point(15, 15);
        nofeatures_neg = 10;
        winStride = cv::Size();
        train_hard_negetive_ = false;

        if(trained_data_location_ != "")
        {
          posSamplesDir = training_input_location_ + "/pos";
          negSamplesDir = training_input_location_ + "/neg";
        }


        //internal data
        FileUtils::createTrainingDir(getSpecificTrainingDataLocation());
        featuresFile = getSpecificTrainingDataLocation() + "/features.dat";
        svmModelFile = getSpecificTrainingDataLocation() + "/svmlightmodel.dat";
        svmModelHard = getSpecificTrainingDataLocation() + "/svmlightmodelhard.dat";
        descriptorVectorFile = getSpecificTrainingDataLocation() + "/descriptorvector.dat";
        descriptorVectorFile = getSpecificTrainingDataLocation() + "/descriptorvectorHard.dat";


      }

      int train();

      void init() {}

      std::string const &getPosSamplesDir() const
      {
        return posSamplesDir;
      }

      void setPosSamplesDir(std::string const &posSamplesDir)
      {
        ODHOGTrainer::posSamplesDir = posSamplesDir;
      }

      std::string const &getNegSamplesDir() const
      {
        return negSamplesDir;
      }

      void setNegSamplesDir(std::string const &negSamplesDir)
      {
        ODHOGTrainer::negSamplesDir = negSamplesDir;
      }

      int getNOFeaturesNeg() const
      {
        return nofeatures_neg;
      }

      void setNOFeaturesNeg(int featno)
      {
        ODHOGTrainer::nofeatures_neg = featno;
      }

      cv::Point const &getStartHogPos() const
      {
        return start_hog_pos;
      }

      void setStartHogPos(cv::Point const &start_hog_pos)
      {
        ODHOGTrainer::start_hog_pos = start_hog_pos;
      }

      cv::Size const &getWinSize() const
      {
        return winSize;
      }

      void setWinSize(cv::Size const &winSize)
      {
        ODHOGTrainer::winSize = winSize;
      }

      cv::Size const &getBlockSize() const
      {
        return blockSize;
      }

      void setBlockSize(cv::Size const &blockSize)
      {
        ODHOGTrainer::blockSize = blockSize;
      }

      cv::Size const &getBlockStride() const
      {
        return blockStride;
      }

      void setBlockStride(cv::Size const &blockStride)
      {
        ODHOGTrainer::blockStride = blockStride;
      }

      cv::Size const &getCellSize() const
      {
        return cellSize;
      }

      void setCellSize(cv::Size const &cellSize)
      {
        ODHOGTrainer::cellSize = cellSize;
      }

      cv::Size const &getTrainingPadding() const
      {
        return trainingPadding;
      }

      void setTrainingPadding(cv::Size const &trainingPadding)
      {
        ODHOGTrainer::trainingPadding = trainingPadding;
      }

      bool isTrainHardNegetive() const
      {
        return train_hard_negetive_;
      }

      void setTrainHardNegetive(bool train_hard_negetive)
      {
        ODHOGTrainer::train_hard_negetive_ = train_hard_negetive;
      }

      double getHitThreshold() const
      {
        return hitThreshold;
      }

    protected:
      //hog specific
      cv::Size winSize;
      cv::Size blockSize;
      cv::Size blockStride;
      cv::Size cellSize;

      cv::HOGDescriptor hog_;

      //algo specific
      cv::Size trainingPadding;
      cv::Point start_hog_pos;
      int nofeatures_neg;
      cv::Size winStride;
      bool train_hard_negetive_;

      //directories
      std::string posSamplesDir;
      std::string negSamplesDir;

      //properties retained
      double hitThreshold;

    private:
      //internal training data
      std::string featuresFile;
      std::string svmModelFile;
      std::string svmModelHard;
      std::string descriptorVectorFile;
      std::string descriptorVectorHard;


      void readDescriptorsFromFile(std::string fileName, std::vector<float> &descriptor_vector);

      void save(std::string filename);

      void createHardTrainingData(cv::HOGDescriptor const &hog, double const hitThreshold,
                              std::vector<std::string> const &negFileNames);

      void calculateFeaturesFromImageLoc(cv::Mat const &imageData, std::vector<float> &featureVector,
                                     cv::HOGDescriptor const &hog, cv::Point startpos);



      void detectTrainingSetTest(cv::HOGDescriptor const &hog, double const hitThreshold,
                             std::vector<std::string> const &posFileNames, std::vector<std::string> const &negFileNames);


      void calculateFeaturesFromInput(std::string const &imageFilename, std::vector<float> &featureVector,
                                  cv::HOGDescriptor &hog);



      void saveDescriptorVectorToFile(std::vector<float> &descriptorVector, std::vector<unsigned int> &_vectorIndices,
                                  std::string fileName);

      void handleNegetivefile(std::string const imageFilename, cv::HOGDescriptor &hog, std::fstream &file);

      double trainWithSVMLight(std::string svmModelFile, std::string svmDescriptorFile, std::vector<float> &descriptorVector);
    };
    /** \example objectdetector/od_hog_train.cpp
     */

  }
}



#endif //OPENDETECTION_ODHOGTRAINER_H

//
// Created by sarkar on 13.08.15.
//

#ifndef OPENDETECTION_ODHOGTRAINER_H
#define OPENDETECTION_ODHOGTRAINER_H

#include <opencv2/objdetect.hpp>
#include "common/pipeline/ODTrainer.h"
#include "common/utils/utils.h"




namespace od
{
  namespace l2d
  {
    /** \brief ODCADRecogTrainerSnapshotBased; One of the new algorithm; details will be explained later
   *
   * \author Kripasindhu Sarkar
   *
   */

    class ODHOGTrainer : public ODTrainer
    {

    public:
      ODHOGTrainer(std::string const &training_input_location_ = "", std::string const &training_data_location_ = "", cv::Size winsize = cv::Size(64,128), cv::Size blocksize = cv::Size(16,16), cv::Size blockstride = cv::Size(8,8), cv::Size cellsize = cv::Size(8,8), float hitshreshold = 0.0):
          ODTrainer(training_input_location_, training_data_location_),  winSize(winsize), blockSize(blocksize), blockStride(blockstride),
          cellSize(cellsize), hog_(winSize, blockSize, blockStride, cellSize, 9)
      {

        cv::Size trainingPadding = cv::Size(0, 0);
        cv::Point start_hog_pos = cv::Point(15, 15);
        cv::Size hogWinSize = cv::Size(64, 128);
        int FEATNUM_NEGFILE = 10;

      }

      int train() {}

      void init() {}


    protected:
      cv::Size winSize;
      cv::Size blockSize;
      cv::Size blockStride;
      cv::Size cellSize;

      cv::HOGDescriptor hog_;

      cv::Size trainingPadding;
      cv::Point start_hog_pos;



      int FEATNUM_NEGFILE = 10;

    };


  }
}



#endif //OPENDETECTION_ODHOGTRAINER_H

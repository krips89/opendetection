/*
 * RobustMatcher.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: eriba
 */

#include "RobustMatcher.h"
#include "Utils.h"
#include "Model.h"
#include <time.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/ml.hpp>
#include <common/utils/utils.h>


void convertToUnsignedSiftGPU(cv::Mat const &cv_des, vector<unsigned char> &siftgpu_des)
{
  int totdes = cv_des.rows * 128;
  siftgpu_des.resize(cv_des.rows * 128);

  int desi = 0;

  for (int i = 0; i < cv_des.rows; i++){
    for (int j = 0; j < 128; j++, desi++)
      siftgpu_des[desi] = (unsigned  char)cv_des.at<float>(i,j);
  }
}

void convertToDmatch(int siftgpu_matches[][2], int num_match, vector<cv::DMatch> &cv_matches)
{
  cv_matches.resize(num_match);
  for(int i  = 0; i < num_match; ++i)
  {
    //How to get the feature matches:
    //SiftGPU::SiftKeypoint & key1 = keys1[match_buf[i][0]];
    //SiftGPU::SiftKeypoint & key2 = keys2[match_buf[i][1]];
    //key1 in the first image matches with key2 in the second image
    cv_matches[i].trainIdx = siftgpu_matches[i][0]; //Assigned first at init
    cv_matches[i].queryIdx = siftgpu_matches[i][1];
  }
}


void RobustMatcher::instantiateMatcher(Model const &model, bool use_gpu)
{

  if (use_gpu_ == true) {

    //####GPU

    //1. for SIFT
    matcher_gpu_ = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2);
    cv::cuda::GpuMat train_descriptors(model.get_descriptors());
    matcher_gpu_->add(std::vector<cv::cuda::GpuMat>(1, train_descriptors));
    cout << "GPU matcher instantiated ..." << endl;

  }
  else
  {
    matcher_ =  cv::makePtr<cv::FlannBasedMatcher>();
  }
}

RobustMatcher::RobustMatcher(Model const &model, bool use_gpu, bool use_gpu_match)
{

  use_gpu_ = use_gpu_match;



  // ORB is the default feature
  ratio_ =0.8f;
  detector_ = cv::ORB::create();
  extractor_ = cv::ORB::create();



  instantiateMatcher(model, use_gpu_match);

//  // BruteFroce matcher with Norm Hamming is the default matcher
//  //matcher_ = cv::makePtr<cv::BFMatcher>((int)cv::NORM_HAMMING, false);
//  //matcher_ = cv::makePtr<cv::BFMatcher>((int)cv::NORM_L2, false);
//
//
//  //####################TEMPORARY CODE BELOW!!!!!!!!!!!!!!!!!!!!!!!!
//
//  cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1); // instantiate LSH index parameters
//  cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);       // instantiate flann search parameters
//  // instantiate FlannBased matcher
//  //Ptr<DescriptorMatcher> matcher = makePtr<FlannBasedMatcher>(indexParams, searchParams);
//  //matcher_ =  cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
//  matcher_ =  cv::makePtr<cv::FlannBasedMatcher>();

}

RobustMatcher::~RobustMatcher()
{
  // TODO Auto-generated destructor stub
}

void RobustMatcher::computeKeyPoints( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints)
{
  detector_->detect(image, keypoints);
}

void RobustMatcher::computeDescriptors( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
  extractor_->compute(image, keypoints, descriptors);
}

int RobustMatcher::ratioTest(std::vector<std::vector<cv::DMatch> > &matches)
{
  int removed = 0;
  // for all matches
  for ( std::vector<std::vector<cv::DMatch> >::iterator
        matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
  {
    // if 2 NN has been identified
    if (matchIterator->size() > 1)
    {
      // check distance ratio
      if ((*matchIterator)[0].distance / (*matchIterator)[1].distance > ratio_)
      {
        matchIterator->clear(); // remove match
        removed++;
      }
    }
    else
    { // does not have 2 neighbours
      matchIterator->clear(); // remove match
      removed++;
    }
  }
  return removed;
}

void get_good_matches(const std::vector<std::vector<cv::DMatch> >& matches, std::vector<cv::DMatch>& goodMatches)
{
  for (std::vector<std::vector<cv::DMatch> >::const_iterator
           matchIterator1 = matches.begin(); matchIterator1 != matches.end(); ++matchIterator1)
  {
    // ignore deleted matches
    if (matchIterator1->empty() || matchIterator1->size() < 2)
      continue;
    if ((*matchIterator1)[0].distance < (*matchIterator1)[1].distance)
      goodMatches.push_back((*matchIterator1)[0]);
  }

}

void RobustMatcher::symmetryTest( const std::vector<std::vector<cv::DMatch> >& matches1,
                     const std::vector<std::vector<cv::DMatch> >& matches2,
                     std::vector<cv::DMatch>& symMatches )
{

  // for all matches image 1 -> image 2
   for (std::vector<std::vector<cv::DMatch> >::const_iterator
       matchIterator1 = matches1.begin(); matchIterator1 != matches1.end(); ++matchIterator1)
   {

      // ignore deleted matches
      if (matchIterator1->empty() || matchIterator1->size() < 2)
         continue;

      // for all matches image 2 -> image 1
      for (std::vector<std::vector<cv::DMatch> >::const_iterator
          matchIterator2 = matches2.begin(); matchIterator2 != matches2.end(); ++matchIterator2)
      {
        // ignore deleted matches
        if (matchIterator2->empty() || matchIterator2->size() < 2)
           continue;

        // Match symmetry test
        if ((*matchIterator1)[0].queryIdx ==
            (*matchIterator2)[0].trainIdx &&
            (*matchIterator2)[0].queryIdx ==
            (*matchIterator1)[0].trainIdx)
        {
            // add symmetrical match
            symMatches.push_back(
              cv::DMatch((*matchIterator1)[0].queryIdx,
                         (*matchIterator1)[0].trainIdx,
                         (*matchIterator1)[0].distance));
            break; // next match in image 1 -> image 2
        }
      }
   }

}

void RobustMatcher::match(const cv::Mat & descriptors_frame, const cv::Mat &descriptors_model, std::vector<cv::DMatch>& good_matches)
{
  std::vector<std::vector<cv::DMatch> > matches;

  if (use_gpu_)
  {
    matcher_gpu_->knnMatch(cv::cuda::GpuMat(descriptors_frame), matches, 2); // return 2 nearest neighbours
    ratioTest(matches);
    for ( std::vector<std::vector<cv::DMatch> >::iterator
              matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
    {
      if (!matchIterator->empty()) good_matches.push_back((*matchIterator)[0]);
    }
  }
  else
  {
    matcher_->knnMatch(descriptors_frame, descriptors_model, matches, 2); // return 2 nearest neighbours
    ratioTest(matches);
    // 4. Fill good matches container
    for ( std::vector<std::vector<cv::DMatch> >::iterator
              matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
    {
      if (!matchIterator->empty()) good_matches.push_back((*matchIterator)[0]);
    }

  }
}

void RobustMatcher::matchNormalized(cv::Mat & descriptors_frame, cv::Mat &descriptors_model, std::vector<cv::DMatch>& good_matches)
{
  od::normL2(descriptors_frame); od::normL2(descriptors_model);
  match(descriptors_frame, descriptors_model, good_matches);
}

void RobustMatcher::robustMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
              std::vector<cv::KeyPoint>& keypoints_frame, const cv::Mat& descriptors_model )
{
  // 1a. Detection of the ORB features
  //this->computeKeyPoints(frame, keypoints_frame);

  // 1b. Extraction of the ORB descriptors
  cv::Mat descriptors_frame;
  //this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

  featureDetector_->computeKeypointsAndDescriptors(frame, descriptors_frame, keypoints_frame);
  //cout << "After Restacking: \n" << descriptors_frame << endl;
  // 2. Match the two image descriptors
  std::vector<std::vector<cv::DMatch> > matches12, matches21;

  // 2a. From image 1 to image 2
  matcher_->knnMatch(descriptors_frame, descriptors_model, matches12, 2); // return 2 nearest neighbours

  // 2b. From image 2 to image 1
  matcher_->knnMatch(descriptors_model, descriptors_frame, matches21, 2); // return 2 nearest neighbours


  // 3. Remove matches for which NN ratio is > than threshold
  // clean image 1 -> image 2 matches
  ratioTest(matches12);
  // clean image 2 -> image 1 matches
  ratioTest(matches21);

  //get_good_matches(matches12, good_matches);

  // 4. Remove non-symmetrical matches
  symmetryTest(matches12, matches21, good_matches);


}

void RobustMatcher::findFeatureAndMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                                     std::vector<cv::KeyPoint>& keypoints_frame,
                                     const cv::Mat& descriptors_model )
{
  good_matches.clear();

  // 1b. Extraction of the ORB descriptors
  cv::Mat descriptors_frame;
  featureDetector_->computeKeypointsAndDescriptors(frame, descriptors_frame, keypoints_frame);

  cout << "df " << descriptors_model.rows << endl;
  cout << "kf" << descriptors_frame.rows << endl;

  match(descriptors_frame, descriptors_model, good_matches);

}


void RobustMatcher::fastRobustMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                                 std::vector<cv::KeyPoint>& keypoints_frame,
                                 const cv::Mat& descriptors_model )
{
  good_matches.clear();

  cv::Mat d_f, d_m;
  // 1a. Detection of the ORB features
  //this->computeKeyPoints(frame, keypoints_frame);

  // 1b. Extraction of the ORB descriptors
  cv::Mat descriptors_frame;
  //this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

  featureDetector_->computeKeypointsAndDescriptors(frame, descriptors_frame, keypoints_frame);

  if (descriptors_frame.type() != CV_32F) {
    descriptors_frame.convertTo(d_f, CV_32F);
  }
  else {
    d_f = descriptors_frame;
  }
  if (descriptors_model.type() != CV_32F) {
    descriptors_model.convertTo(d_f, CV_32F);
  }
  else
  {
    d_m = descriptors_model;
  }

  // 2. Match the two image descriptors
  std::vector<std::vector<cv::DMatch> > matches;
  matcher_->knnMatch(d_f, d_m, matches, 2);

  // 3. Remove matches for which NN ratio is > than threshold
  ratioTest(matches);

  // 4. Fill good matches container
  for ( std::vector<std::vector<cv::DMatch> >::iterator
         matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
  {
    if (!matchIterator->empty()) good_matches.push_back((*matchIterator)[0]);
  }

}



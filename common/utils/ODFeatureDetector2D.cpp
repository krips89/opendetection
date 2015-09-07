//
// Created by sarkar on 20.04.15.
//

#include <GL/gl.h>
#include "ODFeatureDetector2D.h"

using namespace std;
namespace od
{

  ODFeatureDetector2D::ODFeatureDetector2D(string type, bool use_gpu)
  {
    mode_ = SIFT; //by default it is SIFT

    if(use_gpu) {


      //##########GPU VERSION


      if(type == "ORB") {
        mode_ = ORB_GPU;
        feature_detector_ = cv::cuda::ORB::create();
      } else if(type == "SIFT") {
        mode_ = SIFT_GPU;
        sift_gpu_ = new SiftGPU;
        //char * argv[] = {(char *)"-fo", (char *)"-1",  (char *)"-v", (char *)"1", (char *)"-cuda", (char *)"0"};
        char *argv[] = {(char *) "-fo", (char *) "-1", (char *) "-v", (char *) "1"};
        int argc = sizeof(argv) / sizeof(char *);
        sift_gpu_->ParseParam(argc, argv);
        if(sift_gpu_->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
          cout << "FATAL ERROR cannot create SIFTGPU context";
      }
    } else {

      //########CPU VERSIONS


      if(type == "SIFT") {
        mode_ = SIFT;
        feature_detector_ = cv::xfeatures2d::SIFT::create();
      } else if(type == "ORB") {
        mode_ = ORB;
        feature_detector_ = cv::ORB::create();
      } else if(type == "SURF") {
        mode_ = SURF;
        feature_detector_ = cv::xfeatures2d::SURF::create();
      }
    }
  }

  void ODFeatureDetector2D::computeKeypointsAndDescriptors(cv::Mat const &image, cv::Mat &descriptors, vector<cv::KeyPoint> &keypoints)
  {
    if(mode_ == SIFT_GPU) {
      findSiftGPUDescriptors1(image, descriptors, keypoints);
    } else {
      feature_detector_->detect(image, keypoints);
      feature_detector_->compute(image, keypoints, descriptors);
    }
    //viewImage(image, keypoints);
  }

  void CVMatToSiftGPU(const cv::Mat &image, unsigned char *siftImage, cv::Mat &grey)
  {
    siftImage = (unsigned char *) malloc(image.rows * image.cols);
    cv::Mat tmp;
    cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);

    memcpy(siftImage, grey.data, image.rows * image.cols);


//  tmp2.convertTo(tmp, CV_8U);
//  //viewImage(tmp);
//
//  for (int y = 0; y < tmp.rows; ++y) {
//    for (int x = 0; x < tmp.cols; ++x) {
//      siftImage[y * tmp.cols + x] = tmp.at<unsigned char> (y, x);
//    }
//  }
    return;
  }

  void ODFeatureDetector2D::findSiftGPUDescriptors1(cv::Mat const &image, cv::Mat &descriptors, vector<cv::KeyPoint> &keypoints)
  {
    unsigned char *data = image.data;
    cv::Mat greyimage;
    if(image.type() != CV_8U) {
      cv::cvtColor(image, greyimage, cv::COLOR_BGR2GRAY);
      data = greyimage.data;
    }
    sift_gpu_->RunSIFT(image.cols, image.rows, data, GL_LUMINANCE, GL_UNSIGNED_BYTE);

    int nFeat = sift_gpu_->GetFeatureNum();//get feature count
    //allocate memory for readback
    vector<SiftGPU::SiftKeypoint> keys(nFeat);
    //read back keypoints and normalized descritpros
    //specify NULL if you don’t need keypoints or descriptors
    vector<float> imageDescriptors(128 * nFeat);
    sift_gpu_->GetFeatureVector(&keys[0], &imageDescriptors[0]);

    sift_gpu_->SaveSIFT("2.sift");

    //to opencv format
    keypoints.clear();
    descriptors.create(0, 128, CV_32FC1);
    for(int i = 0; i < nFeat; ++i) {
      cv::KeyPoint key(keys[i].x, keys[i].y, keys[i].s, keys[i].o);
      keypoints.push_back(key);
      cv::Mat descriptor(1, 128, CV_32FC1);

      for(int x = 0; x < 128; x++)
        descriptor.at<float>(x) = floor(0.5 + (512.0f * imageDescriptors[(i * 128) + x]));

      descriptors.push_back(descriptor);
    }


    //viewImage(image, keypoints);

  }

  void ODFeatureDetector2D::findSiftGPUDescriptors(cv::Mat const &image, cv::Mat &descriptors, vector<cv::KeyPoint> &keypoints)
  {
    unsigned char *data = image.data;
    cv::Mat greyimage;
    if(image.type() != CV_8U) {
      cv::Mat tmp;
      cv::cvtColor(image, tmp, cv::COLOR_BGR2GRAY);
      data = tmp.data;
    }
    sift_gpu_->RunSIFT(image.cols, image.rows, data, GL_LUMINANCE, GL_UNSIGNED_BYTE);

    int nFeat = sift_gpu_->GetFeatureNum();//get feature count
    //allocate memory for readback
    vector<SiftGPU::SiftKeypoint> keys(nFeat);
    //read back keypoints and normalized descritpros
    //specify NULL if you don’t need keypoints or descriptors
    vector<float> imageDescriptors(128 * nFeat);
    sift_gpu_->GetFeatureVector(&keys[0], &imageDescriptors[0]);

    sift_gpu_->SaveSIFT("2.sift");

    //to opencv format
    keypoints.clear();
    cv::Mat descriptormat = cv::Mat(1, nFeat, CV_32F, &imageDescriptors[0]);
    descriptormat.copyTo(descriptors);

    for(int i = 0; i < nFeat; ++i) {
      cv::KeyPoint key(keys[i].x, keys[i].y, keys[i].s, keys[i].o);
      keypoints.push_back(key);
    }


    //viewImage(image, keypoints);

  }

  void ODFeatureDetector2D::findSiftGPUDescriptors(char const *image_name, cv::Mat &descriptors, vector<cv::KeyPoint> &keypoints)
  {
    sift_gpu_->RunSIFT(image_name);

    int nFeat = sift_gpu_->GetFeatureNum();//get feature count
    //allocate memory for readback
    vector<SiftGPU::SiftKeypoint> keys(nFeat);
    //read back keypoints and normalized descritpros
    //specify NULL if you don’t need keypoints or descriptors
    vector<float> imageDescriptors(128 * nFeat);
    sift_gpu_->GetFeatureVector(&keys[0], &imageDescriptors[0]);

    sift_gpu_->SaveSIFT("1.sift");

    //to opencv format
    keypoints.clear();
    descriptors.create(0, 128, CV_32FC1);
    for(int i = 0; i < nFeat; ++i) {
      cv::KeyPoint key(keys[i].x, keys[i].y, keys[i].s, keys[i].o);
      keypoints.push_back(key);
      cv::Mat descriptor(1, 128, CV_32FC1);
      for(int x = 0; x < 128; x++) descriptor.at<float>(x) = floor(0.5 + 512.0f * imageDescriptors[(i << 7) + x]);
      descriptors.push_back(descriptor);
    }
    cv::Mat image = cv::imread(image_name);
  }

  void ODFeatureDetector2D::computeAndSave(cv::Mat const &image, std::string const path)
  {
    cv::Mat descriptors;
    vector<cv::KeyPoint> keypoints;
    if(mode_ == SIFT_GPU) {
      findSiftGPUDescriptors1(image, descriptors, keypoints);
      sift_gpu_->SaveSIFT(path.c_str());
    } else {
      //DO NOTHING! IMPLEMENT LATER

    }
  }
}

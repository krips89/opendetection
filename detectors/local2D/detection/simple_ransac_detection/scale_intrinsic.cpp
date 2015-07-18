//
// Created by sarkar on 04.05.15.
//


// C++
#include <iostream>
#include <time.h>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>


using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
  if( argc != 4 )
  { cout << "Not sufficient arguments!!"; return -1; }

  string matname = argv[1];
  string matnamescaled = argv[2];
  float scale = atof(argv[3]);

  FileStorage fr(matname, FileStorage::READ);
  FileStorage fw(matnamescaled, FileStorage::WRITE);

  Mat K, d;
  fr["Camera_Matrix"] >> K;
  fr["Distortion_Coefficients"] >> d;


  K = scale * K;
  K.at<double>(2, 2) = 1;

  fw << "Camera_Matrix" << K;
  fw << "Distortion_Coefficients" << d;

  return 1;
}
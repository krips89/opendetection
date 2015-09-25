//
// Created by sarkar on 19.06.15.
//
#include "utils.h"

namespace od
{


  std::vector<std::string> myglob(const std::string &pat)
  {
    using namespace std;
    glob_t glob_result;
    glob(pat.c_str(), GLOB_TILDE, NULL, &glob_result);
    vector<string> ret;
    for(unsigned int i = 0; i < glob_result.gl_pathc; ++i) {
      ret.push_back(string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return ret;
  }

  void normL2(cv::Mat &descriptors)
  {
    for (int r = 0; r < descriptors.rows; r++)
    {
      float norm=0;
      for (int c = 0; c < descriptors.cols; c++) norm+=(descriptors.at<float>(r, c)*descriptors.at<float>(r, c));
      norm = 1.0/sqrt(norm);
      for (int c = 0; c < descriptors.cols; c++) descriptors.at<float>(r, c)*=norm;
    }
  }
}


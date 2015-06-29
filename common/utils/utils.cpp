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

}
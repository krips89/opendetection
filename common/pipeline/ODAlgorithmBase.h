//
// Created by sarkar on 12.06.15.
//

#ifndef OPENDETECTION_ODALGORITHMBASE_H
#define OPENDETECTION_ODALGORITHMBASE_H
#include<iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>


namespace od
{

  /** \brief The base of all algorithm classes
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODAlgorithmBase
  {
  public:

    virtual void parseParameterString(std::string parameter_string)
    { }

    virtual void init()
    { }
  };
}

#endif //OPENDETECTION_ODALGORITHMBASE_H

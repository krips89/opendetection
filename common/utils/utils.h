//
// Created by sarkar on 09.06.15.
//

#ifndef OPENDETECTION_UTILS_H
#define OPENDETECTION_UTILS_H

#include <sys/time.h>
#include <boost/preprocessor.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <fstream>
#include <sstream>

namespace bf = boost::filesystem;

/** \brief misclenious helping functions; will be refactored later
   *
   * \author Kripasindhu Sarkar
   *
   */

#define X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE(r, data, elem)    \
    case elem : return BOOST_PP_STRINGIZE(elem);

#define OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(name, enumerators)                \
    enum name {                                                               \
        BOOST_PP_SEQ_ENUM(enumerators)                                        \
    };                                                                        \
                                                                              \
    inline const char* enumToString(name v)                                       \
    {                                                                         \
        switch (v)                                                            \
        {                                                                     \
            BOOST_PP_SEQ_FOR_EACH(                                            \
                X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE,          \
                name,                                                         \
                enumerators                                                   \
            )                                                                 \
            default: return "[Unknown " BOOST_PP_STRINGIZE(name) "]";         \
        }                                                                     \
    }


template<typename T>
std::string toString(T Number)
{
  std::ostringstream ss;
  ss << Number;
  return ss.str();
}

static std::string getTexfileinObj(std::string objfilename)
{

  boost::filesystem::path p(objfilename);
  std::string input_dir = boost::filesystem::path(objfilename).parent_path().c_str();

  std::ifstream input(objfilename.c_str());
  std::string line;
  while (getline(input, line)) {
    std::istringstream iss(line);
    std::string tok1;
    iss >> tok1;
    if(tok1 == "mtllib")
    {
      std::string tok2;

      iss >> tok2;
      std::string linemtl;

      std::ifstream inputmtl((input_dir + "/" + tok2).c_str());
      while (getline(inputmtl, linemtl))
      {
        std::istringstream issmtl(linemtl);
        issmtl >> tok1;
        if (tok1 == "map_Kd")
        {
          issmtl >> tok2;
          return input_dir  + "/" + tok2;
        }
      }
    }
  }
  return "";
}


class Timer {
private:

  timeval startTime;

public:

  double duration_;

  void start(){
    gettimeofday(&startTime, NULL);
  }

  double stop(){
    timeval endTime;
    long seconds, useconds;
    double duration;

    gettimeofday(&endTime, NULL);

    seconds  = endTime.tv_sec  - startTime.tv_sec;
    useconds = endTime.tv_usec - startTime.tv_usec;

    duration = seconds + useconds/1000000.0;
    duration_ = duration;
    return duration;
  }


  double getDuration(){return duration_;}

  static void printTime(double duration){
    printf("%5.6f seconds\n", duration);
  }
};



template <typename T, typename Ptype>
void printListIn (std::vector<T> list, int n = 0)
{
  int num;
  if (n == 0) num = list.size();
  else num = n < list.size()? n: list.size();

  for (int i = 0; i < num; i++)
    std::cout << (Ptype)list[i] << " ";
  std::cout << std::endl;
}

template <typename T>
void printList (std::vector<T> list )
{
  for (int i = 0; i < list.size(); i++)
    std::cout << list[i] << " ";
  std::cout << std::endl;
}

#endif //OPENDETECTION_UTILS_H

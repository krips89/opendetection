//
// Created by sarkar on 09.06.15.
//

#ifndef OPENDETECTION_UTILS_H
#define OPENDETECTION_UTILS_H

#include <sys/time.h>
#include <boost/preprocessor.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <opencv2/core/core.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <glob.h>



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


namespace od
{

  template<typename T>
  std::string toString(T Number)
  {
    std::ostringstream ss;
    ss << Number;
    return ss.str();
  }

  std::vector<std::string> myglob(const std::string &pat);

  void normL2(cv::Mat &descriptors);


  /**
    * @brief Makes composite image from the given images. Equal number of rows and columns are preferred. Therefore, number of rows = sqrt(number of input images)
    *
    * @param imgs Vector of Images.
    * @param cellSize Size of individual images to be placed inside the composite images. images from `imgs` will be resized to this size before appending.
    * @param messages Messages to be put on the top left of each image in `imgs`. Note `message.size()` should be equal to `imgs.size()`.
    * @return new composite image.
    */
  cv::Mat makeCanvasMultiImages(std::vector<cv::Mat>& imgs, cv::Size cellSize, std::vector<std::string> messages);

  cv::Scalar getHashedColor(std::string name, int constadd);

  static std::string getTexfileinObj(std::string objfilename)
  {

    boost::filesystem::path p(objfilename);
    std::string input_dir = boost::filesystem::path(objfilename).parent_path().c_str();

    std::ifstream input(objfilename.c_str());
    std::string line;
    while(getline(input, line))
    {
      std::istringstream iss(line);
      std::string tok1;
      iss >> tok1;
      if(tok1 == "mtllib")
      {
        std::string tok2;

        iss >> tok2;
        std::string linemtl;

        std::ifstream inputmtl((input_dir + "/" + tok2).c_str());
        while(getline(inputmtl, linemtl))
        {
          std::istringstream issmtl(linemtl);
          issmtl >> tok1;
          if(tok1 == "map_Kd")
          {
            issmtl >> tok2;
            return input_dir + "/" + tok2;
          }
        }
      }
    }
    return "";
  }

/** \brief An utility class for Timer related operations.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class Timer
  {
  private:

    timeval startTime;

  public:

    double duration_;

    void start()
    {
      gettimeofday(&startTime, NULL);
    }

    double stop()
    {
      timeval endTime;
      long seconds, useconds;
      double duration;

      gettimeofday(&endTime, NULL);

      seconds = endTime.tv_sec - startTime.tv_sec;
      useconds = endTime.tv_usec - startTime.tv_usec;

      duration = seconds + useconds / 1000000.0;
      duration_ = duration;
      return duration;
    }


    double getDuration()
    { return duration_; }

    static void printTime(double duration)
    {
      printf("%5.6f seconds\n", duration);
    }
  };


  template<typename T, typename Ptype>
  void printListIn(std::vector<T> list, int n = 0)
  {
    int num;
    if(n == 0) num = list.size(); else num = n < list.size() ? n : list.size();

    for(int i = 0; i < num; i++)
      std::cout << (Ptype) list[i] << " ";
    std::cout << std::endl;
  }

  template<typename T>
  void printList(std::vector<T> list)
  {
    for(int i = 0; i < list.size(); i++)
      std::cout << list[i] << " ";
    std::cout << std::endl;
  }


  /** \brief Utility class for File and directory handling.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class FileUtils
  {
  public:

    static std::string getFirstFile(std::string base_path, std::string extension)
    {
      std::vector<std::string> files;
      std::string start = "";
      std::string ext = extension;
      bf::path dir = base_path;
      FileUtils::getFilesInDirectoryInternal(dir, start, files, ext);
      if (files.size() == 0)
      {
        std::cout << "No file with extension " << extension << " present!\nReturning NULL";
        return "";
      }
      else return files[0];
    }

    static void getFilesInDirectoryRec(std::string base_path, std::string extension, std::vector<std::string> &files)
    {
      std::string start = "";
      std::string ext = extension;
      bf::path dir = base_path;
      FileUtils::getFilesInDirectoryInternal(dir, start, files, ext);
    }

    static void getFilesInDirectoryRec(std::string base_path, std::vector<std::string> extensions, std::vector<std::string> &files)
    {
      std::string start = "";
      std::vector<std::string> exts = extensions;
      bf::path dir = base_path;
      FileUtils::getFilesInDirectoryInternal(dir, start, files, exts);
    }

    static void getFilesInDirectoryInternal(bf::path &dir, std::string &rel_path_so_far, std::vector<std::string> &relative_paths, std::vector<std::string> exts)
    {
      bf::directory_iterator end_itr;
      for(bf::directory_iterator itr(dir); itr != end_itr; ++itr) {
        //check if its a directory, then get models in it
        if(bf::is_directory(*itr)) {
#if BOOST_FILESYSTEM_VERSION == 3
          std::string so_far = rel_path_so_far + (itr->path().filename()).string() + "/";
#else
            std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

          bf::path curr_path = itr->path();
          getFilesInDirectoryInternal(curr_path, so_far, relative_paths, exts);
        } else {
          //check that it is a ply file and then add, otherwise ignore..
          std::vector<std::string> strs;
#if BOOST_FILESYSTEM_VERSION == 3
          std::string file = (itr->path().filename()).string();
#else
            std::string file = (itr->path ()).filename ();
#endif

          boost::split(strs, file, boost::is_any_of("."));
          std::string extension = strs[strs.size() - 1];

          bool flagfound = false;
          for (int exti = 0; exti < exts.size(); exti ++)
            if(file.rfind(exts[exti]) != std::string::npos)
            { flagfound = true; break; }

          if( flagfound == true )
          {
#if BOOST_FILESYSTEM_VERSION == 3
            std::string path = rel_path_so_far + (itr->path().filename()).string();
#else
              std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif
            std::string fullpath = rel_path_so_far + itr->path().string();
            relative_paths.push_back(fullpath);
          }
        }
      }
    }

    static void getFilesInDirectoryInternal(bf::path &dir, std::string &rel_path_so_far, std::vector<std::string> &relative_paths, std::string const &ext)
    {
      bf::directory_iterator end_itr;
      for(bf::directory_iterator itr(dir); itr != end_itr; ++itr) {
        //check if its a directory, then get models in it
        if(bf::is_directory(*itr)) {
#if BOOST_FILESYSTEM_VERSION == 3
          std::string so_far = rel_path_so_far + (itr->path().filename()).string() + "/";
#else
            std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

          bf::path curr_path = itr->path();
          getFilesInDirectoryInternal(curr_path, so_far, relative_paths, ext);
        } else {
          //check that it is a ply file and then add, otherwise ignore..
          std::vector<std::string> strs;
#if BOOST_FILESYSTEM_VERSION == 3
          std::string file = (itr->path().filename()).string();
#else
            std::string file = (itr->path ()).filename ();
#endif

          boost::split(strs, file, boost::is_any_of("."));
          std::string extension = strs[strs.size() - 1];

          if( file.rfind(ext) != std::string::npos )
          {
#if BOOST_FILESYSTEM_VERSION == 3
            std::string path = rel_path_so_far + (itr->path().filename()).string();
#else
              std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif
            std::string fullpath = rel_path_so_far + itr->path().string();
            relative_paths.push_back(fullpath);
          }
        }
      }
    }

    static void getFilesInDirectory(bf::path &dir, std::string &rel_path_so_far, std::vector<std::string> &relative_paths, std::string const &ext)
    {
      bf::directory_iterator end_itr;
      for(bf::directory_iterator itr(dir); itr != end_itr; ++itr) {
        //check if its a directory, then get models in it
        if(bf::is_directory(*itr)) {
#if BOOST_FILESYSTEM_VERSION == 3
          std::string so_far = rel_path_so_far + (itr->path().filename()).string() + "/";
#else
            std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

          bf::path curr_path = itr->path();
          getFilesInDirectoryInternal(curr_path, so_far, relative_paths, ext);
        } else {
          //check that it is a ply file and then add, otherwise ignore..
          std::vector<std::string> strs;
#if BOOST_FILESYSTEM_VERSION == 3
          std::string file = (itr->path().filename()).string();
#else
            std::string file = (itr->path ()).filename ();
#endif

          boost::split(strs, file, boost::is_any_of("."));
          std::string extension = strs[strs.size() - 1];

          if( file.rfind(ext) != std::string::npos )
          {
#if BOOST_FILESYSTEM_VERSION == 3
            std::string path = rel_path_so_far + (itr->path().filename()).string();
#else
              std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif
            std::string fullpath = rel_path_so_far + itr->path().string();
            relative_paths.push_back(path);
          }
        }
      }
    }

    static void createTrainingDir(std::string training_dir)
    {
      bf::path trained_dir = training_dir;
      if(!bf::exists(trained_dir))
        bf::create_directory(trained_dir);
    }

    static void getArgvArgc(std::string const &commandline, char ***argv, int &argc)
    {
      enum
      {
        kMaxArgs = 64
      };

      argc = 0;
      *argv = new char *[kMaxArgs];
      (*argv)[argc++] = (char *) "program";

      char *p;
      p = strtok((char *) commandline.c_str(), " ");
      while(p && argc < kMaxArgs) {
        (*argv)[argc++] = p;
        p = strtok(0, " ");
      }
    }
  };
}
#endif //OPENDETECTION_UTILS_H

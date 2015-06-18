//
// Created by sarkar on 12.06.15.
//

#ifndef OPENDETECTION_ODALGORITHMBASE_H
#define OPENDETECTION_ODALGORITHMBASE_H
#include<iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>


namespace bf = boost::filesystem;

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
    void getFilesInDirectory(bf::path &dir, std::string &rel_path_so_far, std::vector<std::string> &relative_paths, std::string const &ext)
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
          getFilesInDirectory(curr_path, so_far, relative_paths, ext);
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

          if(extension.compare(ext) == 0) {
#if BOOST_FILESYSTEM_VERSION == 3
            std::string path = rel_path_so_far + (itr->path().filename()).string();
#else
              std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif

            relative_paths.push_back(path);
          }
        }
      }
    }

    void createTrainingDir(std::string &training_dir)
    {
      bf::path trained_dir = training_dir;
      if(!bf::exists(trained_dir))
        bf::create_directory(trained_dir);
    }

    void getArgvArgc(std::string const &commandline, char ***argv, int &argc)
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

    virtual void parseParameterString(std::string parameter_string)
    { }

    virtual void init()
    { }
  };
}

#endif //OPENDETECTION_ODALGORITHMBASE_H

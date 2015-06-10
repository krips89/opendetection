//
// Created by sarkar on 08.06.15.
//

#ifndef OPENDETECTION_TRAINER_H
#define OPENDETECTION_TRAINER_H

#include<iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;

namespace bf = boost::filesystem;

namespace od
{
  class Trainer
  {
  public:

    Trainer(string const &training_input_location_, string const &training_data_location_) : training_input_location_(
        training_input_location_), training_data_location_(training_data_location_)
    { }

    virtual int train() = 0;

    string getTrainingInputLocation() const
    {
      return training_input_location_;
    }

    void setTrainingInputLocation(string training_input_location_)
    {
      this->training_input_location_ = training_input_location_;
    }

    string getTrainingDataLocation() const
    {
      return training_data_location_;
    }

    void setTrainingDataLocation(string training_data_location_)
    {
      this->training_data_location_ = training_data_location_;
    }

    void createTrainingDir (std::string & training_dir)
    {
      bf::path trained_dir = training_dir;
      if(!bf::exists(trained_dir))
        bf::create_directory(trained_dir);
    }

    void
    getFilesInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext)
    {
      bf::directory_iterator end_itr;
      for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
      {
        //check if its a directory, then get models in it
        if (bf::is_directory (*itr))
        {
#if BOOST_FILESYSTEM_VERSION == 3
          std::string so_far = rel_path_so_far + (itr->path ().filename ()).string() + "/";
#else
            std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

          bf::path curr_path = itr->path ();
          getFilesInDirectory (curr_path, so_far, relative_paths, ext);
        }
        else
        {
          //check that it is a ply file and then add, otherwise ignore..
          std::vector < std::string > strs;
#if BOOST_FILESYSTEM_VERSION == 3
          std::string file = (itr->path ().filename ()).string();
#else
            std::string file = (itr->path ()).filename ();
#endif

          boost::split (strs, file, boost::is_any_of ("."));
          std::string extension = strs[strs.size () - 1];

          if (extension.compare (ext) == 0)
          {
#if BOOST_FILESYSTEM_VERSION == 3
            std::string path = rel_path_so_far + (itr->path ().filename ()).string();
#else
              std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif

            relative_paths.push_back (path);
          }
        }
      }
    }

  protected:
    string training_input_location_, training_data_location_;
  };

}
#endif //OPENDETECTION_TRAINER_H

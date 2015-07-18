/*
 * Model.cpp
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#include <fstream>
#include <iostream>
#include <pugixml.hpp>
#include "Model.h"
#include "CsvWriter.h"
#include <sstream>
#include <ostream>
#include <boost/algorithm/string.hpp>

Model::Model() : list_points2d_in_(0), list_points2d_out_(0), list_points3d_in_(0)
{
  n_correspondences_ = 0;
}

Model::~Model()
{
  // TODO Auto-generated destructor stub
}

void Model::add_correspondence(const cv::Point2f &point2d, const cv::Point3f &point3d)
{
  list_points2d_in_.push_back(point2d);
  list_points3d_in_.push_back(point3d);
  n_correspondences_++;
}

void Model::add_outlier(const cv::Point2f &point2d)
{
  list_points2d_out_.push_back(point2d);
}

void Model::add_descriptor(const cv::Mat &descriptor)
{
  descriptors_.push_back(descriptor);
}

void Model::add_keypoint(const cv::KeyPoint &kp)
{
  list_keypoints_.push_back(kp);
}


/** Save a CSV file and fill the object mesh */
void Model::save(const std::string path)
{
  cv::Mat points3dmatrix = cv::Mat(list_points3d_in_);
  cv::Mat points2dmatrix = cv::Mat(list_points2d_in_);
  //cv::Mat keyPointmatrix = cv::Mat(list_keypoints_);

  cv::FileStorage storage(path, cv::FileStorage::WRITE);
  storage << "points_3d" << points3dmatrix;
  storage << "points_2d" << points2dmatrix;
  storage << "keypoints" << list_keypoints_;
  storage << "descriptors" << descriptors_;

  storage.release();
}

/** Load a YAML file using OpenCv functions **/
void Model::load(const std::string path)
{
  cv::Mat points3d_mat;

  cv::FileStorage storage(path, cv::FileStorage::READ);
  storage["points_3d"] >> points3d_mat;
  storage["descriptors"] >> descriptors_;

  points3d_mat.copyTo(list_points3d_in_);

  std::cout << list_points3d_in_.size() << endl;
  std::cout << descriptors_;
  storage.release();
  id = path;
}

void Model::load_new_desc(const std::string path)
{
  f_type = "SIFT";

  std::fstream infile(path.c_str());
  if (!infile.is_open())
  {
    std::cout << "ERRROR opening model file!!";
    return ;

  }
  int nop, desc_size, point_size;
  infile >> nop;
  infile >> point_size >> desc_size;

  descriptors_ = Mat(nop, desc_size, CV_32FC1);


  for (int i = 0; i < nop; i++)
  {

  	float x, y, z;
  	infile >> x >> y >> z;
    cv::Point3f p3d(x, y, z);
    list_points3d_in_.push_back(p3d);

    cv::KeyPoint kp;
    infile >> kp.pt.x >> kp.pt.y >> kp.octave >> kp.angle >> kp.response >> kp.size;
    list_keypoints_.push_back(kp);

    for (int j = 0; j < desc_size; j++)
    {
      infile >> descriptors_.at<float>(i, j);
    }
  }
  id = path;
}

template <typename T>
string ps ( T thing )
{
  cout << thing;
  ostringstream ss;
  ss << thing;
  return ss.str();
}

void Model::load_new_xml(const std::string path)
{

  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(path.c_str());

  if (!result)
  {
    std::cout << "ERRROR opening model file!!";
    return ;

  }
  pugi::xml_node pts_node = doc.child("Model").child("Points");

  if (pts_node.first_child())
    f_type = pts_node.first_child().attribute("desc_type").as_string();
  else return;


  for (pugi::xml_node pt_node = pts_node.first_child(); pt_node; pt_node = pt_node.next_sibling())
  {
    Mat single_desc = Mat::zeros(1, get_descriptor_size(), CV_32FC1);
    stringstream ss;
    ss.str(pt_node.attribute("p3d").as_string());
    float x, y, z;
    ss >> x >> y >> z; ss.clear();
    cv::Point3f p3d(x, y, z);
    list_points3d_in_.push_back(p3d);

    ss.str(pt_node.attribute("p2d_prop").as_string());
    cv::KeyPoint kp;
    ss >> kp.pt.x >> kp.pt.y >> kp.octave >> kp.angle >> kp.response >> kp.size; ss.clear();
    list_keypoints_.push_back(kp);

    ss.str(pt_node.attribute("desc").as_string());
    float d;
    for(int it = 0; it < get_descriptor_size(); it ++)
    {
      ss >> d;
      //cout <<it << " " <<  ss.eof() << " " <<  d << endl;
      single_desc.at<float> (0, it) = d;
    }
    ss.clear();

    descriptors_.push_back(single_desc);
  }

  id = path;
  //std::cout << list_points3d_in_.size() << endl;
  //std::cout << descriptors_;
}


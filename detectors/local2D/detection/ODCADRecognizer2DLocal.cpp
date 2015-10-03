/*
Copyright (c) 2015, Kripasindhu Sarkar
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by sarkar on 08.06.15.
//

#include "ODCADRecognizer2DLocal.h"


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/viz.hpp>
#include <sys/time.h>
#include <common/utils/utils.h>

// PnP Tutorial
#include "simple_ransac_detection/Mesh.h"
#include "simple_ransac_detection/Model.h"
#include "simple_ransac_detection/PnPProblem.h"
#include "simple_ransac_detection/RobustMatcher.h"
#include "simple_ransac_detection/ModelRegistration.h"
#include "simple_ransac_detection/Utils.h"


using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

namespace od
{
  namespace l2d
  {

    void ODCADRecognizer2DLocal::parseParameterString(string parameter_string)
    {
      const String keys = "{help h        |      | print this message                   }"
          "{video v       |      | path to recorded video               }"
          "{test_images img       |      | image for detection               }"
          "{cam_id         |      | pass true if you want the input from the camera             }"
          "{camera_intrinsic_file      |      | yml file containing camera parameters             }"
          "{use_gpu        |      | use gpu or cpu                     }"
          "{use_gpu_match        |      | use gpu or cpu for matching                    }"
          "{keypoints k   |2000  | number of keypoints to detect        }"
          "{ratio r       |0.7   | threshold for ratio test             }"
          "{iterations it |500   | RANSAC maximum iterations count      }"
          "{error e       |2.0   | RANSAC reprojection errror           }"
          "{confidence c  |0.95  | RANSAC confidence                    }"
          "{inliers in    |30    | minimum inliers for Kalman update    }"
          "{method  pnp   |0     | PnP method: (0) ITERATIVE - (1) EPNP - (2) P3P - (3) DLS}"
          "{fast f        |false  | use of robust fast match             }"
          "{metainfo meta        |  | provide meta information in Detection             }";

      char **argv;
      int argc;
      FileUtils::getArgvArgc(parameter_string, &argv, argc);

      CommandLineParser parser(argc, argv, keys);
      if(parser.has("help"))
      {
        parser.printMessage();

      } else
      {
        camera_intrinsic_file =
            parser.get<string>("camera_intrinsic_file").size() > 0 ? parser.get<string>("camera_intrinsic_file") : camera_intrinsic_file;

        use_gpu = parser.has("use_gpu");
        use_gpu_match = parser.has("use_gpu_match");

        numKeyPoints = !parser.has("keypoints") ? parser.get<int>("keypoints") : numKeyPoints;
        ratioTest = !parser.has("ratio") ? parser.get<float>("ratio") : ratioTest;
        fast_match = !parser.has("fast") ? parser.get<bool>("fast") : fast_match;
        iterationsCount = !parser.has("iterations") ? parser.get<int>("iterations") : iterationsCount;
        reprojectionError = !parser.has("error") ? parser.get<float>("error") : reprojectionError;
        confidence = !parser.has("confidence") ? parser.get<float>("confidence") : confidence;
        minInliers = !parser.has("inliers") ? parser.get<int>("inliers") : minInliers;
        pnpMethod = !parser.has("method") ? parser.get<int>("method") : pnpMethod;
        metainfo_ = parser.has("metainfo");
      }

    }

    void ODCADRecognizer2DLocal::init()
    {
      FileStorage fs(camera_intrinsic_file, FileStorage::READ);
      Mat cam_man, dist_coeff;
      fs["Camera_Matrix"] >> cam_man;
      fs["Distortion_Coefficients"] >> dist_coeff;
      pnp_detection = PnPProblem(cam_man, dist_coeff);

      // get all trained models
      FileUtils::getFilesInDirectoryRec(getSpecificTrainingDataLocation(), TRAINED_DATA_ID_, model_names);

      for(int i = 0; i < model_names.size(); i++)
      {
        Model model;
        model.load_new_xml(model_names[i]);
        models.push_back(model);
      }
      if(models.size() > 0)
        f_type_default = models[0].f_type;

      featureDetector = boost::make_shared<ODFeatureDetector2D>(f_type_default, use_gpu);

    }



    ODDetections* ODCADRecognizer2DLocal::detect(ODSceneImage *scene)
    {
      ODDetections3D *detections = detectOmni(scene);
      return detections;
    }


    ODDetections3D* ODCADRecognizer2DLocal::detectOmni(ODSceneImage *scene)
    {

      vector<KeyPoint> keypoints_scene;
      Mat descriptor_scene;
      featureDetector->computeKeypointsAndDescriptors(scene->getCVImage(), descriptor_scene, keypoints_scene);
      scene->setDescriptors(descriptor_scene);
      scene->setKeypoints(keypoints_scene);

      ODDetections3D *detections = new ODDetections3D;
      cv::Mat viz = scene->getCVImage().clone();

      for(int i = 0; i < models.size(); i++)
      {

        ODDetection3D *detection;
        if(detectSingleModel(scene, models[i], detection, viz))
        {
          detections->push_back(detection);

          if(metainfo_)
            drawModel(viz, &models[i], pnp_detection.get_R_vect(), pnp_detection.get_t_matrix(), pnp_detection.get_A_matrix(), pnp_detection.get_dist_coef(),  yellow);
        }
      }
      detections->setMetainfoImage(viz);
      return detections;
    }

    bool ODCADRecognizer2DLocal::detectSingleModel(ODSceneImage *scene, Model const &model, ODDetection3D *&detection3D, Mat & frame_vis)
    {

      //reset
      pnp_detection.clearExtrinsics();

      vector<Point3f> list_points3d_model = model.get_points3d();  // list with model 3D coordinates
      vector<KeyPoint> list_keypoints_model = model.get_keypoints();  // list with model 3D coordinates
      Mat descriptors_model = model.get_descriptors();                  // list with descriptors of each 3D coordinate

      RobustMatcher rmatcher(model, use_gpu, use_gpu_match); // instantiate RobustMatcher


      // -- Step 1: Robust matching between model descriptors and scene descriptors
      vector<DMatch> good_matches;       // to obtain the 3D points of the model
      vector<KeyPoint> keypoints_scene = scene->getKeypoints();  // to obtain the 2D points of the scene

      Mat frame = scene->getCVImage();

      //normalize
      Mat scenedes = scene->getDescriptors();
      rmatcher.matchNormalized(scenedes, descriptors_model, good_matches);
      //rmatcher.match(scenedes, descriptors_model, good_matches);

      if(good_matches.size() <= 0) return false;

      vector<Point3f> list_points3d_model_match; // container for the model 3D coordinates found in the scene
      vector<KeyPoint> list_keypoints_model_match; // container for the model 2D coordinates in the textured image of the corresponding 3D coordinates
      vector<Point2f> list_points2d_scene_match; // container for the model 2D coordinates found in the scene

      for(unsigned int match_index = 0; match_index < good_matches.size(); ++match_index)
      {
        Point3f point3d_model = list_points3d_model[good_matches[match_index].trainIdx];  // 3D point from model
        KeyPoint kp_model = list_keypoints_model[good_matches[match_index].trainIdx];  // 2D point from model
        Point2f point2d_scene = keypoints_scene[good_matches[match_index].queryIdx].pt; // 2D point from the scene

        list_points3d_model_match.push_back(point3d_model);         // add 3D point
        list_keypoints_model_match.push_back(kp_model);
        list_points2d_scene_match.push_back(point2d_scene);         // add 2D point
      }


      Mat inliers_idx;
      vector<Point2f> list_points2d_inliers;


      // -- Step 3: Estimate the pose using RANSAC approach
      pnp_detection.estimatePoseRANSAC(list_points3d_model_match, list_points2d_scene_match, pnpMethod, inliers_idx,
                                       iterationsCount, reprojectionError, confidence);

      if(inliers_idx.rows < minInliers) return false;

      cout << "RECOGNIZED: " << model.id << endl;
      //else everything is fine; report the detection
      detection3D = new ODDetection3D();
      detection3D->setLocation(pnp_detection.get_t_matrix());
      detection3D->setPose(pnp_detection.get_R_matrix());
      detection3D->setType(ODDetection::OD_DETECTION_RECOG);
      detection3D->setId(model.id);



      return true;
    }

  }
}
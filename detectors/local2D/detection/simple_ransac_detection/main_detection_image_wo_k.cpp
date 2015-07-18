//--model=Data/Chicken/Mesh.xml --mesh=Data/Chicken/Mesh.ply
// --use_gpu --fast --method=1 --error=2 --confidence=0.9 --iterations=500 --inliers=30 --model=Data/Chicken/Mesh.xml --mesh=Data/Chicken/Mesh.ply --test_images=./Data/Chicken/Chicken/*.JPG --camera_intrinsic_file=Data/out_camera_dataset_101.yml
// --use_gpu --fast --method=1 --error=2 --confidence=0.9 --iterations=500 --inliers=30 --test_images=./Data/Lion/test_images/IMG_*.JPG --camera_intrinsic_file=Data/out_camera_data_lion_old.yml
//  --fast --method=1 --error=2 --confidence=0.9 --iterations=500 --inliers=30 --test_images=./Data/Lion/Lion2/IMG_*.JPG --camera_intrinsic_file=Data/out_camera_dataset_101.yml
//--fast --method=1 --error=2 --confidence=0.9 --iterations=500 --inliers=10 --model=Data/Totem/Param.SIFT.xml --mesh=Data/Totem/Param.ply --test_images=./Data/Totem/Totem/IMG_0251.JPG --camera_intrinsic_file=Data/out_camera_dataset_101.yml

// C++
#include <iostream>
#include <time.h>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/viz.hpp>


#include <sys/time.h>

// PnP Tutorial
#include "Mesh.h"
#include "Model.h"
#include "PnPProblem.h"
#include "RobustMatcher.h"
#include "ModelRegistration.h"
#include "Utils.h"

/**  GLOBAL VARIABLES  **/

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;


string tutorial_path = ""; // path to tutorial



//string yml_read_path = tutorial_path + "Data/cookies_ORB.yml"; // 3dpts + descriptors
//string ply_read_path = tutorial_path + "Data/box.ply";         // mesh

//string video_read_path = tutorial_path + "Data/box.mp4";       // recorded video
string yml_read_path = tutorial_path + "Data/Lion/Final.SIFT.xml"; // 3dpts + descriptors
string ply_read_path = tutorial_path + "Data/Lion/Final.ply";         // mesh
string ply_tex_path = tutorial_path + "Data/Totem/Texture.png";         // mesh


string camera_intrinsic_file = tutorial_path + "Data/out_camera_data_lion_old.yml";         // mesh

//
//string yml_read_path = tutorial_path + "Data/Lion/low_res_texture/Final.pairs"; // 3dpts + descriptors
//string ply_read_path = tutorial_path + "Data/Lion/low_res_texture/Final.ply";         // mesh
//string ply_tex_path = tutorial_path + "Data/Lion/low_res_texture/Texture_1024.png";         // mesh


//// Intrinsic camera parameters: UVC WEBCAM
//double fx = 3.51567979e+03;                           // focal length in mm
//double fy = 3.52343259e+03;
//double sx = 1, sy = 1;             // sensor size
//double cx = 2.11793527e+03, cy = 1.49318756e+03;        // image size

// Intrinsic camera parameters: UVC WEBCAM
double fx = 3.51567979e+03;                           // focal length in mm
double fy = 3.52343259e+03;
double sx = 1, sy = 1;             // sensor size
double cx = 2.11793527e+03, cy = 1.49318756e+03;        // image size

double params_WEBCAM[] = { fx,   // fx
                           fy,  // fy
                           cx,      // cx
                           cy};    // cy
//double param_dist[] = {-0.18835486,  0.09173112,  0.00188108,  0.00033594,  0.16224491};
double param_dist[] = {1.4481173997393210e-01, -3.5247562241022906e-01, 0, 0, 1.4483165395214911e-01};

// Some basic colors
Scalar red(0, 0, 255);
Scalar green(0,255,0);
Scalar blue(255,0,0);
Scalar yellow(0,255,255);


// Robust Matcher parameters
int numKeyPoints = 2000;      // number of detected keypoints
float ratioTest = 0.70f;          // ratio test
bool fast_match = true;       // fastRobustMatch() or robustMatch()
bool use_gpu = false;
bool use_gpu_match = false;

// RANSAC parameters
int iterationsCount = 500;      // number of Ransac iterations.
float reprojectionError = 2.0;  // maximum allowed distance to consider it an inlier.
double confidence = 0.95;        // ransac successful confidence.

// Kalman Filter parameters
int minInliersKalman = 30;    // Kalman threshold updating

// PnP parameters
int pnpMethod = SOLVEPNP_EPNP;


/**  Functions headers  **/
void help();
void initKalmanFilter( KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);
void updateKalmanFilter( KalmanFilter &KF, Mat &measurements,
                         Mat &translation_estimated, Mat &rotation_estimated );
void fillMeasurements( Mat &measurements,
                       const Mat &translation_measured, const Mat &rotation_measured);


class FrameGenerator
{
public:
  FrameGenerator(CommandLineParser parser)
  {
    //some hardcoded default values (TEST ONE IMAGES), the program will run even without any arguments
    cameraID = 0;
    curr_image = -1;
    string test_images = tutorial_path + "Data/Lion/test_images/IMG_20150226_102852.jpg";
    image_list = myglob(test_images);

    inputType = IMAGE_LIST;

    exhausted = false;
    cout << "yo " << parser.get<int>("cam_id") << endl;

    //parsing input from the arguments now
    if (parser.get<string>("test_images").size() > 0)
    {
      test_images = parser.get<string>("test_images");
      inputType = IMAGE_LIST;
      image_list = myglob(test_images);
    }
    if (parser.get<string>("video").size() > 0) {
      video_read_path = parser.get<string>("video");
      inputType = VIDEO_FILE;
      inputCapture.open(video_read_path);
      if (!inputCapture.isOpened()) { cout << "FATAL: Cannot open video capture!"; exhausted = true;}
    }
    if(parser.has("cam_id"))      //MAX PREFERENCE
    {
      cameraID = !parser.get<int>("cam_id") ? parser.get<int>("cam_id") : cameraID;
      inputType = CAMERA;
      inputCapture.open(cameraID);
      if (!inputCapture.isOpened()) { cout << "FATAL: Cannot open video capture!"; exhausted = true;}
    }


  }

  Mat getNextFrame()
  {
    Mat result;
    if (inputType == IMAGE_LIST)
    {
      curr_image ++ ;

      //sleep(3); //wait for 3 seconds

      if (curr_image == image_list.size() - 1)
        exhausted = true;
      cout << "Frame: " << image_list[curr_image] << endl;


//      KFeatureDetector fd("SIFT", true);
//      cv::Mat descriptors; vector<KeyPoint> kps;
      //fd.findSiftGPUDescriptors(image_list[curr_image].c_str(), descriptors, kps);

      result = imread(image_list[curr_image], IMREAD_COLOR);
//      Mat result1 = imread(image_list[curr_image], IMREAD_GRAYSCALE);
//      Mat temp;
//      cv::cvtColor(result, temp, cv::COLOR_BGR2GRAY);

      //2
//      fd.findSiftGPUDescriptors(result1, descriptors, kps );
//      fd.findSiftGPUDescriptors(temp, descriptors, kps );
//      fd.findSiftGPUDescriptors(result, descriptors, kps );

    }
    else
    {
      if( inputCapture.isOpened() ) {
        //as real time as possible
        inputCapture.read(result);
      }
      else exhausted = false;
    }

    return result;
  }

public:
  enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

  string input;

  int cameraID;
  vector<string> image_list;
  string video_read_path;
  int curr_image;
  VideoCapture inputCapture;

  InputType inputType;
  bool exhausted;


private:
  string patternToUse;


};

Mat drawMatchesAfterRansac(Mat frame_vis, vector<Point2f> l_pt_scene, vector<KeyPoint> l_kp_model, Mat inliner_idx) {

  Mat img_object = imread(ply_tex_path, IMREAD_GRAYSCALE );

  vector<KeyPoint> l_kp_scene;

  for (int i =0; i < l_pt_scene.size(); i++)
  {
    KeyPoint kp(l_pt_scene[i].x, l_pt_scene[i].y, 1); l_kp_scene.push_back(kp);
  }

  //make matches
  vector<DMatch> match;
  for (int i =0; i < inliner_idx.rows; i++)
  {
    int good_index = inliner_idx.at<int>(i);         // i-inlier
    DMatch m(good_index, good_index, 1); match.push_back(m);
  }

  Mat img_matches;
  drawMatches(frame_vis, l_kp_scene,
      img_object, l_kp_model,
      match, img_matches, Scalar::all(-1), Scalar::all(-1),
      vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  return img_matches;
}

viz::Mesh getMesh()
{
  Mat lena = imread("Data/simplecube/flower.jpeg");

  std::vector<Vec3d> points;
  std::vector<Vec2d> tcoords;
  std::vector<int> polygons;
  for(size_t i = 0; i < 64; ++i)
  {
    double angle = CV_PI/2 * i/64.0;
    points.push_back(Vec3d(0.00, cos(angle), sin(angle))*0.75);
    points.push_back(Vec3d(1.57, cos(angle), sin(angle))*0.75);
    tcoords.push_back(Vec2d(0.0, i/64.0));
    tcoords.push_back(Vec2d(1.0, i/64.0));
  }

  for(int i = 0; i < (int)points.size()/2-1; ++i)
  {
    int polys[] = {3, 2*i, 2*i+1, 2*i+2, 3, 2*i+1, 2*i+2, 2*i+3};
    polygons.insert(polygons.end(), polys, polys + sizeof(polys)/sizeof(polys[0]));
  }

  cv::viz::Mesh mesh;
  mesh.cloud = Mat(points, true).reshape(3, 1);
  mesh.tcoords = Mat(tcoords, true).reshape(2, 1);
  mesh.polygons = Mat(polygons, true).reshape(1, 1);
  mesh.texture = lena;
  return mesh;
}

void viz_exp()
{
  viz::Mesh mesh = getMesh();

  double w = 1024, h = 800;
  viz::Camera  camera(606, 606, w/2, h/2, Size(w, h));

  viz::Viz3d viz("show_textured_mesh");
  viz.setCamera(camera);


  viz.setBackgroundMeshLab();

  viz.showWidget("mesh", viz::WMesh(mesh));
  viz.setRenderingProperty("mesh", viz::SHADING, viz::SHADING_PHONG);
  viz.showWidget("text2d", viz::WText("Textured mesh", Point(20, 20), 20, viz::Color::green()));
  viz.spin();
}

/**  Main program  **/
int main(int argc, char *argv[])
{

  //viz_exp();


  help();

  const String keys =
      "{help h        |      | print this message                   }"
      "{video v       |      | path to recorded video               }"
      "{test_images img       |      | image for detection               }"
      "{cam_id         |      | pass true if you want the input from the camera             }"
      "{camera_intrinsic_file      |      | yml file containing camera parameters             }"
      "{model         |      | path to yml model                    }"
      "{mesh          |      | path to ply mesh                     }"
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
      ;
  CommandLineParser parser(argc, argv, keys);
  if (parser.has("help"))
  {
      parser.printMessage();
      return 0;
  }
  else
  {
    camera_intrinsic_file = parser.get<string>("camera_intrinsic_file").size() > 0 ? parser.get<string>("camera_intrinsic_file") : camera_intrinsic_file;
    yml_read_path = parser.get<string>("model").size() > 0 ? parser.get<string>("model") : yml_read_path;
    ply_read_path = parser.get<string>("mesh").size() > 0 ? parser.get<string>("mesh") : ply_read_path;

    use_gpu = parser.has("use_gpu");
    use_gpu_match = parser.has("use_gpu_match");

    numKeyPoints = !parser.has("keypoints") ? parser.get<int>("keypoints") : numKeyPoints;
    ratioTest = !parser.has("ratio") ? parser.get<float>("ratio") : ratioTest;
    fast_match = !parser.has("fast") ? parser.get<bool>("fast") : fast_match;
    iterationsCount = !parser.has("iterations") ? parser.get<int>("iterations") : iterationsCount;
    reprojectionError = !parser.has("error") ? parser.get<float>("error") : reprojectionError;
    confidence = !parser.has("confidence") ? parser.get<float>("confidence") : confidence;
    minInliersKalman = !parser.has("inliers") ? parser.get<int>("inliers") : minInliersKalman;
    pnpMethod = !parser.has("method") ? parser.get<int>("method") : pnpMethod;
  }


  FileStorage fs(camera_intrinsic_file, FileStorage::READ);
  Mat cam_man, dist_coeff;
  fs["Camera_Matrix"] >> cam_man;
  fs["Distortion_Coefficients"] >> dist_coeff;
  PnPProblem pnp_detection(cam_man, dist_coeff);
  PnPProblem pnp_detection_est(cam_man, dist_coeff);



  KalmanFilter KF;         // instantiate Kalman Filter
  int nStates = 18;            // the number of states
  int nMeasurements = 6;       // the number of measured states
  int nInputs = 0;             // the number of control actions
  double dt = 0.125;           // time between measurements (1/FPS)

  Mat measurements(nMeasurements, 1, CV_64F); measurements.setTo(Scalar(0));
  bool good_measurement = false;



  // 3D model and its correspondenses
  Model model;               // instantiate Model object
  model.load_new_xml(yml_read_path); // load a 3D textured object model
  Mesh mesh;                 // instantiate Mesh object
  mesh.load(ply_read_path);  // load an object mesh

  vector<Point3f> list_points3d_model = model.get_points3d();  // list with model 3D coordinates
  vector<KeyPoint> list_keypoints_model = model.get_keypoints();  // list with model 3D coordinates
  Mat descriptors_model = model.get_descriptors();                  // list with descriptors of each 3D coordinate



  RobustMatcher rmatcher(model, use_gpu, use_gpu_match); // instantiate RobustMatcher


  //Timers!
  timeval start, end, end_detect;
  Timer timer_algo = Timer();
  Timer timer_display = Timer();
  Timer timer_fps = Timer();
  timer_fps.start();


  double fps, sec;
  // frame counter
  int counter = 0;


  //Frames!
  FrameGenerator frameGenerator(parser);
  Mat frame_vis;
  Mat img_all_matches;
  Mat img_selective_matches;
  Mat frame;
  namedWindow("Overlay", WINDOW_NORMAL);



  //namedWindow("Before", WINDOW_NORMAL); namedWindow("After", WINDOW_NORMAL);

  while(!frameGenerator.exhausted && waitKey(30) != 27) // while valid frame
  {
    //read the frame
    cout << "Processing next frame ...." << endl;
    frame = frameGenerator.getNextFrame();
    frame_vis = frame.clone();    // refresh visualisation frame


    // start the clock
    timer_algo.start(); timer_display.start();


    // -- Step 1: Robust matching between model descriptors and scene descriptors
    vector<DMatch> good_matches;       // to obtain the 3D points of the model
    vector<KeyPoint> keypoints_scene;  // to obtain the 2D points of the scene


    if (fast_match) {
      rmatcher.findFeatureAndMatch(frame, good_matches, keypoints_scene, descriptors_model);
    }
    else {
      rmatcher.robustMatch(frame, good_matches, keypoints_scene, descriptors_model);
    }


    Mat img_object = imread(ply_tex_path, IMREAD_GRAYSCALE);
    drawMatches(frame, keypoints_scene,
        img_object, model.get_keypoints(),
        good_matches, img_all_matches, Scalar::all(-1), Scalar::all(-1),
        vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //imshow("Before", img_all_matches);

    // -- Step 2: Find out the 2D/3D correspondences

    vector<Point3f> list_points3d_model_match; // container for the model 3D coordinates found in the scene
    vector<KeyPoint> list_keypoints_model_match; // container for the model 2D coordinates in the textured image of the corresponding 3D coordinates
    vector<Point2f> list_points2d_scene_match; // container for the model 2D coordinates found in the scene

    for (unsigned int match_index = 0; match_index < good_matches.size(); ++match_index) {
      Point3f point3d_model = list_points3d_model[good_matches[match_index].trainIdx];  // 3D point from model
      KeyPoint kp_model = list_keypoints_model[good_matches[match_index].trainIdx];  // 2D point from model
      Point2f point2d_scene = keypoints_scene[good_matches[match_index].queryIdx].pt; // 2D point from the scene

      list_points3d_model_match.push_back(point3d_model);         // add 3D point
      list_keypoints_model_match.push_back(kp_model);
      list_points2d_scene_match.push_back(point2d_scene);         // add 2D point
    }

    // Draw outliers
    draw2DPoints(frame_vis, list_points2d_scene_match, red);


    Mat inliers_idx;
    vector<Point2f> list_points2d_inliers;

    if (good_matches.size() > 0) // None matches, then RANSAC crashes
    {

      // -- Step 3: Estimate the pose using RANSAC approach
      pnp_detection.estimatePoseRANSAC(list_points3d_model_match, list_points2d_scene_match,
          pnpMethod, inliers_idx,
          iterationsCount, reprojectionError, confidence);

      // -- Step 4: Catch the inliers keypoints to draw
      for (int inliers_index = 0; inliers_index < inliers_idx.rows; ++inliers_index) {
        int n = inliers_idx.at<int>(inliers_index);         // i-inlier
        Point2f point2d = list_points2d_scene_match[n]; // i-inlier point 2D
        list_points2d_inliers.push_back(point2d);           // add i-inlier to list

      }

      /*std::cout << pnp_detection.get_R_matrix() << std::endl;
      std::cout << pnp_detection.get_t_matrix() << std::endl;
      std::cout << pnp_detection.get_P_matrix() << std::endl;
      std::cout << pnp_detection.get_A_matrix() << std::endl << std::endl;*/
      // Draw inliers points 2D
      draw2DPoints(frame_vis, list_points2d_inliers, blue);


      //##########DEBUG DRAWING###########
      img_selective_matches = drawMatchesAfterRansac(frame_vis, list_points2d_scene_match, list_keypoints_model_match, inliers_idx);
      //imshow("After", img_selective_matches);


      if (inliers_idx.rows >= minInliersKalman) good_measurement = true;
      else good_measurement = false;

    }

    // -- Step X: Draw pose
    timer_algo.stop();
    cout << "Time taken algo: " << timer_algo.getDuration() << endl;

    if (good_measurement) {
      //drawObjectMesh1(frame_vis, &mesh,  &model, &pnp_detection, green);  // draw current pose
      pnp_detection_est.set_P_matrix(pnp_detection.get_R_matrix(), pnp_detection.get_t_matrix());
      drawObjectMesh(frame_vis, &mesh, &pnp_detection_est, yellow); // draw estimated pose
      cout << "Object detected!!!" << endl;
      if(frameGenerator.inputType == FrameGenerator::IMAGE_LIST)
        imwrite(frameGenerator.image_list[frameGenerator.curr_image] + ".detected.jpg", frame_vis);
    }
    else {
      //drawObjectMesh1(frame_vis, &mesh, &model, &pnp_detection_est, yellow); // draw estimated pose
      //drawObjectMesh(frame_vis, &mesh, &pnp_detection_est, yellow); // draw estimated pose

    }


    // FRAME RATE

    // see how much time has elapsed
    timer_display.stop();
    timer_fps.stop();

    // calculate current FPS
    ++counter;
    sec = timer_fps.getDuration();
    cout << "Time taken frame: " << timer_display.getDuration() << endl;
    fps = counter / sec;

    drawFPS(frame_vis, fps, yellow); // frame ratio
    double detection_ratio = ((double) inliers_idx.rows / (double) good_matches.size()) * 100;
    drawConfidence(frame_vis, detection_ratio, yellow);


    // -- Step X: Draw some debugging text

    // Draw some debug text
    int inliers_int = inliers_idx.rows;
    int outliers_int = (int) good_matches.size() - inliers_int;
    string inliers_str = IntToString(inliers_int);
    string outliers_str = IntToString(outliers_int);
    string n = IntToString((int) good_matches.size());
    string text = "Found " + inliers_str + " of " + n + " matches";
    string text2 = "Inliers: " + inliers_str + " - Outliers: " + outliers_str;

    drawText(frame_vis, text, green);
    drawText2(frame_vis, text2, red);

    imshow("Overlay", frame_vis);

    if (good_measurement) {
      if(frameGenerator.inputType == FrameGenerator::IMAGE_LIST)
        imwrite(frameGenerator.image_list[frameGenerator.curr_image] + ".detected.jpg", frame_vis);
    }

  }


  waitKey(0);




  cout << "GOODBYE ..." << endl;

}

/**********************************************************************************************************/
void help()
{
cout
<< "--------------------------------------------------------------------------"   << endl
<< "This program shows how to detect an object given its 3D textured model. You can choose to "
<< "use a recorded video or the webcam."                                          << endl
<< "Usage:"                                                                       << endl
<< "./cpp-tutorial-pnp_detection -help"                                           << endl
<< "Keys:"                                                                        << endl
<< "'esc' - to quit."                                                             << endl
<< "--------------------------------------------------------------------------"   << endl
<< endl;
}


Detection 2D {#detection_2d}
====
[TOC]

Detection 2D {#detection_2d1}
===

This article goes through the 2D detection methods covered in OD. Specifically, it covers the classes - od::g2d::ODHOGDetector through a tutorial.

2D detection methods are performed by the classes Detector2D. They accept a `SceneImage` and performs detection/recognition on them. Currently Detector2Ds are classified into g2d and l2d namespaces. g2d covers detection methods which uses global 2D features (like HOG/Cascade) while l2d covers detection methods which uses local 2D features (like SIFT/SURF/ORB) for detection/recognition. Different 2D detectors that are available currently: od::g2d::ODHOGDetector, od::g2d::ODCascadeDetector, od::g2d::ODFaceRecognizer, od::l2d::ODCADRecognizer2DLocal

##HOG feature based detection {#detection_2d2}

HOGDetector is a HOG feature based linear classifier. It accepts an image (od::ODSceneImage), computes its HOG feature (in a multiscale mannar for detectOmni() and a single descriptor from the resized image for detect()), runs a linear SVM obtained either by HOGTrainer through training or some default ones (from OpenCV), and informs if the classifier is true thereby providing detection.

A complete example including training is provided in `examples/objectdetector/od_hog_train.cpp`. For positive and negetive example get the data from the INRIA human dataset: http://pascal.inrialpes.fr/data/human/.
 
In this tutorial we will go through a more complete application in present in `examples/apps/global2D/od_multihog_app.cpp`. The code is provided here verbatime: 

\code{.cpp}

/** \brief Example of the usage of HOG detector
   *
   * \author Kripasindhu Sarkar
   *
   */

#include "detectors/global2D/detection/ODHOGDetector.h"
#include "common/utils/ODFrameGenerator.h"

#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"


using namespace od;
using namespace std;

cv::Size sizesingle(640, 480);

int main(int argc, char *argv[])
{
  std::string trained_data_dir(argv[1]), input_video(argv[2]), output_video = "output.avi";
  if (argc > 3) output_video = argv[3];


  //get 3 detectors of different types
  vector<string> messages; messages.push_back("Original");
  vector<g2d::ODHOGDetector*> detectors;
  g2d::ODHOGDetector *detector1 = new g2d::ODHOGDetector; //
  messages.push_back("OpenCV Default People"); detectors.push_back(detector1);

  g2d::ODHOGDetector *detector2 = new g2d::ODHOGDetector; detector2->setSvmtype(g2d::ODHOGDetector::OD_DAIMLER_PEOPLE);
  messages.push_back("OpenCV Daimler People"); detectors.push_back(detector2);

  g2d::ODHOGDetector *detector3 = new g2d::ODHOGDetector(trained_data_dir);
  messages.push_back("Custom HOG from trained data"); detectors.push_back(detector3);

  //init all detectors
  for (int i = 0; i < detectors.size(); i++) detectors[i]->init();

  //get scenes
  od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_DEVICE> frameGenerator(input_video); 
  cv::VideoWriter videoWriter(output_video, CV_FOURCC('M','J','P','G'), 25, sizesingle * 2, true);


  //GUI
  cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
  while(frameGenerator.isValid() && cv::waitKey(33) != 27)
  {
    od::ODSceneImage * scene = frameGenerator.getNextFrame();

    vector<cv::Mat> images_to_show;
    images_to_show.push_back(scene->getCVImage()); //push the first image

    //detect 3 times
    for (int i = 0; i < detectors.size(); i++)
    {
      ODDetections2D *detections =  detectors[i]->detectOmni(scene);
      if(detections->size() > 0)
        images_to_show.push_back(detections->renderMetainfo(*scene).getCVImage());
      else images_to_show.push_back(scene->getCVImage());
    }

    cv::Mat multiimage = makeCanvasMultiImages(images_to_show, sizesingle, messages);
    cv::imshow("Overlay", multiimage);
    videoWriter << multiimage;

    delete scene;
  }

  return 0;
}
\endcode

###Data {#detection_2d3}
This app compares the results of HOG based detection with three different trained classifiers. For this app you need a pre-trained hog descriptor in your `trained_data` directory. You can either train  as in `examples/objectdetector/od_hog_train.cpp`, or get the OD pre-trained data from the \ref getting_started2 "Data Repository".

For the query video, get a clip containing many pedestrians. For example you can get one from http://www.robots.ox.ac.uk/ActiveVision/Research/Projects/2009bbenfold_headpose/project.html#datasets

Run the app from the build directory as:

    examples/apps/od_multihog_app <path_to_data>/trained_data/ <input_pedestrian_video> <output_comparison_video_with_detection>

Depending on the input video, you will see something like the following:

\htmlonly
<div align="center">
<iframe width="800" height="600" src="https://www.youtube.com/embed/NaED6B-S4ks" frameborder="0" allowfullscreen></iframe>
</div>
\endhtmlonly


###Code explanation {#detection_2d4}
We first init 3 different instances of HOGDetector of different settings.  

      vector<g2d::ODHOGDetector*> detectors;
      g2d::ODHOGDetector *detector1 = new g2d::ODHOGDetector; //
      messages.push_back("OpenCV Default People"); detectors.push_back(detector1);
    
      g2d::ODHOGDetector *detector2 = new g2d::ODHOGDetector; detector2->setSvmtype(g2d::ODHOGDetector::OD_DAIMLER_PEOPLE);
      messages.push_back("OpenCV Daimler People"); detectors.push_back(detector2);
    
      g2d::ODHOGDetector *detector3 = new g2d::ODHOGDetector(trained_data_dir);
      messages.push_back("Custom HOG from trained data"); detectors.push_back(detector3);      
      
You can set different types of linear SVMs for HOG detector using `setSvmtype` function. For adding a custom SVM use the type OD_CUSTOM and set the linear SVM weight vector using `setSVMDetector()`. You have to then update the other HOG detector parameters accordingly (like winSize etc) with which the your SVM was trained.  If you give a trained data directory, it will use the xml file from the directory. Here, we set three different available HOG detectors.
  
After setting all the parameters you need to call `init()` which is done in the following line.
  
     //init all detectors
     for (int i = 0; i < detectors.size(); i++) detectors[i]->init();
     
Then we create a FrameGenerator object, which is a templated class for grabbing both Images and Point Clouds (from kinect). Use od::GENERATOR_TYPE_DEVICE for grabbing frames from camera (kinekt for Point Cloud) or video which is done in the following line.      
      
      //get scenes
      od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_DEVICE> frameGenerator(input_video); 
      
The next valid frame can be accessed now by `frameGenerator.getNextFrame();` which is being done in loop until exhaustion.   

Each ImageScene is then checked by the detector using the detectOmni() function which searches the whole image for detections. The resultant Detections2D contains information about the detections made (like Type/Bounding box etc). We use an function `renderMetainfo` to draw the bounding box of all the detections made.
 
    //detect 3 times
    for (int i = 0; i < detectors.size(); i++)
    {
      ODDetections2D *detections =  detectors[i]->detectOmni(scene);
      if(detections->size() > 0)
        images_to_show.push_back(detections->renderMetainfo(*scene).getCVImage());
      else images_to_show.push_back(scene->getCVImage());
    }

The function `od::makeCanvasMultiImages` take N images and concatenate them in order forming a big image with messages. That concatenated image with having information from all the detectors are then being shown. 
  
    cv::Mat multiimage = makeCanvasMultiImages(images_to_show, sizesingle, messages);
    cv::imshow("Overlay", multiimage);
    
    
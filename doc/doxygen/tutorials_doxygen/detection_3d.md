
Detection 3D {#detection_3d}
====
[TOC]

Detection 3D {#detection_3d1}
===========


3D detection methods are performed by the classes Detector3D. They accept a `od::ODScenePointCloud` and performs detection/recognition on them. We will go through a tutorial on global feature based detection for explanation. 


##Detection based on global features {#detection_3d2}

In this tutorial we will cover the example in `examples/objectdetector/od_pc_global_real_time.cpp which is a demo of the detector class od::g3d::ODCADDetector3DGlobal. This class detects 3D CAD models using global features of the point cloud. The corresponding Trainer class is  od::g3d::ODCADDetectTrainer3DGlobal. They both internally use the functionality of the app 3d_rec_framework of PCL.


By default CADDetector3D (there is a CADDetector2D as well which detects CAD models in a 2D image - od::l2d::ODCADRecognizer2DLocal) uses ESF (Ensemble of Shape Functions) as the global feature. Allowed features are VFH, CVFH and ESF which can be set through `setDescName()` or through the constructor. 

###Data {#detection_3d3}

You need to get 3D CAD models of the objects you want to detect in the query scene. You can download CAD models from the CAD model databases available in the internet (eg: 3D warehouse: https://3dwarehouse.sketchup.com/?hl=en) or acquire your own CAD models of the real objects from a 3D scanners (which are popular in Robotics). We have provided some example CAD models in our \ref getting_started2 "Data Repository" at location <source_to_data>/training_input/CADModels. If you don't have objects which look like them (which is obvious), you won't get successful detection. If you don't have a 3D scanner, I would suggest you to look at the CAD models in 3D warehouse which looks 'similar' to the real object. Thanks to the the 3D global features which do not require CAD models to be of exactly the same geometry as the real object. Somewhat 'similar looking' CAD model will work as well. 

The CAD models is to be placed in a strict directory structure before training which is documented in od::g3d::ODCADDetectTrainer3DGlobal. This particular demo uses kinect to acquire Point Cloud in the real time, so you need a kinect device as well.

###Code {#detection_3d4}
The code is provided here verbatim.

\code{.cpp}

/** \brief Example of the usage of global pipeline
   *
   * \author Kripasindhu Sarkar
   *
   */


#include "common/pipeline/ObjectDetector.h"
#include "common/pipeline/ODDetection.h"
#include "detectors/global3D/ODPointCloudGlobalMatching.h"
#include "common/utils/ODFrameGenerator.h"


int main(int argc, char *argv[])
{

  std::string training_input_dir(argv[1]), trained_data_dir(argv[2]);

  //trainer
  od::ODTrainer *trainer = new od::g3d::ODCADDetectTrainer3DGlobal(training_input_dir, trained_data_dir);
  trainer->train();


  //detector
  od::g3d::ODCADDetector3DGlobal<pcl::PointXYZRGBA> *detector = new od::g3d::ODCADDetector3DGlobal<pcl::PointXYZRGBA>();
  detector->setTrainingInputLocation(training_input_dir);
  detector->setTrainedDataLocation(trained_data_dir);
  detector->init();

  //GUI and feedback
  od::ODScenePointCloud<pcl::PointXYZRGBA> *frame;

  pcl::visualization::PCLVisualizer vis ("kinect");

  od::ODFrameGenerator<od::ODScenePointCloud<pcl::PointXYZRGBA>, od::GENERATOR_TYPE_DEVICE> frameGenerator;
  while(frameGenerator.isValid())
  {

    frame = frameGenerator.getNextFrame();

    //remove previous point clouds and text and add new ones in the visualizer
    vis.removeAllPointClouds();
    vis.removeAllShapes();
    vis.addPointCloud<pcl::PointXYZRGBA> (frame->getPointCloud(), "frame");

    //Detect
    od::ODDetections3D * detections = detector->detectOmni(frame);

    //add all the detections in the visualizer with its id as text
    for (size_t i = 0; i < detections->size (); i++)
    {
      std::stringstream cluster_name;
      cluster_name << "cluster_" << i;
      pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> random_handler (detections->at(i)->getMetainfoCluster());
      vis.addPointCloud<pcl::PointXYZ> (detections->at(i)->getMetainfoCluster(), random_handler, cluster_name.str ());

      pcl::PointXYZ pos; pos.x = detections->at(i)->getLocation()[0]; pos.y = detections->at(i)->getLocation()[1]; pos.z = detections->at(i)->getLocation()[2];
      vis.addText3D (detections->at(i)->getId(), pos, 0.015f, 1, 0, 1, cluster_name.str() + "_txt", 0);
    }

    vis.spinOnce ();
  }

  return 0;
}

\endcode

###Execution {#detection_3d5}

After compiling the OD tree, run the binary from the build directory as: 

    examples/objectdetector/od_example_pc_global_real_time <path_to_CAD_models> <trained_data_directory>

You will see output similar to the following video:

\htmlonly
<div align="center">
<iframe width="420" height="315" src="https://www.youtube.com/embed/kosqMb6aeMU" frameborder="0" allowfullscreen></iframe>
</div>
\endhtmlonly


###Code explanation {#detection_3d6}
The code is straightforward and self explanatory. We first initialize detector and trainer with `training input location` with CAD models and the `trained_data location`.

       //trainer
        od::ODTrainer *trainer = new od::g3d::ODCADDetectTrainer3DGlobal(training_input_dir, trained_data_dir);
        trainer->train();
            
        //detector
        od::g3d::ODCADDetector3DGlobal<pcl::PointXYZRGBA> *detector = new od::g3d::ODCADDetector3DGlobal<pcl::PointXYZRGBA>();
        detector->setTrainingInputLocation(training_input_dir);
        detector->setTrainedDataLocation(trained_data_dir);
        detector->init();     
      
We then get a FrameGenerator of PointCloud and Device types to get Point Cloud Scene from kinect device. We also instantiate a VTK window through PCLVisualizer.
 
 
    pcl::visualization::PCLVisualizer vis ("kinect");
    od::ODFrameGenerator<od::ODScenePointCloud<pcl::PointXYZRGBA>, od::GENERATOR_TYPE_DEVICE> frameGenerator;
              
We then get Point Cloud scenes in a loop and perform Omnidetection on them.
   
     frame = frameGenerator.getNextFrame();     
     //Detect
     od::ODDetections3D * detections = detector->detectOmni(frame);
     
We then add each detection made in the PCLVisualizer window with different colors and draw its classification ID on top of them.
     
     vis.addPointCloud<pcl::PointXYZ> (detections->at(i)->getMetainfoCluster(), random_handler, cluster_name.str ());
     
    pcl::PointXYZ pos; pos.x = detections->at(i)->getLocation()[0]; pos.y = detections->at(i)->getLocation()[1]; pos.z = detections->at(i)->getLocation()[2];
      vis.addText3D (detections->at(i)->getId(), pos, 0.015f, 1, 0, 1, cluster_name.str() + "_txt", 0);
            

Basic Structures {#basic_structures}
====
[TOC]

Basic Structure {#basic_structures1}
======

This article covers the basic structures and pipelines of OD. 

The basic classes in OD are Trainers  and Detectors. A Trainer (the offline stage) of a detection method  acts on `training input data` to produce intermediate data called `trained data`. A corresponding Detector (the online stage) of the same method uses the `trained data` produced by Trainer to detect or recognize object in a given `Scene` (query image/point cloud). `Trained data` is usually stored in a preconfigured directory structure depending on the method starting from the base directory set for OpenDetection - `trained data location`. 
              
The data by Trainer can be used by any of the Detectors. The Decector can use data of different types of Trainers (or no trainers at all). Therefore there is many-to-many mapping between Trainers and Detector which is currently resolved by Documentation (i.e. one needs to see the documentation to find out what Trainer to use for a given Detector). In future we plan to associate this mapping by grouping compatible Trainer/Detector under `ObjectDetector`s. 


##Trainer {#basic_structures2}

Each `Trainer` (od::ODTrainer) implements a virtual function `train` with the following signature:

    virtual int train() = 0;

##Detector {#basic_structures3}
    
Each Detector (od::ODDetector) implements two functions - `detect()` and `detectOmni()` of the following signature. `detectOmni()` performs a detection/recognition on the entire scene (unsegmented and unprocessed) and provides information about the detection as well as its exact location. detect() takes an 'object candidate' or a segmented/processed scene as an input and identifies if the entire scene is a detection.

    virtual ODDetections* detect(ODScene *scene);
    virtual ODDetections* detectOmni(ODScene *scene);
    
Depending on the type of scene, Detectors are categorised in od::Detector2D and od::Detector3D.
    
##Detection {#basic_structures4}
A result of a Detector is Detections - a collection of Detection (od::ODDetection). Detection contains detection/recognition details as well as its exact location in the scene(for example bounding box for od::ODDetection2D and location/orientation for od::Detection3D) .

##Typical code structure {#basic_structures5}

A very typical code looks covering most of the pipeline looks like: 

\code{.cpp}
//train:
od::g2d::ODHOGTrainer *trainer = new od::g2d::ODHOGTrainer("", trained_data_dir); //chose a trainer type
trainer->setPosSamplesDir(pos_samples);                                           //set all the configurations as required by the trainer, the default values are provided as well
trainer->setNegSamplesDir(neg_samples);
trainer->setNOFeaturesNeg(10);
trainer->setTrainHardNegetive(true);
trainer->train();                                                                 //train!

//detect:
ODDetector *detector = new od::g2d::ODHOGDetector;  //chose a detector type
detector->setTrainingDataLocation(trained_data_dir);
detector->init();                                   //init with the required options

//do as may detections as needed in a loop using the initialized settings:
ODDetections2D *detections =  detector->detectOmni(scene); //Use the detect* methods for detection. sene is a scene object from frameGenerator

//infer
showimage(detections->renderMetainfo(*scene).getCVImage())  //do something with the detections, 
\endcode


For more details please take a look into the examples provided with the `examples` folder in the repository and the rest of the user guide.

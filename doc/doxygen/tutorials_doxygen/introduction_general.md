Introduction {#introduction_general}
================

[TOC]


Philosophy and motivation {#introduction_general1}
===

Object detection and recognition is the most important focus of Computer Vision. We are constantly in search of methods to have a 'detection' or 'recognition' system as powerful as the human being. Given a random query image, we want to have a system capable of answering the questions like: What is present in the image? Is there a house in it? Is there a human in it? Where is the house located?  Questions like these which are so trivial to human beings, are extremely difficult for a computer - the same computer which is way faster than human beings in terms of sequential calculations and executions. 

Objective {#introduction_general2}
---
* The library is built with a very specific goal - to answer the fundamental problem of Computer Vision - Object Recognition and Detection.
* We make available to everyone the existing solution in this direction in a common, intuitive and user-friendly APIs. 


Why OpenDetection? {#introduction_general3}
---
From the Computer Vision research community, there has been good solutions to very specific tasks; for example, face detection in an image, detection of human beings, or of some objects of very particular kinds (high texture/regular etc.). The methods for solving these different tasks can be very different and highly targeted for that special object type. Because of this reason we have implementation of crude detection/recognition algorithms scattered over different places without any common standard. Even with the availability of dedicated computer vision libraries like OpenCV/PCL, implementation of a simple object detection application based on the existing techniques is not a trivial task as it should be. 

Thus, the purpose of this library is to contribute in this field and make available the existing solution in this direction in a common, very structured, and a very user-friendly APIs. Every detection algorithm here is as simple as calling a function, and one can fine tune the algorithm changing the configurable parameters. 

The main philosophy is that all the APIs of detection we provide, irrespective of the type of detection and recognition is: 

1. Consistent and easy to use.
2. Fast
3. Follow a proper structure.


To detect face using the default cascade detector, the usage looks like: 
\code{.cpp}
od::g2d::ODCascadeDetector *detector = new od::g2d::ODCascadeDetector;  //chose a detector type
detector->setTrainingDataLocation(trained_cascade);
detector->init();                                   //init with the required options, default values are provided
ODDetections2D *detections =  detector->detectOmni(scene); //Use the detect* methods for detection.  sene is a scene object from frameGenerator

showimage(detections->renderMetainfo(*scene).getCVImage())  //do something with the detections, 
\endcode


For detecting people with a default pretrained HOG based detector, the usage looks like:
 
\code{.cpp}
ODDetector *detector = new od::g2d::ODHOGDetector;  //chose a detector type
detector->init();                                   //init with the required options, default values are provided
ODDetections2D *detections =  detector->detectOmni(scene); //Use the detect* methods for detection.  sene is a scene object from frameGenerator

showimage(detections->renderMetainfo(*scene).getCVImage())  //do something with the detections, 
\endcode

To train your own HOG based detector and then use the trained detector for detection, the usage looks like:

\code{.cpp}
//train
od::g2d::ODHOGTrainer *trainer = new od::g2d::ODHOGTrainer("", trained_data_dir); //chose a trainer type
trainer->setPosSamplesDir(pos_samples);                                           //set all the configurations as required by the trainer, the default values are provided as well
trainer->setNegSamplesDir(neg_samples);
trainer->setNOFeaturesNeg(10);
trainer->setTrainHardNegetive(true);
trainer->train();                                                                 //train!

//detect: do as many detection using the trained value
ODDetector *detector = new od::g2d::ODHOGDetector;  //chose a detector type
detector->setTrainingDataLocation(trained_data_dir);
detector->init();                                   //init with the required options
ODDetections2D *detections =  detector->detectOmni(scene); //Use the detect* methods for detection. sene is a scene object from frameGenerator

//infer
showimage(detections->renderMetainfo(*scene).getCVImage())  //do something with the detections, 
\endcode

And so on... Our APIs are highly structured and follow deep polymorphism for the detector hierarchy. The library is in C++ for the fast implementation. The details are provided in the next sections. 

Usage and target audience {#introduction_general4}
----
The project was originated with the aim of having a vision tool for robotics (in particular for [Robocomp](https://github.com/robocomp/robocomp)). So, we have the following different categories of target audience. 
   
   1. **Robotics Applications and robots:** The simple APIs does not involve one to know the intricate details of the application, thereby making this library a good application tools - what is required for robotics.
   2. **Computer Vision beginners and enthusiasts:**  On the same line as our APIs are simple and easy-to-use, its a great tool for experimentation for new computer vision scholars.
   3. **Computer Vision researchers:** The method dependent parameters to fine-tune detections to its limit, makes this a great tool for computer vision experts. It is also a great tool for comparison of results different methods. After all this is just a collection of various detection/recognition method together under one simple roof.  


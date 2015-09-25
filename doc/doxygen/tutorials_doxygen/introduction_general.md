Introduction {#introduction_general}
================

Philosophy and motivation {#phil}
====

Object detection and recognition is the most important focus of Computer Vision. After all we are constantly in search of methods to have a 'detection' or 'recognition' system as powerful as the human being. Given a random query image, we want to have a system capable of answering the questions like: What is present in the image? Is there a house in it? Is there a human in it? Where is the house located? What is its rough orientation w.r.t. human? 

Questions like these which are so trivial to human beings, are extremely difficult for a computer. The same computer which is so fast compared to human beings in terms of sequential calculations and executions, fails for this general task. To solve this very specific and interesting problem, originated the field of Computer Vision. Though Vision branches to a large variety of sub topics now, its ultimate core and the collective goal remains the same, the reason it is originated - scene 'understanding' and object 'understanding' in an image; which in other words are, object detection and recognition.   
   
   
Computer Vision community has been focused for many decades to solve this problem or at least parts of the problem. There has been good solution for some very specific tasks, for example, face detection in an image, or detection of human beings, or of some very particular types of object (highly textured objects, objects with distinct shapes, objects with distinct properties). The methods for solving these tasks can be very different and highly targeted to solve that particular specialized object detection problem. Because of this reason we have implementation of crude detection/recognition techniques scattered over different places without any common standard. They are floating around in different research communities with different notations and usage conventions. Even with the availability of dedicated computer vision libraries like OpenCV, implementation of a simple object detection application based on the existing techniques is not trivial and most of the time quite abstruse.

Therefore, we come up with this library with a very specific goal - to answer the fundamental problem of Computer Vision - Object Recognition and Detection. The purpose of this library is to contribute in this field and make available to everyone the existing solution in this direction in a common, intuitive and user-friendly APIs.

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

And so on...

Usage and target audience {#targetaud} 
=======




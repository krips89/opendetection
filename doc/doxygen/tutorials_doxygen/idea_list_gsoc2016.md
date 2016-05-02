
GSoC 2016 Ideas {#idea_list_gsoc2016}
====
[TOC]

Google Summer of Code 2016 - Ideas  {#idea_list_gsoc20161}
====

**We are in!**

We are selected in Google Summer of Code 2016 as a mentoring organization! <a href="https://summerofcode.withgoogle.com/organizations/6007728078061568/"><b>This</b></a> is our page in the GSoC site. The project ideas are discussed in this page. Note that *we changed the descriptions of the project ideas slightly* from the time of our application.

Student participating should already have some idea of the detection framework of the library. The best way to get started is to follow some examples and demos after going through \ref getting_started guide.

Application should go through the mentors for few iterations. Post the application at https://groups.google.com/forum/#!forum/opendetection **as well as the respective mentors.** The mentors can decide to take the topic offline and discuss with the applicant with more depth based on the applicant's expertise. 
 
##Basic principles {#idea_list_gsoc201611}
The main philosophy of this library is to provide user detection/recognition algorithms in a **common, intuitive and user-friendly APIs** (perhaps we want to build up a structure similar to VTK for object detection algorithms). This is the only reason we have this separate library. Therefore, please go through the existing <a href="inherits.html"><b>classes</b></a> to understand the framework before your application.
 
 Thus, all the algorithms/methods you propose in your application have to extend the previous interfaces which will abstract all the implementation details from the user, and at the same time provide users the ability to configure with parameters. In other words, we don't want one of the many demo applications of detection algorithms. We want the detection algorithms under the existing APIs (Trainers and Detectors). This should be the main focus of the contribution. 
 
If you think that some of our APIs and framework interfaces are not optimal (in terms of usability and speed or any other reason) and have some suggestions for the improvement, please provide them in your application (specifically in Project Idea 2) as well.
 
##Project Idea summary {#idea_list_gsoc20162}
Following are the list of ideas we will focus for this time in decreasing order of priority.


###1. CNN based algorithms in OpenDetection {#idea_list_gsoc20163}

**Key tasks**: 
* Make a short report of the popular and state-of-the-art detection/recognition techniques (to let us know that you have the proficiency in this area)
* Add CNN based algorithms in the library with OpenDetection Compatible APIs in the following order: - 
* Implement OD interface for training and testing (under od::ODTrainer s) neural network architectures with convolutional layer and possible recurrent layers using Caffe.
* Implement interface for forward phase of object detection (under od::ODDetector s or one of its subclasses) on some pre-trained caffe models (or trained models from the previous step) based on architectures which have been used for object detection. 
* Implement train/test data preparation/region extraction and other preprocessing steps (maybe in a separate module).
* Implement any other state-of-the-art detection architectures which uses CNNs.  

**Prerequisites**:

* We are expecting a Master student/PhD student in Machine Learning/Computer Vision for this project. If not, you should have a good knowledge in neural networks and experience in training and testing different types of neural networks/CNNs. 
* Knowledge of Caffe. 
* High command in C++. 

**Mentors**: Kripasindhu Sarkar, Aditya Tewari 

###2. Framework design and library maintenance {#idea_list_gsoc20164}

This is one of the important project and have a very strong influence to the future of the library. 
 
**Key tasks**:  
 * Improve the existing OpenDetection framework in terms of usability.
 * Adding more helper functions/classes to improve the library 
    - Some wrappers for visualization - right now we directly use opencv functions and pcl visualizers
    - Other changes
 * Preparing ProjectConfig.cmake file to make the library usable by other projects. 
 * Verifying and fixing cross platform compilation.
 * Package creation for different platforms.
 * Reduce dependencies of the library to minimum. 
 * General maintenance

**Prerequisites**:
* Good knowledge of Software Engineering principles. 
* High command in C++ and **CMake** (Good knowledge of CMake is very important).
* Knowledge of OpenCV/PCL.
* It will be nice if you have already maintained/managed a C++ library before. If not, you should have experience coding in different platforms (at least have knowledge in build toolchains in windows and linux). 

**Mentors**: Marco A. Gutiérrez, Kripasindhu Sarkar 

###3. Integration of some other popular detection algorithms in OpenDetection {#idea_list_gsoc20165}

**Key tasks**:
* This is an open idea. We always would like to integrate **state-of-the-art** algorithms in our framework. It would be easy to port an available C++ code in this framework instead of implementing an idea/paper from scratch. So, we expect you to *do some literature review and report different algorithms* with their availability and the ones you would like to add. Please note that, just because the implementation of some algorithm is available online does not mean that it is the state-of-the-art or one of the popular algorithms. 
* Implement/port the identified detection algorithms under od::ODDetector2D/od::ODDetector3D and the respective *Trainer* s. 
 
**Prerequisites**: Expertise in Computer Vision and C++. 

**Mentors**: Kripasindhu Sarkar, Marco A. Gutiérrez 


##Mentors:

    Kripasindhu Sarkar
    PhD Student, Augmented Vision Group, 
    German Research Center for Artificial Intelligence (DFKI), Kaiserslautern
    kripasindhu.sarkar@dfki.de             
    
    Aditya Tewari  
    PhD Student, Augmented Vision Group, 
    German Research Center for Artificial Intelligence (DFKI), Kaiserslautern
    aditya.tewari@dfki.de
    
    Marco A. Gutiérrez    
    PhD Student, RoboLab,
    University of Extremadura
    marcog@unex.es    
    
    
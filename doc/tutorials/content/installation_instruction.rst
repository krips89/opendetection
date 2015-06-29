.. _installation_instruction:

Installation Instructions
=========================

To compile OD from source, install its dependencies as stated below.


Dependencies
------------

1. OpenCV 3.0
    
    :Source: http://opencv.org/downloads.html or https://github.com/Itseez/opencv
    
        
    Required setting
    
    - OpenCV contrib (for xfeatures2d module handing SIFT/SURF features):
     
           Detailed instructions with source: https://github.com/itseez/opencv_contrib
           
           :Additional CMAKE options: OPENCV_EXTRA_MODULES_PATH=<opencv_contrib>/modules
           
    - OpenCV CUDA module (for GPU enabled feature detectors and matcher): 
    
           :Additional CMAKE options: WITH_CUDA=ON
    
    Compile and install Opencv 3.0 with the above settings.
 
 
2. VTK 6.0 or above
    
    Just download and compile latest VTK with its default settings.
    
   
3. PCL 1.6 or above    
    
    *Source:* https://github.com/PointCloudLibrary/pcl or https://github.com/PointCloudLibrary/pcl/releases
    
    Required settings:
    
    * 3d_rec_framework (for ESF, ESF etc recognition pipeline)
    
        :Additional CMAKE options: BUILD_apps=ON, BUILD_apps_3d_rec_framework=ON
    
4. Eigen

OD
--
Download the source from https://github.com/krips89/opendetection. With the above dependencies installed, OD should compile fine. If you still encounter a problem shoot an email at krips.from.iit.kgp@gmail.com.


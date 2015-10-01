
Getting Started {#getting_started}
====
[TOC]

Getting Started {#getting_started11}
====

##Install OD {#getting_started1}

First install OD as explained in \ref installation_instruction. Basically, after installing dependency you will be downloading and building OpenDetection with: 

@code{.bash}
git clone https://github.com/krips89/opendetection.git

cd opendetection 
mkdir build; cd build
cmake ..
make -j8
@endcode

##Get Data {#getting_started2}
 
After compiling OD, you will need some data to train or some existing pre-trained data to run any of the examples in the `examples` directory. To help get started we have provided our own data in the repo opendetection_data. Download our data from  https://github.com/krips89/opendetection_data using
  
@code{.bash}
git clone https://github.com/krips89/opendetection_data.git
@endcode 

Make `<path_to_data>/trained_data` as your trained_data location in all the examples, as pre-trained data of some `Detector`s are already provided with proper directory structure.


##Run your first examples {#getting_started3}

Go to the build directory of OD (<path_to_source>/build). Run a face detector from your webcam using: 

@code{.bash}
./examples/objectdetector/od_cascade_cam <path_to_data>/trained_data/
@endcode 

Or get some images containing people (suppose in <path_to_images>/*.JPG) and run the people detector using: 
@code{.bash}
./examples/objectdetector/od_image_hog_files <path_to_data>/trained_data/ "<path_to_images>/*.JPG"
@endcode 

Depending on the image, you will see something like the following.

\image html snap_hog3.png



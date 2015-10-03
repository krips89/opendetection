
Writing your own app{#writing_a_new_app}
====
[TOC]

Writing your own app{#writing_a_new_app1}
======

Put your c++ files in a suitable folder under example/apps. In the apps/CMakeLists.txt add the following line: 

    OD_ADD_EXAMPLE(<name_of_app> FILES <source_files> 
                LINK_WITH od_common <od_detector_lib_list>)
                
Where `<od_detector_lib_list>` is the list of OpenDetection libraries. Following are the currently available libraries for different detectors types:
                
* local2d: od_local_image_detector
* global2d: od_global_image_detector
* global3d: od_pointcloud_global_detector
* miscellaneous: od_miscellaneous_detector
                
Have a look at the existing CMakeLists.txt in `examples/apps` or `examples/objectdetector` for sample.
 
                 
OD_ADD_EXAMPLE(od_image_camera FILES od_image_cadrecog_camera.cpp
                LINK_WITH od_common od_local_image_detector)

OD_ADD_EXAMPLE(od_example_files FILES od_image_cadrecog_files.cpp
                LINK_WITH od_common od_local_image_detector)






OD_ADD_EXAMPLE(od_hog_train FILES od_hog_train.cpp
                LINK_WITH od_common od_global_image_detector)

OD_ADD_EXAMPLE(od_image_hog_files FILES od_image_hog_files.cpp
                LINK_WITH od_common od_global_image_detector)

OD_ADD_EXAMPLE(od_image_customhog FILES od_image_customhog.cpp
                LINK_WITH od_common od_global_image_detector)

OD_ADD_EXAMPLE(od_cascade_cam FILES od_cascade_cam.cpp
                LINK_WITH od_common od_global_image_detector)

OD_ADD_EXAMPLE(od_cascade_files FILES od_cascade_files.cpp
                LINK_WITH od_common od_global_image_detector)

OD_ADD_EXAMPLE(od_image_facerecog FILES od_image_facerecog.cpp
                LINK_WITH od_common od_global_image_detector)




OD_ADD_EXAMPLE(od_example_pc_global FILES od_pc_global.cpp
                LINK_WITH od_common od_pointcloud_global_detector)

OD_ADD_EXAMPLE(od_example_pc_global_files FILES od_pc_global_files.cpp
                LINK_WITH od_common od_pointcloud_global_detector)

OD_ADD_EXAMPLE(od_example_pc_global_real_time FILES od_pc_global_real_time.cpp
                LINK_WITH od_common od_pointcloud_global_detector)

OD_ADD_EXAMPLE(od_example_framegenerator FILES od_example_framegenerator.cpp
                LINK_WITH od_common)



OD_ADD_EXAMPLE(od_multialgo_files FILES od_multialgo_files.cpp
                LINK_WITH od_miscellaneous_detector)

OD_ADD_EXAMPLE(od_multialgo_pc FILES od_multialgo_pc.cpp
                LINK_WITH od_miscellaneous_detector)


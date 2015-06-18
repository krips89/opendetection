OD_ADD_EXAMPLE(od_example_usage FILES od_example_usage.cpp
                LINK_WITH od_common od_local_image_detector)

OD_ADD_EXAMPLE(od_example_pc_global FILES od_example_pc_global.cpp
                LINK_WITH od_common od_pointcloud_global_detector)
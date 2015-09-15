OD_ADD_EXAMPLE(odcadrecog_test_single_model FILES cadrecog2d/od_test_single_db_single_model.cpp
                LINK_WITH od_common od_local_image_detector)
add_custom_command(TARGET odcadrecog_test_single_model POST_BUILD COMMAND cp odcadrecog_test_single_model /home/sarkar/libs/usr/local/bin/)
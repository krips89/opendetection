OD_ADD_EXAMPLE(odcadrecog_test_single_model FILES cadrecog2D/od_test_single_db_single_model.cpp
                LINK_WITH od_common od_local_image_detector)

OD_ADD_EXAMPLE(od_multihog_app FILES global2D/od_multihog_app.cpp
                LINK_WITH od_common od_global_image_detector)
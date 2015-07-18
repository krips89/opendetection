set(SUBSYS_NAME pointcloud_global_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The global detector for point cloouds")
set(SUBSYS_DEPS od_common ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} pugixml siftgpu)

set(build TRUE)
#PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
#PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})


if(build)

    #add_subdirectory(detection/simple_ransac_detection)

    set(incs
        "ODPointCloudGlobalMatching.h"

        "detection/ODPointCloudGlobalMatchingDetector.h"
        "training/ODPointCloudGlobalMatchingTrainer.h"
        )

    set(impl_incs
        )

    set(srcs
        "ODPointCloudGlobalMatching.cpp"

        "detection/ODPointCloudGlobalMatchingDetector.cpp"
        "training/ODPointCloudGlobalMatchingTrainer.cpp"

        #${SUB_IMPL_SRCS}
        )

    #include all the individual implementation dirs


    #set mandatory directories
    include_directories("${OD_SOURCE_DIR}" )

    OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${srcs} INCS ${incs} ${impl_incs})


    if(SUBSYS_DEPS)
      target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS})
    endif(SUBSYS_DEPS)

    #PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")


endif(build)
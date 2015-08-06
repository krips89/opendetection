set(SUBSYS_NAME miscellaneous_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "global feature based detection in 2D images")

set(SUBSYS_DEPS od_common od_global_image_detector od_local_image_detector ${OpenCV_LIBS})


set(build TRUE)
#PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
#PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})


if(build)

    set(incs
        "detection/ODDetectorMultiAlgo.h"
        )

    set(impl_incs
        )

    set(srcs
        "detection/ODDetectorMultiAlgo.cpp"
        )


    OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${srcs} INCS ${incs} ${impl_incs})


    if(SUBSYS_DEPS)
      target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS})
    endif(SUBSYS_DEPS)

    #PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")


endif(build)
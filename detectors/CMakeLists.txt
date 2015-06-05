set(SUBSYS_NAME detectors)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The detector module of OpenDetection having the implementation of all the detectors")
set(SUBSYS_DEPS ${OpenCV_LIBS} od_common)

set(build TRUE)
#PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
#PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})


if(build)

    set(submodules
          image_local
    )

    set(incs
        "image_local/ODImageLocalMatchingSimple.h"
        )

    set(impl_incs
        )

    set(srcs
        "image_local/ODImageLocalMatchingSimple.cpp"
        )




    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/image_local")

    #!!!!!!!!!!!!!!CAUTION TEMPORARY HACK!!!!!!!!!!!!!!!!!!!!!!
    include_directories("${OD_SOURCE_DIR}/common/pipeline")

    OD_ADD_LIBRARY("${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    OD_ADD_INCLUDES("${SUBSYS_NAME}" ${incs} ${impl_incs})

    if(SUBSYS_DEPS)
      target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS})
    endif(SUBSYS_DEPS)

    #PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")


endif(build)
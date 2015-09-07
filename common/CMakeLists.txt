set(SUBSYS_NAME common)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The core module of OpenDetection having the pipeline logic")
set(SUBSYS_DEPS ${OpenCV_LIBS} ${PCL_LIBRARIES})

if(WITH_GPU)
  set(SUBSYS_DEPS ${SUBSYS_DEPS} siftgpu)
endif()

set(build TRUE)


if(build)

    set(submodules
          pipeline
    )

    set(incs
        "pipeline/ObjectDetector.h"
        "pipeline/ODAlgorithmBase.h"

        "pipeline/ODScene.h"
        "pipeline/ODDetection.h"

        "pipeline/ODTrainer.h"
        "pipeline/ODDetector.h"

        "utils/ODFrameGenerator.h"
        "utils/utils.h"
        "utils/ODFeatureDetector2D.h"

        "bindings/svmlight.h"
        )

    set(impl_incs
        )

    set(srcs
        "pipeline/ObjectDetector.cpp"
        "pipeline/ODScene.cpp"
        "utils/utils.cpp"
        "utils/ODFeatureDetector2D.cpp"
        )

    OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${srcs} INCS ${incs} ${impl_incs})
    install(FILES ${incs} DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${SUBSYS_NAME} COMPONENT ${LIB_NAME})

    if(SUBSYS_DEPS)
      target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS})
    endif(SUBSYS_DEPS)

    #PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")


endif(build)
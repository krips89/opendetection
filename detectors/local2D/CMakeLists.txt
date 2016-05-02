set(SUBSYS_NAME local_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The local feature matching based detector")

set(SUBSYS_DEPS od_common ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} pugixml)


set(build TRUE)
#PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
#PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})


if(build)

    #include all the individual implementation dirs
    add_subdirectory(detection/simple_ransac_detection)

    set(incs
        "ODImageLocalMatching.h"

        "training/ODCADRecogTrainerSnapshotBased.h"
        "detection/ODCADRecognizer2DLocal.h"

        )

    set(impl_incs
        )

    set(srcs
        "ODImageLocalMatching.cpp"
        "training/ODCADRecogTrainerSnapshotBased.cpp"
        "detection/ODCADRecognizer2DLocal.cpp"

        ${SUB_IMPL_SRCS}
        )


    OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${srcs} INCS ${incs} ${impl_incs})
    install(FILES ${incs} DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${SUBSYS_NAME} COMPONENT ${LIB_NAME})

    if(SUBSYS_DEPS)
      target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS})
    endif(SUBSYS_DEPS)

    #PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")


endif(build)
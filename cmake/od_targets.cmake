
# installation structure
if(NOT OD_INSTALL_RUNTIME_DIR)
  set(OD_INSTALL_RUNTIME_DIR bin)
endif()
if(NOT OD_INSTALL_LIBRARY_DIR)
  set(OD_INSTALL_LIBRARY_DIR lib)
endif()
if(NOT OD_INSTALL_ARCHIVE_DIR)
  set(OD_INSTALL_ARCHIVE_DIR lib)
endif()
if(NOT OD_INSTALL_INCLUDE_DIR)
  set(OD_INSTALL_INCLUDE_DIR include/od-{OD_VERSION}/od)
endif()
if(NOT OD_INSTALL_DATA_DIR)
  set(OD_INSTALL_DATA_DIR share/od-${OD_VERSION})
endif()
if(NOT OD_INSTALL_DOC_DIR)
  set(OD_INSTALL_DOC_DIR share/doc/od-${OD_VERSION})
endif()
if(NOT OD_INSTALL_PACKAGE_DIR)
  set(OD_INSTALL_PACKAGE_DIR "lib/cmake/od-${OD_VERSION}")
endif()
if(NOT OD_INSTALL_DOXYGEN_DIR)
  set(OD_INSTALL_DOXYGEN_DIR ${OD_INSTALL_DOC_DIR}/doxygen)
endif()

# global installation type
set(OD_LIB_TYPE SHARED)

# usefull functions called
macro(OD_ADD_LIBRARY1 _name _srcs _incs _impl_incs)

    set(lib_name "od_${_name}")

    message(input to od_add_library: ${OD_LIB_TYPE} ${_srcs} ${_incs} ${_impl_incs})
    add_library(${lib_name} ${OD_LIB_TYPE} ${_srcs} ${_incs} ${_impl_incs})

    # allways link libs:
    target_link_libraries(${lib_name} ${Boost_LIBRARIES})

    # target properties
    set_target_properties(${lib_name} PROPERTIES
        VERSION ${OD_VERSION}
        SOVERSION ${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}
        )

    # Install library
    install(TARGETS ${lib_name}
        RUNTIME DESTINATION ${OD_INSTALL_RUNTIME_DIR} COMPONENT ${lib_name}
        LIBRARY DESTINATION ${OD_INSTALL_LIBRARY_DIR} COMPONENT ${lib_name}
        ARCHIVE DESTINATION ${OD_INSTALL_ARCHIVE_DIR} COMPONENT ${lib_name})

    #install includes
    install(FILES ${_incs} ${_impl_incs}
            DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${_name}
            COMPONENT ${lib_name})

endmacro(OD_ADD_LIBRARY1)

macro(OD_ADD_LIBRARY _name)

    set(lib_name "od_${_name}")

    message(input to od_add_library: ${OD_LIB_TYPE} ${ARGN})
    add_library(${lib_name} ${OD_LIB_TYPE} ${ARGN})

    # allways link libs:
    target_link_libraries(${lib_name} ${Boost_LIBRARIES})

    # target properties
    set_target_properties(${lib_name} PROPERTIES
        VERSION ${OD_VERSION}
        SOVERSION ${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}
        )

    # Install library
    install(TARGETS ${lib_name}
        RUNTIME DESTINATION ${OD_INSTALL_RUNTIME_DIR} COMPONENT ${lib_name}
        LIBRARY DESTINATION ${OD_INSTALL_LIBRARY_DIR} COMPONENT ${lib_name}
        ARCHIVE DESTINATION ${OD_INSTALL_ARCHIVE_DIR} COMPONENT ${lib_name})

    #install includes
    install(FILES ${_incs} ${_impl_incs}
            DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${_name}
            COMPONENT ${lib_name})

endmacro(OD_ADD_LIBRARY)



macro(OD_ADD_INCLUDES _name)
    #install includes
    install(FILES ${ARGN}
            DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${_name}
            COMPONENT ${lib_name})

endmacro(OD_ADD_INCLUDES)
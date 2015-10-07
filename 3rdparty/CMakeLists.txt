#add mandatory 3rd party builds
add_subdirectory(pugixml-1.6)

if(WITH_SVMLIGHT)
  add_subdirectory(svmlight)
endif(WITH_SVMLIGHT)

if(WITH_GPU)
  add_subdirectory(SiftGPU)
endif(WITH_GPU)
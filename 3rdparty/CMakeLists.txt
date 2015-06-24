#add mandatory 3rd party builds
add_subdirectory(pugixml-1.6)

if(WITH_GPU)
  #option(SIFTGPU_ENABLE_CUDA "Enable cuda for SIFTGPU" OFF)
  add_subdirectory(SiftGPU)
endif(WITH_GPU)
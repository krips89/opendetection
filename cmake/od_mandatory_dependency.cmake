find_package(PCL REQUIRED)
find_package( OpenCV REQUIRED )
find_package(VTK REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED )

include_directories("${OD_SOURCE_DIR}" ${EIGEN_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} /home/sarkar/reconstruction/SiftGPU/src/SiftGPU)
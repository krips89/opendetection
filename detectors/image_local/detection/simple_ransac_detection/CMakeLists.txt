set(simple_ransac_src_dir ${CMAKE_CURRENT_SOURCE_DIR})

set(SUB_IMPL_SRCS
        ${simple_ransac_src_dir}/CsvReader.cpp
        ${simple_ransac_src_dir}/CsvWriter.cpp
        ${simple_ransac_src_dir}/ModelRegistration.cpp
        ${simple_ransac_src_dir}/Mesh.cpp
        ${simple_ransac_src_dir}/Model.cpp
        ${simple_ransac_src_dir}/PnPProblem.cpp
        ${simple_ransac_src_dir}/Utils.cpp
        ${simple_ransac_src_dir}/RobustMatcher.cpp
        ${simple_ransac_src_dir}/KFeatureDetector.cpp
PARENT_SCOPE)

#add_executable(ObjectDetection ${sample_dir}main_detection_video.cpp ${sample_pnplib} )
#include_directories(/home/sarkar/reconstruction/SiftGPU/src/SiftGPU)
#add_executable(ObjectDetection ${sample_dir}main_detection_image_wo_k.cpp ${sample_pnplib})
#add_executable(scale_intrinsic ${sample_dir}scale_intrinsic.cpp)

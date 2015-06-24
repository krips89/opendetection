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
PARENT_SCOPE)
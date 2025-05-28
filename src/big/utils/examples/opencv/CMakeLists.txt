cmake_minimum_required(VERSION 3.2)
project(nncase_sdk C CXX)

set(k230_sdk "${PROJECT_SOURCE_DIR}/../../../../../")
set(CMAKE_EXE_LINKER_FLAGS "-T ${PROJECT_SOURCE_DIR}/cmake/link.lds --static")

# set opencv
set(k230_opencv ${k230_sdk}/src/big/utils/lib/opencv)
include_directories(${k230_opencv}/include/opencv4/)
link_directories(${k230_opencv}/lib  ${k230_opencv}/lib/opencv4/3rdparty)

# set mmz
link_directories(${k230_sdk}/src/big/mpp/userapps/lib)

# # set openblas
# set(k230_openblas ${k230_sdk}/src/big/ai/examples/test_openblas_rtt_case/install)
# include_directories(${k230_openblas}/include/)
# link_directories(${k230_openblas}/lib)

add_subdirectory(1_opencv_calcHist)
add_subdirectory(2_opencv_threshold)
add_subdirectory(3_opencv_findContours)
add_subdirectory(4_opencv_features2d)
add_subdirectory(5_opencv_objdetect)

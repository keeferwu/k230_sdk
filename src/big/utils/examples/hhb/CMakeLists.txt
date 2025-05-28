cmake_minimum_required(VERSION 3.2)
project(hhb C CXX)

set(utils_root "${PROJECT_SOURCE_DIR}/../../")

set(CMAKE_EXE_LINKER_FLAGS "-T ${PROJECT_SOURCE_DIR}/cmake/link.lds --static")

include_directories(${utils_root}/lib/csi-nn2/include)
include_directories(${utils_root}/lib/hhb-prebuilt-decode/include)
link_directories(${utils_root}/lib/csi-nn2/lib)
link_directories(${utils_root}/lib/hhb-prebuilt-decode/lib)

add_subdirectory(mbv2_onnx_int8)

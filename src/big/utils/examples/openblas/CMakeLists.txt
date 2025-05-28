cmake_minimum_required(VERSION 3.2)
project(nncase_sdk C CXX)

set(k230_sdk "${PROJECT_SOURCE_DIR}/../../../../../")
set(CMAKE_EXE_LINKER_FLAGS "-T ${PROJECT_SOURCE_DIR}/cmake/link.lds --static")

# set mmz
link_directories(${k230_sdk}/src/big/mpp/userapps/lib)

# set openblas
set(k230_openblas ${k230_sdk}/src/big/utils/lib/openblas)
include_directories(${k230_openblas}/include/)
link_directories(${k230_openblas}/lib)

add_subdirectory(1_openblas_level1)
add_subdirectory(2_openblas_level2)
add_subdirectory(3_openblas_level3)
add_subdirectory(4_fortran_example)

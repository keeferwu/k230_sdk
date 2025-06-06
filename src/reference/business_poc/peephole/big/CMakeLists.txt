cmake_minimum_required(VERSION 3.2)
include(../../../../big/nncase/examples/cmake/Riscv64.cmake)

project(peephole_dev C CXX)

set(nncase_sdk_root "${PROJECT_SOURCE_DIR}/../../../../big/nncase")
set(k230_sdk ${nncase_sdk_root}/../../../)
set(CMAKE_EXE_LINKER_FLAGS "-T ${nncase_sdk_root}/examples/cmake/link.lds --static")
set(ipcmsg_root ${k230_sdk}/src/common/cdk/user/component/ipcmsg)
set(datafifo_root ${k230_sdk}/src/common/cdk/user/component/datafifo)
set(media_root "${PROJECT_SOURCE_DIR}/media")
set(person_detect_root "${PROJECT_SOURCE_DIR}/person_detect")
set(comm_server_root "${PROJECT_SOURCE_DIR}/comm_server")
set(app_root "${PROJECT_SOURCE_DIR}/app")

set(k230_opencv ${k230_sdk}/src/big/utils/lib/opencv)
set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/../" CACHE STRING "The path to use for make install" FORCE)
set(CMAKE_INSTALL_PREFIX "../")
file(GLOB APP_SOURCES
    ${media_root}/src/*.cpp
    ${person_detect_root}/src/*.cpp
    ${app_root}/*.cpp
    ${comm_server_root}/src/*.cpp)

set(BIN peephole_dev.elf)

include_directories(${nncase_sdk_root}/riscv64/rvvlib/include)
include_directories(${k230_sdk}/src/big/mpp/userapps/api/)
include_directories(${k230_sdk}/src/big/mpp/include)
include_directories(${k230_sdk}/src/big/mpp/include/comm)
include_directories(${k230_sdk}/src/big/mpp/userapps/sample/sample_vo)
include_directories(${PROJECT_SOURCE_DIR}/media/include)
include_directories(${PROJECT_SOURCE_DIR}/person_detect/include)
include_directories(${PROJECT_SOURCE_DIR}/app)
include_directories(${k230_sdk}/src/rt-smart/kernel/rt-thread/libcpu/risc-v/virt64)
include_directories(${k230_opencv}/include/opencv4/)
include_directories(${nncase_sdk_root}/riscv64)
include_directories(${nncase_sdk_root}/riscv64/nncase/include)
include_directories(${nncase_sdk_root}/riscv64/nncase/include/nncase/runtime)
include_directories(${ipcmsg_root}/include)
include_directories(${k230_sdk}/src/common/cdk/user/mapi/include)
include_directories(${k230_sdk}/src/common/cdk/user/mapi/include/api)
include_directories(${k230_sdk}/src/common/cdk/user/mapi/include/comm)
include_directories(${comm_server_root}/include)
include_directories((${k230_sdk}/src/common/cdk/user/component/datafifo/include))

link_directories(${nncase_sdk_root}/riscv64/rvvlib/)
link_directories(${k230_opencv}/lib ${k230_opencv}/lib/opencv4/3rdparty)
link_directories(${nncase_sdk_root}/riscv64/nncase/lib/)
link_directories(${k230_sdk}/src/big/mpp/userapps/lib)
link_directories(${k230_sdk}/src/common/cdk/user/mapi/mediaserver/lib)
link_directories(${ipcmsg_root}/slave/lib)
link_directories(${datafifo_root}/host/lib)
link_directories(${k230_sdk}/src/big/rt-smart/userapps/sdk/rt-thread/lib/risc-v/rv64)

add_executable(${BIN} ${APP_SOURCES})
target_link_libraries(${BIN} -Wl,--start-group rvv Nncase.Runtime.Native nncase.rt_modules.k230 functional_k230 sys vicap vb cam_device cam_engine
 hal oslayer ebase fpga isp_drv binder auto_ctrol common cam_caldb isi 3a buffer_management cameric_drv video_in virtual_hal start_engine cmd_buffer
 switch cameric_reg_drv t_database_c t_mxml_c t_json_c t_common_c vo connector sensor atomic dma -Wl,--end-group)

target_link_libraries(${BIN} opencv_imgcodecs opencv_imgproc opencv_core zlib libjpeg-turbo libopenjp2 libpng libtiff libwebp csi_cv)

target_link_libraries(${BIN} mapi ipcmsg datafifo venc aenc vdec adec ai ao vvi vdss dpu dma rtthread)

install(TARGETS ${BIN} DESTINATION out)
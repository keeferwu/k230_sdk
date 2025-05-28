set(src main.c model.c io.c process.c)
set(bin mbv2_onnx_int8.elf)

add_executable(${bin} ${src})
target_link_libraries(${bin} shl_c908 jpeg png z)

install(TARGETS ${bin} DESTINATION bin)
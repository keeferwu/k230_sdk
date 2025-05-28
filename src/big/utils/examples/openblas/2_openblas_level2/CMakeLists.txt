set(src openblas_level2.cpp)
set(bin 2_openblas_level2.elf)

add_executable(${bin} ${src})
target_link_libraries(${bin} sys)
target_link_libraries(${bin} openblas)

install(TARGETS ${bin} DESTINATION bin)
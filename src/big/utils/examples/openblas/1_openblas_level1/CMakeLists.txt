set(src openblas_level1.cpp)
set(bin 1_openblas_level1.elf)

add_executable(${bin} ${src})
target_link_libraries(${bin} sys)
target_link_libraries(${bin} openblas)

install(TARGETS ${bin} DESTINATION bin)
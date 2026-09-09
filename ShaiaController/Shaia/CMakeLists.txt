add_subdirectory(Src/${ARM_CONTROLLER_LIB})
add_subdirectory(Src/${HEAD_CONTROLLER_LIB})

add_library(${SHAIA_LIB} Src/Shaia.cpp)
target_include_directories(${SHAIA_LIB} PUBLIC ${CMAKE_SOURCE_DIR}/Shaia/Include)
target_link_libraries(${SHAIA_LIB} PUBLIC ${ARM_CONTROLLER_LIB} ${HEAD_CONTROLLER_LIB} ${OSC_LIB} ${SERIAL_LIB})

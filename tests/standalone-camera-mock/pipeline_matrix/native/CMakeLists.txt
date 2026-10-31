cmake_minimum_required(VERSION 3.20)
project(wrl_pipeline_matrix LANGUAGES C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
enable_testing()
add_subdirectory("../../capture_mock/native" "${CMAKE_CURRENT_BINARY_DIR}/capture")
set(MONADO_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." CACHE PATH "Enclosing Monado checkout")
get_filename_component(MONADO_SOURCE_DIR "${MONADO_SOURCE_DIR}" ABSOLUTE)
add_library(wrl_pipeline_matrix SHARED matrix.cpp component_log.c
  "${MONADO_SOURCE_DIR}/src/xrt/auxiliary/util/u_sink_converter.c"
  "${MONADO_SOURCE_DIR}/src/xrt/auxiliary/util/u_sink_force_genlock.c"
  "${MONADO_SOURCE_DIR}/src/xrt/auxiliary/util/u_frame.c"
  "${MONADO_SOURCE_DIR}/src/xrt/auxiliary/util/u_format.c")
target_include_directories(wrl_pipeline_matrix SYSTEM PRIVATE "${MONADO_SOURCE_DIR}/src/external/bcdec")
target_link_libraries(wrl_pipeline_matrix PRIVATE monado_queue)
target_compile_options(wrl_pipeline_matrix PRIVATE -Wall -Wextra)

add_executable(pipeline_matrix_smoke matrix_smoke.cpp)
target_link_libraries(pipeline_matrix_smoke PRIVATE wrl_pipeline_matrix)
add_test(NAME native_matrix_conversion_pairing COMMAND pipeline_matrix_smoke)

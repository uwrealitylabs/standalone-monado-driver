cmake_minimum_required(VERSION 3.20)
project(wrl_capture_mock LANGUAGES C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(MONADO_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." CACHE PATH "Enclosing Monado checkout")
get_filename_component(MONADO_SOURCE_DIR "${MONADO_SOURCE_DIR}" ABSOLUTE)
find_package(Threads REQUIRED)
foreach(config have build)
  configure_file("${MONADO_SOURCE_DIR}/src/xrt/include/xrt/xrt_config_${config}.h.cmake_in"
                 "${CMAKE_CURRENT_BINARY_DIR}/generated/xrt/xrt_config_${config}.h")
endforeach()
add_library(monado_queue STATIC
  "${MONADO_SOURCE_DIR}/src/xrt/auxiliary/util/u_sink_queue.c")
target_include_directories(monado_queue SYSTEM PUBLIC
  "${MONADO_SOURCE_DIR}/src/xrt/include"
  "${MONADO_SOURCE_DIR}/src/xrt/auxiliary"
  "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(monado_queue PUBLIC Threads::Threads)
add_executable(queue_baseline queue_baseline.cpp)
target_link_libraries(queue_baseline PRIVATE monado_queue)
enable_testing()
add_test(NAME unchanged_monado_queue COMMAND queue_baseline)

find_package(ZLIB REQUIRED)
add_library(wrl_mock_capture SHARED mock_capture.cpp)
target_link_libraries(wrl_mock_capture PRIVATE monado_queue ZLIB::ZLIB)
target_compile_options(wrl_mock_capture PRIVATE -Wall -Wextra -Wpedantic)

add_executable(capture_lifecycle capture_lifecycle.cpp)
target_link_libraries(capture_lifecycle PRIVATE wrl_mock_capture monado_queue)
add_test(NAME native_capture_lifecycle COMMAND capture_lifecycle)

add_executable(capture_group_handoff capture_group_handoff.cpp)
target_link_libraries(capture_group_handoff PRIVATE wrl_mock_capture monado_queue)
add_test(NAME native_image_bundle_handoff COMMAND capture_group_handoff)

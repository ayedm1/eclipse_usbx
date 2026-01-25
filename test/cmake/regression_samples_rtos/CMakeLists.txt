cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
cmake_policy(SET CMP0057 NEW)

project(regression_test LANGUAGES C)

get_filename_component(SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../regression_samples_rtos
                       ABSOLUTE)

add_compile_options(
  -m32
  -std=c99
  -ggdb
  -g3
  -gdwarf-2
  -fdiagnostics-color
  -Werror
  -DTX_INCLUDE_USER_DEFINE_FILE
  ${${CMAKE_BUILD_TYPE}})
add_link_options(-m32)

enable_testing()


execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/run.sh build_libs)
add_custom_target(build_libs ALL COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/run.sh
                                         build_libs)

add_dependencies(usbx build_libs)
target_include_directories(usbx PUBLIC ${CMAKE_BINARY_DIR}/../libs/inc)

add_library(threadx SHARED IMPORTED GLOBAL)
add_library("azrtos::threadx" ALIAS threadx)
set_target_properties(
  threadx PROPERTIES IMPORTED_LOCATION
                     ${CMAKE_BINARY_DIR}/../libs/threadx/libthreadx.so)

target_compile_options(
  usbx
  PRIVATE -Werror
          -Wall
          -Wextra
          -pedantic
          -fmessage-length=0
          -fsigned-char
          -ffunction-sections
          -fdata-sections
          -Wunused
          -Wuninitialized
          -Wmissing-declarations
          -Wconversion
          -Wpointer-arith
          -Wshadow
          -Wlogical-op
          -Waggregate-return
          -Wfloat-equal)

set(samples_cases
    ${SOURCE_DIR}/usbx_hid_mouse_demo_device_rtos_test.c
)

set(test_utility_files
    ${SOURCE_DIR}/usbxtestcontrol.c)


add_library(test_utility ${test_utility_files})
target_link_libraries(test_utility PUBLIC azrtos::usbx azrtos::threadx)
target_compile_definitions(test_utility PUBLIC CTEST)


foreach(test_case ${samples_cases})
  get_filename_component(test_name ${test_case} NAME_WE)
  add_executable(${test_name} ${test_case})
  target_link_libraries(${test_name} PRIVATE test_utility)
  add_test(${CMAKE_BUILD_TYPE}::${test_name} ${test_name})
endforeach()

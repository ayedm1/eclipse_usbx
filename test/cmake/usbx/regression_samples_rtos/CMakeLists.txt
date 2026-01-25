cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
cmake_policy(SET CMP0057 NEW)

project(regression_test LANGUAGES C)

get_filename_component(SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../regression_samples_rtos
                       ABSOLUTE)

set(ux_sample_device_hid_mouse_test_cases
    ${SOURCE_DIR}/usbx_hid_mouse_demo_device_rtos_test.c
)

set(test_utility_files
    ${SOURCE_DIR}/usbxtestcontrol.c)
    
add_library(test_utility ${test_utility_files})
target_link_libraries(test_utility PUBLIC azrtos::usbx azrtos::threadx)
target_compile_definitions(test_utility PUBLIC CTEST)


foreach(test_case ${test_cases})
  get_filename_component(test_name ${test_case} NAME_WE)
  add_executable(${test_name} ${test_case})
  target_link_libraries(${test_name} PRIVATE test_utility)
  add_test(${CMAKE_BUILD_TYPE}::${test_name} ${test_name})
endforeach()

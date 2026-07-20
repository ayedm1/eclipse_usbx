cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
cmake_policy(SET CMP0057 NEW)

project(regression_samples_rtos LANGUAGES C)

get_filename_component(REPO_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../.. ABSOLUTE)
set(SOURCE_DIR ${REPO_DIR}/test/regression_samples_rtos)

set(sample_test_cases
  ${SOURCE_DIR}/usbx_hid_mouse_demo_device_rtos_test.c
  ${SOURCE_DIR}/usbx_hid_keyboard_demo_device_rtos_test.c
  ${SOURCE_DIR}/usbx_hid_mouse_keyboard_demo_device_rtos_test.c
  ${SOURCE_DIR}/usbx_hid_consumer_demo_device_rtos_test.c
)

add_library(regression_samples_rtos_utility STATIC
  ${SOURCE_DIR}/usbxtestcontrol.c
  ${SOURCE_DIR}/ux_test.c
)
target_include_directories(regression_samples_rtos_utility PUBLIC ${SOURCE_DIR})
target_compile_definitions(regression_samples_rtos_utility PUBLIC CTEST)
target_link_libraries(regression_samples_rtos_utility PUBLIC azrtos::usbx azrtos::threadx azrtos::netxduo azrtos::filex)

foreach(test_case ${sample_test_cases})
  get_filename_component(test_name ${test_case} NAME_WE)
  add_executable(${test_name} ${test_case})
  target_include_directories(${test_name} PRIVATE ${SOURCE_DIR})
  target_link_libraries(${test_name} PRIVATE regression_samples_rtos_utility)
  add_test(${CMAKE_BUILD_TYPE}::${test_name} ${test_name})
endforeach()

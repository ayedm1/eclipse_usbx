cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
cmake_policy(SET CMP0057 NEW)

project(regression_test LANGUAGES C)

get_filename_component(SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../regression_samples_regression
                       ABSOLUTE)

set(ux_sample_device_hid_mouse_test_cases
    ${SOURCE_DIR}/usbx_hid_mouse_demo_device_rtos_test.c
)


set(test_utility_files
    ${SOURCE_DIR}/usbxtestcontrol.c
    ${SOURCE_DIR}/ux_test.c
    ${SOURCE_DIR}/ux_host_class_dummy.c
    ${SOURCE_DIR}/ux_host_class_dummy.h
    ${SOURCE_DIR}/ux_device_class_dummy.c
    ${SOURCE_DIR}/ux_device_class_dummy.h
    ${SOURCE_DIR}/ux_device_class_dummy_hub.c
    ${SOURCE_DIR}/ux_device_class_dummy_hub.h
    # ${SOURCE_DIR}/ux_device_class_dummy_printer.c
    # ${SOURCE_DIR}/ux_device_class_dummy_printer.h
    ${SOURCE_DIR}/ux_test_utility_sim_no_overriding.c
    ${SOURCE_DIR}/ux_test_race_condition_overrides.c
    ${SOURCE_DIR}/ux_test_dcd_sim_slave.c
    ${SOURCE_DIR}/ux_test_hcd_sim_host.c
    ${SOURCE_DIR}/ux_test_utility_sim.c
    ${SOURCE_DIR}/ux_test_standalone_references.c
    ${SOURCE_DIR}/usbx_ux_host_class_storage_fx_driver.c)
add_library(test_utility ${test_utility_files})
target_link_libraries(test_utility PUBLIC azrtos::usbx azrtos::threadx)
target_compile_definitions(test_utility PUBLIC CTEST)

if (CMAKE_BUILD_TYPE MATCHES "msrc_rtos_build")
  set(test_cases
    ${ux_msrc_test_cases}
    ${ux_msrc_test_cases_rtos}
  )
elseif (CMAKE_BUILD_TYPE MATCHES "msrc_standalone_build")
  set(test_cases
    ${ux_msrc_test_cases}
    ${ux_msrc_test_cases_standalone}
  )
elseif(NOT (CMAKE_BUILD_TYPE MATCHES "standalone.*"))
  if (CMAKE_BUILD_TYPE MATCHES "nofx_.*")
    set(test_cases
      ${ux_class_storage_test_cases}
    )
  elseif (CMAKE_BUILD_TYPE MATCHES "memory_management_.*")
    set(test_cases
      ${ux_class_memory_management_test_cases}
    )
  else()
    set(test_cases
      ${ux_basic_test_cases}
      ${ux_utility_test_cases}
      ${ux_utility_os_test_cases}
      ${ux_stack_test_cases}
      ${ux_dpump_test_cases}
      ${ux_class_hub_test_cases}
      ${ux_device_class_storage_tx_test_cases}
      ${ux_class_dfu_test_cases}
      ${ux_class_print_test_cases}
      ${ux_class_ccid_test_cases}
      )
    if(NOT (CMAKE_BUILD_TYPE MATCHES "optimized.*"))
      list(APPEND test_cases
        ${ux_stack_test_cases_hid}
        ${ux_stack_test_cases_cdc}
        ${ux_stack_cdc_acm_test_cases}
        ${ux_class_cdc_acm_test_cases}
        ${ux_class_audio_test_cases}
        ${ux_class_rndis_test_cases}
        ${ux_class_cdc_ecm_test_cases}
        ${ux_class_hid_test_cases}
        ${ux_class_video_test_cases}
        ${ux_class_storage_test_cases}
        ${ux_class_pima_test_cases}
        ${ux_class_gser_test_cases}
        ${ux_class_prolific_test_cases}
        ${ux_class_swar_test_cases}
        ${ux_msrc_test_cases}
        )
    endif()
  endif()
else()
  set(test_cases
      ${ux_basic_test_cases}
      ${ux_utility_test_cases}
      ${ux_dpump_test_cases}
    )
  if (CMAKE_BUILD_TYPE MATCHES "standalone_device.*")
    list(APPEND test_cases
      ${ux_utility_os_test_cases}
      ${ux_class_storage_device_standalone_test_cases}
      ${ux_class_cdc_acm_device_standalone_test_cases}
      ${ux_class_video_device_standalone_test_cases}
      ${ux_class_hid_device_standalone_test_cases}
      ${ux_class_dfu_test_cases}
      ${ux_msrc_test_cases}
      ${ux_class_printer_device_standalone_test_cases}
      ${ux_class_audio_device_standalone_test_cases}
      ${ux_class_ccid_test_cases}
      )
  elseif (CMAKE_BUILD_TYPE MATCHES "standalone_host.*")
    list(APPEND test_cases
      ${ux_utility_os_test_cases}
      ${ux_class_storage_host_standalone_test_cases}
      ${ux_class_cdc_acm_host_standalone_test_cases}
      ${ux_class_hid_host_standalone_test_cases}
      ${ux_class_print_test_cases}
      ${ux_class_hub_standalone_test_cases}
    )
  else ()
    list(APPEND test_cases
      ${ux_class_dfu_test_cases}
    )
  endif()
endif()

foreach(test_case ${test_cases})
  get_filename_component(test_name ${test_case} NAME_WE)
  add_executable(${test_name} ${test_case})
  target_link_libraries(${test_name} PRIVATE test_utility)
  add_test(${CMAKE_BUILD_TYPE}::${test_name} ${test_name})
endforeach()

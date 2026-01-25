cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
cmake_policy(SET CMP0054 NEW)
cmake_policy(SET CMP0057 NEW)
cmake_policy(SET CMP0077 NEW)

project(regression_demo_test LANGUAGES C)

get_filename_component(SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../regression_samples_rtos
                       ABSOLUTE)

set(BUILD_CONFIGURATIONS 
    default_build_coverage )

set(CMAKE_CONFIGURATION_TYPES
    ${BUILD_CONFIGURATIONS}
    CACHE STRING "list of supported configuration types" FORCE)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
                                             ${CMAKE_CONFIGURATION_TYPES})
list(GET CMAKE_CONFIGURATION_TYPES 0 BUILD_TYPE)
if((NOT CMAKE_BUILD_TYPE) OR (NOT ("${CMAKE_BUILD_TYPE}" IN_LIST
                                   CMAKE_CONFIGURATION_TYPES)))
  set(CMAKE_BUILD_TYPE
      "${BUILD_TYPE}"
      CACHE STRING "Build Type of the project" FORCE)
endif()

message(STATUS "Build for usbx")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}.")

set(default_build_coverage "")

# Control if USBX is static or shared
if($ENV{USBX_STATIC})
  message(STATUS "Building STATIC usbx")
  set(BUILD_SHARED_LIBS OFF)
else()
  message(STATUS "Building usbx BUILD_SHARED_LIBS: ${BUILD_SHARED_LIBS}")
endif()
if(NOT BUILD_SHARED_LIBS)
  if(CMAKE_BUILD_TYPE MATCHES ".*_coverage")
    add_link_options(-fprofile-arcs)
    add_link_options(-lgcov)
  endif()
endif()

# Control if it's for 64 bit or 32 bit
if($ENV{ENABLE_64})
  message(STATUS "Building for 64bit")
else()
  add_compile_options(-m32)
  add_link_options(-m32)
  message(STATUS "Building for 32bit")
endif()

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

set(THREADX_ARCH "linux")
set(THREADX_TOOLCHAIN "gnu")


add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../../ usbx)


# Build ThreadX library once
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

add_library(filex SHARED IMPORTED GLOBAL)
add_library("azrtos::filex" ALIAS filex)
set_target_properties(
  filex PROPERTIES IMPORTED_LOCATION
                   ${CMAKE_BINARY_DIR}/../libs/filex/libfilex.so)
               
add_library(netxduo SHARED IMPORTED GLOBAL)
add_library("azrtos::netxduo" ALIAS netxduo)
set_target_properties(
  netxduo PROPERTIES IMPORTED_LOCATION
                     ${CMAKE_BINARY_DIR}/../libs/netxduo/libnetxduo.so)

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

# Remove includes and files not needed from usbx build
set(UX_STANDALONE_HOST_EXCLUDES
  # ux_host_class_asix
  # ux_host_class_audio
  # ux_host_class_cdc_acm
  # ux_host_class_cdc_ecm
  # ux_host_class_gser
  # ux_host_class_hid
  # ux_host_class_hub
  # ux_host_class_pima
  # ux_host_class_printer
  # ux_host_class_prolific
  # ux_host_class_storage
  # ux_host_class_swar
  # ux_host_class_video
  # ux_hcd_ehci
  # ux_hcd_ohci
)
set(UX_STANDALONE_DEVICE_EXCLUDES
  # ux_device_class_audio
  # ux_device_class_cdc_acm
  # ux_device_class_cdc_ecm
  # ux_device_class_dfu
  # ux_device_class_hid
  # ux_device_class_pima
  # ux_device_class_rndis
  # ux_device_class_ccid
  # ux_device_class_printer
  # ux_device_class_video
)
set(UX_STANDALONE_UTILITY_EXCLUDES
  # ux_utility_event
  # ux_utility_delay
  # ux_utility_mutex
  # ux_utility_semaphore
  # ux_utility_thread
  # ux_utility_timer
)

set(UX_STANDALONE_PICTBRIDGE_EXCLUDES
  # ux_pictbridge
)
set(UX_STANDALONE_NX_EXCLUDES
  # ux_network_driver
)
set(UX_STANDALONE_FX_EXCLUDES
  # ux_host_class_storage_driver_entry
)

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

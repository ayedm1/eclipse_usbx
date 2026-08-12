target_sources(${PROJECT_NAME} PRIVATE
    # {{BEGIN_TARGET_SOURCES}}

    # {{END_TARGET_SOURCES}}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../test/externals/threadx/common/inc
)

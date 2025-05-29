set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/artifacts/example")

# Thunderscope Test App
add_executable(thunderscope_test thunderscope_test.cpp)

target_link_libraries(thunderscope_test tslitex_static)

if(WIN32)
    target_link_libraries(thunderscope_test setupapi)
endif()

# Thunderscope FW Update CLI
add_executable(thunderscope_fw thunderscope_fw.cpp)

target_link_libraries(thunderscope_fw tslitex_static)

if(WIN32)
    target_link_libraries(thunderscope_fw setupapi)
endif()

# Thunderscope Cal CLI
add_executable(thunderscope_cal thunderscope_cal.cpp)

target_link_libraries(thunderscope_cal tslitex_static)

if(WIN32)
    target_link_libraries(thunderscope_cal setupapi)
endif()

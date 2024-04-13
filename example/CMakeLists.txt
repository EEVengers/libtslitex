set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/artifacts/example")

add_executable(thunderscope_test thunderscope_test.cpp)

target_link_libraries(thunderscope_test litepcie)

if(WIN32)
    target_link_libraries(thunderscope_test setupapi)
endif()
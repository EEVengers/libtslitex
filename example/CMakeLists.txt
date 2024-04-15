set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/artifacts/example")

add_executable(litepcie_test litepcie_test.cpp)
add_executable(thunderscope_test thunderscope_test.cpp)

target_link_libraries(litepcie_test litepcie)
target_link_libraries(thunderscope_test litepcie)

if(WIN32)
    target_link_libraries(litepcie_test setupapi)
    target_link_libraries(thunderscope_test setupapi)
endif()
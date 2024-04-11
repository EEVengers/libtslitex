
include_directories(${PROJECT_SOURCE_DIR}/litepcie/public_h)
include_directories(${PROJECT_SOURCE_DIR}/litepcie/include)


add_executable(litepcie_test litepcie_test.cpp)

set_property(TARGET litepcie_test PROPERTY CXX_STANDARD 20)
set_property(TARGET litepcie_test PROPERTY C_STANDARD 17)

target_link_libraries(litepcie_test litepcie)
target_link_libraries(litepcie_test setupapi)


add_executable(thunderscope_test thunderscope_test.cpp)

set_property(TARGET thunderscope_test PROPERTY CXX_STANDARD 20)
set_property(TARGET thunderscope_test PROPERTY C_STANDARD 17)


target_link_libraries(thunderscope_test litepcie)
target_link_libraries(thunderscope_test setupapi)

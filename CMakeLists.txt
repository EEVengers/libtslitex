cmake_minimum_required(VERSION 3.20)


project(libthunderscope)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/artifacts)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/artifacts)

add_definitions(-DUNICODE -D_UNICODE)

add_subdirectory(litepcie)

add_subdirectory(example)


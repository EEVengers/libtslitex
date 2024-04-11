cmake_minimum_required(VERSION 3.20)

project(liblitepcie)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-DUNICODE -D_UNICODE)

add_subdirectory(src)



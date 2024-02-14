cmake_minimum_required(VERSION 3.20)

enable_language(C)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON) # error if compiler doesn't support c17
set(CMAKE_C_EXTENSIONS OFF) # use c17 instead of gnu17


enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # error if compiler doesn't support c++17
set(CMAKE_CXX_EXTENSIONS OFF) # use c++17 instead of gnu++17

project(libthunderscope)

add_definitions(-DUNICODE -D_UNICODE)

add_subdirectory(litepcie)

add_subdirectory(example)


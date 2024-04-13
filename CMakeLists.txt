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

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/artifacts/thunderscope")

add_definitions(-DUNICODE -D_UNICODE)

add_subdirectory(litepcie)

add_subdirectory(example)

set(TS_SOURCES
    src/i2c.c
    src/gpio.c
    src/spi.c
    src/thunderscope.c
    )

set(TS_HEADERS
    src/i2c.h
    src/gpio.h
    src/spi.h
    )

set(TS_LIB_HEADERS 
    include/thunderscope.h
    include/channel.h
    include/adc.h
    include/afe.h
    )

add_library(thunderscope SHARED ${TS_SOURCES}
            ${TS_LIB_HEADERS}
            )

set_target_properties(thunderscope PROPERTIES POSITION_INDEPENDENT_CODE 1)


target_link_libraries(thunderscope litepcie)
target_include_directories(thunderscope PUBLIC include)

file(COPY ${TS_LIB_HEADERS} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/include)
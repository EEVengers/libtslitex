cmake_minimum_required(VERSION 3.20)

enable_language(C)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON) # error if compiler doesn't support c17
set(CMAKE_C_EXTENSIONS ON)


enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # error if compiler doesn't support c++17
set(CMAKE_CXX_EXTENSIONS ON)

project(libtslitex)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/artifacts/libtslitex")

add_definitions(-DUNICODE -D_UNICODE)

add_definitions(-DEN_LOGGING)
add_definitions(-DEN_LOGGING_DEBUG)

add_subdirectory(litepcie)


set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(TS_SOURCES
    src/i2c.c
    src/gpio.c
    src/spi.c
    src/afe.c
    src/adc.c
    src/hmcad15xx.c
    src/lmh6518.c
    src/mcp4728.c
    src/mcp443x.c
    src/mcp_clkgen.c
    src/mcp_zl3026x.c
    src/platform.c
    src/samples.c
    src/spiflash.c
    src/ts_channel.c
    src/thunderscope.c
    )

set(TS_HEADERS
    src/i2c.h
    src/gpio.h
    src/spi.h
    src/adc.h
    src/afe.h
    src/mcp_clkgen.h
    src/mcp_zl3026x.h
    src/platform.h
    src/lmh6518.h
    src/mcp4728.h
    src/mcp443x.h
    src/hmcad15xx.h
    src/samples.h
    src/spiflash.h
    src/ts_channel.h
    src/util.h
    )

set(TS_LIB_HEADERS 
    include/thunderscope.h
    include/ts_common.h
    )

####
# LIBTSLITEX SHARED LIBRARY
####
add_library(tslitex SHARED ${TS_SOURCES}
            ${TS_LIB_HEADERS}
            )

target_link_libraries(tslitex litepcie)
target_include_directories(tslitex PUBLIC include)

####
# LIBTSLITEX STATIC LIBRARY
####    
add_library(tslitex_static STATIC ${TS_SOURCES}
            ${TS_LIB_HEADERS}
            )

#target_compile_definitions(tslitex_static tslitex_static_STATIC)
set_target_properties(tslitex_static PROPERTIES POSITION_INDEPENDENT_CODE 1)


target_link_libraries(tslitex_static litepcie)
target_include_directories(tslitex_static PUBLIC include)

file(COPY ${TS_LIB_HEADERS} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/include)

add_subdirectory(example)
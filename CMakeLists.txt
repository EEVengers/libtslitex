cmake_minimum_required(VERSION 3.22...4.1)

project(libtslitex)

# Set Version
include(version.cmake)

enable_language(C)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON) # error if compiler doesn't support c17
set(CMAKE_C_EXTENSIONS ON)

if(MSVC)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /std:c17 /experimental:c11atomics")
endif(MSVC)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # error if compiler doesn't support c++17
set(CMAKE_CXX_EXTENSIONS ON)

################################
# Project Dependencies
################################
include(FetchContent)

# json-c Library
FetchContent_Declare(
    json_c
    GIT_REPOSITORY  https://github.com/json-c/json-c
    GIT_TAG         2372e9518e6ba95b48d37ec162bc7d93b297b52f
    FIND_PACKAGE_ARGS NAMES json-c
)

set(BUILD_SHARED_LIBS OFF)
FetchContent_MakeAvailable(json_c)

# zlib Library
FetchContent_Declare(
    zlib
    GIT_REPOSITORY  https://github.com/madler/zlib.git
    GIT_TAG         v1.3.1
    FIND_PACKAGE_ARGS NAMES zlib
)

set(ZLIB_BUILD_TESTING OFF)
set(ZLIB_BUILD_SHARED  OFF)
set(ZLIB_INSTALL       OFF)
FetchContent_MakeAvailable(zlib)



set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/artifacts/libtslitex")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/artifacts/libtslitex")

option(ENABLE_FACTORY_PROVISIONING "Include Factory Provisioning Functions" OFF)

if(ENABLE_FACTORY_PROVISIONING)
    add_compile_definitions(FACTORY_PROVISIONING_API)
endif()

add_compile_definitions(UNICODE _UNICODE)

add_compile_definitions(EN_LOGGING)

add_compile_definitions("$<$<CONFIG:Debug>:EN_LOGGING_DEBUG>")

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
    src/ts_data.c
    src/ts_fw_manager.c
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
    src/ts_data.h
    src/ts_fw_manager.h
    src/util.h
    )

set(TS_LIB_HEADERS 
    include/thunderscope.h
    include/ts_common.h
    include/ts_calibration.h
    )

####
# LIBTSLITEX SHARED LIBRARY
####
add_library(tslitex SHARED ${TS_SOURCES}
            ${TS_LIB_HEADERS}
            )

set_target_properties(tslitex PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
set_target_properties(tslitex PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/$<0:>
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/$<0:>
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/$<0:>)

target_link_libraries(tslitex PUBLIC litepcie)
target_link_libraries(tslitex PRIVATE json-c-static zlibstatic)

if(APPLE)
    target_link_libraries(tslitex PRIVATE "-framework IOKit")
endif()

target_include_directories(tslitex PUBLIC include)

####
# LIBTSLITEX STATIC LIBRARY
####    
add_library(tslitex_static STATIC ${TS_SOURCES}
            ${TS_LIB_HEADERS}
            )

set_target_properties(tslitex_static PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
set_target_properties(tslitex_static PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/$<0:>
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/$<0:>
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/$<0:>)

set_target_properties(tslitex_static PROPERTIES POSITION_INDEPENDENT_CODE 1)

target_link_libraries(tslitex_static PUBLIC litepcie)
target_link_libraries(tslitex_static PRIVATE json-c-static zlibstatic)
target_include_directories(tslitex_static PUBLIC include)

file(COPY ${TS_LIB_HEADERS} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/include)

####
# EXAMPLE APPLICATIONS
####
add_subdirectory(example)

####
# BINDINGS
####
add_subdirectory(bindings/python)


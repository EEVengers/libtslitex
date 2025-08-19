cmake_minimum_required(VERSION 3.20)

find_package(
    Python
    COMPONENTS Interpreter Development.Module
)

if(Python_FOUND)

add_custom_command(
    OUTPUT tslitex/tslitex.c
    COMMAND Python::Interpreter -m cython
    "${CMAKE_CURRENT_SOURCE_DIR}/tslitex.pyx" --output-file tslitex/tslitex.c
    DEPENDS tslitex.pyx tslitex.pxd
    VERBATIM
    )
    
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/lib/include/thunderscope.h
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "$<TARGET_FILE_DIR:tslitex_static>"
    "${CMAKE_CURRENT_BINARY_DIR}/lib"
    DEPENDS tslitex_static
)

add_custom_target(pydeps
    DEPENDS tslitex/tslitex.c
    ${CMAKE_CURRENT_BINARY_DIR}/lib/include/thunderscope.h
    tslitex_static
)

set(PY_BIND_LIBS tslitex_static litepcie)
if(WIN32)
    list(APPEND PY_BIND_LIBS setupapi)
endif()
list(JOIN PY_BIND_LIBS "\", \"" PY_EXT_LIBS)

set(PY_WHL_VERSION_STRING "${${PROJECT_NAME}_VERSION}")
if (NOT ${${PROJECT_NAME}_VERSION_AHEAD} EQUAL "0")
    set(PY_WHL_VERSION_STRING "${PY_WHL_VERSION_STRING}.dev${${PROJECT_NAME}_VERSION_AHEAD}")
endif()

configure_file(pyproject.toml.in pyproject.toml @ONLY)
configure_file(__init__.py.in tslitex/__init__.py COPYONLY)

add_custom_target(PyBindings
        COMMAND Python::Interpreter -m pipx run build --wheel .
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        COMMENT "Building python whl"
        DEPENDS pydeps tslitex_static
        )

if (NOT DEFINED PY_PLATFORM)
    set(PY_PLATFORM "auto")
endif()
if (NOT DEFINED PY_ARCH)
    set(PY_ARCH "auto64")
endif()

add_custom_target(PyWheels
        COMMAND Python::Interpreter -m cibuildwheel --platform ${PY_PLATFORM} --archs ${PY_ARCH} --config-file pyproject.toml
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        COMMENT "Building all Python wheels"
        DEPENDS pydeps tslitex_static
        )
else()
message(STATUS "Python bindings not available: Python not found")
endif()

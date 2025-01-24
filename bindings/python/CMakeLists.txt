cmake_minimum_required(VERSION 3.20)

find_package(
    Python
    COMPONENTS Interpreter Development.Module
    REQUIRED
)

add_custom_command(
    OUTPUT tslitex.c
    COMMAND Python::Interpreter -m cython
    "${CMAKE_CURRENT_SOURCE_DIR}/tslitex.pyx" --output-file tslitex.c
    DEPENDS tslitex.pyx tslitex.pxd tscalibration.pxd
    VERBATIM
    )
    
python_add_library(pytslitex MODULE WITH_SOABI EXCLUDE_FROM_ALL tslitex.c)

target_link_libraries(pytslitex PRIVATE tslitex_static)
target_include_directories(pytslitex PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_custom_target(PyBindings
        COMMAND "${PYTHON_EXECUTABLE}" -m pip wheel .
        EXCLUDE_FROM_ALL
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        COMMENT "Building python whl"
        )

add_dependencies(PyBindings pytslitex)

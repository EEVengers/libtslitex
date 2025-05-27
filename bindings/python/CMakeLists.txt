cmake_minimum_required(VERSION 3.20)

find_package(
    Python
    COMPONENTS Interpreter Development.Module
    REQUIRED
)

add_custom_command(
    OUTPUT tslitex/tslitex.c
    COMMAND Python::Interpreter -m cython
    "${CMAKE_CURRENT_SOURCE_DIR}/tslitex.pyx" --output-file tslitex/tslitex.c
    DEPENDS tslitex.pyx tslitex.pxd
    VERBATIM
    )

set(PY_BIND_LIBS tslitex_static litepcie)
if(WIN32)
    list(APPEND PY_BIND_LIBS setupapi)
endif()
list(JOIN PY_BIND_LIBS "\", \"" PY_EXT_LIBS)

configure_file(pyproject.toml.in pyproject.toml @ONLY)
configure_file(__init__.py.in tslitex/__init__.py COPYONLY)

add_custom_target(PyBindings
        COMMAND Python::Interpreter -m pipx run build --wheel .
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        COMMENT "Building python whl"
        DEPENDS tslitex/tslitex.c
        )


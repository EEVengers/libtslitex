set(CMAKE_C_EXTENSIONS ON) # gnu17 expected for linux lib

set(litepcie_INTERFACE_HEADERS
    public_h/csr.h
    public_h/config.h
    public_h/litepcie.h
    public_h/soc.h
    )

if(WIN32)
set(litepcie_PLATFORM_HEADERS
    public_h/litepcie_win.h
    )
else()
set(litepcie_PLATFORM_HEADERS
    public_h/litepcie_linux.h
    )
endif()

set(litepcie_HEADERS
    include/liblitepcie.h
    include/litepcie_dma.h
    include/litepcie_flash.h
    include/litepcie_helpers.h
    )

set(litepcie_SOURCES
    src/litepcie_dma.c
    src/litepcie_helpers.c
    )

add_library(litepcie STATIC ${litepcie_SOURCES}
            ${litepcie_HEADERS}
            ${litepcie_INTERFACE_HEADERS}
            ${litepcie_PLATFORM_HEADERS}
            )

set_target_properties(litepcie PROPERTIES POSITION_INDEPENDENT_CODE 1)

target_include_directories(litepcie PUBLIC public_h include)

if(WIN32)
    target_link_libraries(litepcie setupapi)
endif()
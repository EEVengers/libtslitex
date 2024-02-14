set(litepcie_INTERFACE_HEADERS
    public_h/csr.h
    public_h/litepcie_dmadrv.h
    public_h/litepcie_public.h
    public_h/soc.h
    )

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

add_library(litepcie STATIC ${litepcie_SOURCES} ${litepcie_HEADERS} ${litepcie_INTERFACE_HEADERS})
target_include_directories(litepcie PUBLIC public_h include)
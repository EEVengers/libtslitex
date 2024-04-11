
include_directories(../public_h)

include_directories(../include)

add_library(litepcie STATIC litepcie_dma.c litepcie_helpers.c)

set_property(TARGET litepcie PROPERTY C_STANDARD 17)



#ifndef _LITEPCIE_H_
#define _LITEPCIE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "csr.h"
#include "soc.h"
#include "config.h"

#ifdef _WIN32
#include "litepcie_win.h"
#elif __linux__
#include "litepcie_linux.h"
#elif __APPLE__
#include "litepcie_mac.h"
#else
#error "UNSUPPORTED PLATFORM"
#endif

#ifdef __cplusplus
}
#endif
#endif
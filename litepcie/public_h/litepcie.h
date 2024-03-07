

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
#else
#include "litepcie_linux.h"
#endif

#ifdef __cplusplus
}
#endif
#endif
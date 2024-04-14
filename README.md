# Thunderscope Library

This repository provides a library for the LiteX Thunderscope gateware.  It is built upon the litepcie driver and includes the register definition files from the Thunderscope gateware build.  The Thunderscope litepcie driver should be built separately.

![image](doc/thunderscope.png)

## Requirements

For the Windows build, requires:
- VS 2022
- Cmake

## Build

```cmd
> cd build
> cmake ..
> cmake --build .
```
Output binaries are in the `build/artifacts` folder.

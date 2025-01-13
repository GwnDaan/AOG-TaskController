# AOG-TaskController 🚜

This is an experimental project to control sections of an ISOBUS implement using AgOpenGPS. It is based on the [AgIsoStack++](https://github.com/Open-Agriculture/AgIsoStack-plus-plus) library.

## How to build

### Windows

You'll need cmake:

```bash
cmake -S. -B build -Wno-dev
cmake --build build
```

If the NTCAN SDK is not found:
Download <https://esd.eu/en/products/can-sdk> and during install select all dependencies:
(restart cmd.exe before running build)

## How to run?

```bash
build\Debug\AOGTaskController.exe
```

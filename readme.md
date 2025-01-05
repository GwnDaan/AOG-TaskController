# AOG-TaskController 🚜

## How to build?

### Windows

You'll need cmake:

```bash
cmake -S. -B build -Wno-dev
cmake --build build
```

If NTCAN SDK is not found:
Download <https://esd.eu/en/products/can-sdk> and during install select all dependencies:
(restart cmd.exe before running build)

## How to run?

```bash
build\Debug\AOGTaskController.exe
```

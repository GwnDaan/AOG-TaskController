# AOGTaskController 🚜


## How to build?


### Windows:
You'll need cmake: 

cmake -S. -B build -Wno-dev
cmake --build build

If NTCAN SDK is not found modify:
Download and during install select dependendencies from here: https://esd.eu/en/products/can-sdk
(restart cmd.exe before running build)

## How to run?

start "" /B build\Debug\AOGTaskController.exe
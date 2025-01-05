cmake_minimum_required(VERSION 3.16)
project("AOGTaskController")

set(BUILD_EXAMPLES ON)

if(WIN32)
  set(CAN_DRIVER "WindowsPCANBasic")
  list(APPEND CAN_DRIVER "TouCAN")
  list(APPEND CAN_DRIVER "WindowsInnoMakerUSB2CAN")
  list(APPEND CAN_DRIVER "SYS_TEC")
elseif(APPLE)
  set(CAN_DRIVER "MacCANPCAN")
else()
  set(CAN_DRIVER "SocketCAN")
endif()
list(APPEND CAN_DRIVER "NTCAN")

include(FetchContent)

set(BOOST_INCLUDE_LIBRARIES asio)
set(BOOST_ENABLE_CMAKE ON)
FetchContent_Declare(
  Boost
  URL https://github.com/boostorg/boost/releases/download/boost-1.86.0/boost-1.86.0-cmake.7z
      USES_TERMINAL_DOWNLOAD
      TRUE
  DOWNLOAD_NO_EXTRACT FALSE)
FetchContent_MakeAvailable(Boost)

FetchContent_Declare(
  isobus
  GIT_REPOSITORY https://github.com/Open-Agriculture/AgIsoStack-plus-plus.git
  GIT_TAG ca7ffb744a853a97759ae1d07137765f59902915)
FetchContent_MakeAvailable(isobus)

find_package(Threads REQUIRED)

file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
add_executable(AOGTaskController ${SRC_FILES})

target_compile_features(AOGTaskController PUBLIC cxx_std_11)
set_target_properties(AOGTaskController PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(AOGTaskController
                           PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(
  AOGTaskController PRIVATE isobus::Isobus isobus::HardwareIntegration
                            Threads::Threads isobus::Utility Boost::asio)

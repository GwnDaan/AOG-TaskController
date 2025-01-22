cmake_minimum_required(VERSION 3.16)
project("AOG-TaskController")

set(BUILD_EXAMPLES OFF)
set(BUILD_TESTING OFF)

set(CAN_DRIVER "WindowsPCANBasic")
# list(APPEND CAN_DRIVER "TouCAN")

# list(APPEND CAN_DRIVER "WindowsInnoMakerUSB2CAN")

# list(APPEND CAN_DRIVER "SYS_TEC")

# list(APPEND CAN_DRIVER "NTCAN")

include(FetchContent)

set(BOOST_INCLUDE_LIBRARIES asio)
set(BOOST_ENABLE_CMAKE ON)
FetchContent_Declare(
  Boost
  URL https://github.com/boostorg/boost/releases/download/boost-1.86.0/boost-1.86.0-cmake.7z
  DOWNLOAD_NO_EXTRACT FALSE
  USES_TERMINAL_DOWNLOAD TRUE DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(Boost)

FetchContent_Declare(
  isobus
  GIT_REPOSITORY https://github.com/Open-Agriculture/AgIsoStack-plus-plus.git
  GIT_TAG 58686f401282d34b8f15a704a82798222083ca3f
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(isobus)

FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(json)

find_package(Threads REQUIRED)

file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
add_executable("AOG-TaskController" ${SRC_FILES} resources/AppIcon.rc)

target_compile_features("AOG-TaskController" PUBLIC cxx_std_20)
set_target_properties("AOG-TaskController" PROPERTIES CXX_EXTENSIONS OFF)
set_target_properties("AOG-TaskController" PROPERTIES WIN32_EXECUTABLE TRUE)

target_include_directories("AOG-TaskController"
                           PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(
  "AOG-TaskController"
  PRIVATE isobus::Isobus isobus::HardwareIntegration Threads::Threads
          isobus::Utility Boost::asio nlohmann_json::nlohmann_json)

install(TARGETS "AOG-TaskController" RUNTIME DESTINATION bin
                                             COMPONENT applications)

add_custom_command(
  TARGET "AOG-TaskController"
  POST_BUILD
  COMMENT "Copying icon.ico to the binary directory"
  COMMAND
    "${CMAKE_COMMAND}" -E copy_if_different
    ${CMAKE_CURRENT_LIST_DIR}/resources/icon.ico
    "$<TARGET_FILE_DIR:AOG-TaskController>"
  VERBATIM)

install(
  FILES ${CMAKE_CURRENT_LIST_DIR}/resources/icon.ico
  DESTINATION bin
  COMPONENT applications)

add_custom_command(
  TARGET AOG-TaskController
  POST_BUILD
  COMMENT "Copying PCANBasic.dll to the build directory"
  COMMAND
    "${CMAKE_COMMAND}" -E copy_if_different
    ${CMAKE_CURRENT_LIST_DIR}/lib/PCANBasic.dll
    "$<TARGET_FILE_DIR:AOG-TaskController>"
  VERBATIM)

install(
  FILES ${CMAKE_CURRENT_LIST_DIR}/lib/PCANBasic.dll
  DESTINATION bin
  COMPONENT applications)

add_custom_command(
  TARGET AOG-TaskController
  POST_BUILD
  COMMENT "Copying InnoMakerUsb2CanLib.dll to the build directory"
  COMMAND
    "${CMAKE_COMMAND}" -E copy_if_different
    ${CMAKE_CURRENT_LIST_DIR}/lib/InnoMakerUsb2CanLib.dll
    "$<TARGET_FILE_DIR:AOG-TaskController>"
  VERBATIM)

install(
  FILES ${CMAKE_CURRENT_LIST_DIR}/lib/InnoMakerUsb2CanLib.dll
  DESTINATION bin
  COMPONENT applications)

set(CPACK_PACKAGE_NAME "AOG-TaskController")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "ISOBUS-TC for AOG")
set(CPACK_PACKAGE_VENDOR "")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "AOG-TaskController")
set(CPACK_COMPONENTS_ALL applications)
set(CPACK_COMPONENT_APPLICATIONS_DISPLAY_NAME "AOG-TaskController")
set(CPACK_COMPONENT_APPLICATIONS_DESCRIPTION
    "An ISOBUS task controller for the AgOpenGPS project")
set(CPACK_PACKAGE_CONTACT "github.com/GwnDaan/AOG-TaskController")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_LIST_DIR}/LICENSE")
set(CPACK_PACKAGE_EXECUTABLES "AOG-TaskController" "AOG-TaskController")
set(CPACK_ALL_INSTALL_TYPES Full)
set(CPACK_COMPONENT_LIBRARIES_INSTALL_TYPES Full)
set(CPACK_COMPONENT_APPLICATIONS_INSTALL_TYPES Full)
set(CPACK_NSIS_INSTALLED_ICON_NAME "bin/AOG-TaskController.exe")
set(CPACK_NSIS_CONTACT ${CPACK_PACKAGE_CONTACT})
set(CPACK_NSIS_MUI_ICON ${CMAKE_CURRENT_LIST_DIR}/resources/icon.ico)
set(CPACK_NSIS_MUI_UNICON ${CPACK_NSIS_MUI_ICON})
set(CPACK_NSIS_DISPLAY_NAME "AOG-TaskController")
set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS
    "Delete $SMPROGRAMS/$STARTMENU_FOLDER/AOG-TaskController.lnk")
set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
set(CMAKE_INSTALL_SYSTEM_RUNTIME_COMPONENT applications)
include(InstallRequiredSystemLibraries)

# Must be after the last CPACK macros
include(CPack)

cmake_minimum_required(VERSION 3.16)
project("AOG-TaskController")
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 5)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION
    "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}"
)

find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --tags --exact-match HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_TAG
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(GIT_TAG MATCHES "^v([0-9]+)\\.([0-9]+)\\.([0-9]+)(-beta\\.[0-9]+)?$")
    set(PROJECT_VERSION_MAJOR ${CMAKE_MATCH_1})
    set(PROJECT_VERSION_MINOR ${CMAKE_MATCH_2})
    set(PROJECT_VERSION_PATCH ${CMAKE_MATCH_3})
    if(CMAKE_MATCH_4)
      set(PROJECT_VERSION
          "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}${CMAKE_MATCH_4}"
      )
    else()
      set(PROJECT_VERSION
          "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}"
      )
    endif()
    message(
      STATUS "Building from git tag: ${GIT_TAG}, version: ${PROJECT_VERSION}")
  endif()
endif()

set(BUILD_EXAMPLES OFF)
set(BUILD_TESTING OFF)

if(NOT DEFINED CAN_DRIVER)
  if(WIN32)
    set(CAN_DRIVER "WindowsPCANBasic")
    list(APPEND CAN_DRIVER "WindowsInnoMakerUSB2CAN")
    list(APPEND CAN_DRIVER "TouCAN")
    list(APPEND CAN_DRIVER "SYS_TEC")
  elseif(APPLE)
    set(CAN_DRIVER "MacCANPCAN")
  else()
    set(CAN_DRIVER "SocketCAN")
  endif()
endif()

include(FetchContent)

set(BOOST_INCLUDE_LIBRARIES asio align)
set(BOOST_ENABLE_CMAKE ON)
FetchContent_Declare(
  Boost
  URL https://github.com/boostorg/boost/releases/download/boost-1.86.0/boost-1.86.0-cmake.7z
  DOWNLOAD_NO_EXTRACT FALSE
  USES_TERMINAL_DOWNLOAD TRUE DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(Boost)

FetchContent_Declare(
  isobus
  GIT_REPOSITORY https://github.com/gunicsba/AgIsoStack-plus-plus.git
  GIT_TAG e79699e9f58b332d2c1146ebeb5e25c8908c0e35
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(isobus)

FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(json)

include(FetchContent)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
FetchContent_Declare(
  git_version
  GIT_REPOSITORY https://github.com/andrew-hardin/cmake-git-version-tracking.git
  GIT_TAG 904dbda1336ba4b9a1415a68d5f203f576b696bb)
FetchContent_MakeAvailable(git_version)
unset(CMAKE_POLICY_VERSION_MINIMUM)

find_package(Threads REQUIRED)

set(AOG_TC_IOP_SOURCE "${CMAKE_CURRENT_LIST_DIR}/resources/AOG_TC.iop")
set(AOG_TC_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(AOG_TC_IOP_HEADER "${AOG_TC_GENERATED_INCLUDE_DIR}/AOG_TC_iop_data.hpp")
file(MAKE_DIRECTORY "${AOG_TC_GENERATED_INCLUDE_DIR}")
file(READ "${AOG_TC_IOP_SOURCE}" AOG_TC_IOP_HEX HEX)
string(REGEX REPLACE "([0-9a-fA-F][0-9a-fA-F])" "0x\\1," AOG_TC_IOP_BYTES
                     "${AOG_TC_IOP_HEX}")
file(
  WRITE "${AOG_TC_IOP_HEADER}"
  "#pragma once\n\n#include <cstdint>\n\ninline constexpr std::uint8_t AOG_TC_IOP_DATA[] = {${AOG_TC_IOP_BYTES}};\n"
)
set_property(
  DIRECTORY
  APPEND
  PROPERTY CMAKE_CONFIGURE_DEPENDS "${AOG_TC_IOP_SOURCE}")

file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
if(WIN32)
  add_executable(${PROJECT_NAME} ${SRC_FILES} resources/AppIcon.rc)
else()
  add_executable(${PROJECT_NAME} ${SRC_FILES})
endif()

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)
if(WIN32)
  set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
endif()

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include ${CMAKE_CURRENT_LIST_DIR}/resources
          ${AOG_TC_GENERATED_INCLUDE_DIR})

target_compile_definitions(
  ${PROJECT_NAME} PUBLIC PROJECT_VERSION="${PROJECT_VERSION}"
                         PROJECT_NAME="${PROJECT_NAME}")

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE isobus::Isobus
          isobus::HardwareIntegration
          Threads::Threads
          isobus::Utility
          Boost::asio
          nlohmann_json::nlohmann_json
          cmake_git_version_tracking)

if(WIN32)
  # For crash_handler.cpp's minidump writer (MiniDumpWriteDump).
  target_link_libraries(${PROJECT_NAME} PRIVATE dbghelp)
endif()

if(MSVC)
  # Keep Release optimizations but still emit a PDB, so a crash dump (see
  # crash_handler.hpp) can actually be symbolicated afterwards — without this,
  # .dmp files are close to useless.
  target_compile_options(${PROJECT_NAME} PRIVATE /Zi)
  target_link_options(${PROJECT_NAME} PRIVATE /DEBUG /OPT:REF /OPT:ICF)
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin COMPONENT applications)

option(AOG_TC_VALIDATE_IOP "Build and register the object pool validator test"
       ON)
if(AOG_TC_VALIDATE_IOP)
  # BUILD_TESTING stays off so FetchContent dependencies do not add their suites
  enable_testing()
  add_executable(iop_validator
                 ${CMAKE_CURRENT_LIST_DIR}/tools/iop_validator.cpp)
  target_compile_features(iop_validator PRIVATE cxx_std_20)
  set_target_properties(iop_validator PROPERTIES CXX_EXTENSIONS OFF)
  target_link_libraries(iop_validator PRIVATE isobus::Isobus isobus::Utility)
  add_test(NAME object_pool_attributes_in_range COMMAND iop_validator
                                                        "${AOG_TC_IOP_SOURCE}")

  # A deliberately broken pool, so the suite asserts more than "today's pool
  # passes". One case per message, since a shared exit code stays green as long
  # as any one check still fires.
  set(AOG_TC_MALFORMED_IOP
      ${CMAKE_CURRENT_LIST_DIR}/tools/testdata/malformed_pool.iop)
  add_test(NAME malformed_pool_coerced_attribute
           COMMAND iop_validator "${AOG_TC_MALFORMED_IOP}")
  set_tests_properties(
    malformed_pool_coerced_attribute
    PROPERTIES PASS_REGULAR_EXPRESSION "format byte has undefined value")
  add_test(NAME malformed_pool_dangling_reference
           COMMAND iop_validator "${AOG_TC_MALFORMED_IOP}")
  set_tests_properties(
    malformed_pool_dangling_reference
    PROPERTIES PASS_REGULAR_EXPRESSION "active mask references object 4660")
  add_test(NAME malformed_pool_value_out_of_range
           COMMAND iop_validator "${AOG_TC_MALFORMED_IOP}")
  set_tests_properties(
    malformed_pool_value_out_of_range PROPERTIES PASS_REGULAR_EXPRESSION
                                                 "InputBoolean value 24")

  # PASS_REGULAR_EXPRESSION makes CTest ignore the exit code, which is the part
  # a CI gate actually reads.
  add_test(NAME malformed_pool_exit_status COMMAND iop_validator
                                                   "${AOG_TC_MALFORMED_IOP}")
  set_tests_properties(malformed_pool_exit_status PROPERTIES WILL_FAIL TRUE)

  # Tractor Facilities (PGN 65033) encode/decode unit tests
  add_executable(
    test_tractor_facilities
    ${CMAKE_CURRENT_LIST_DIR}/tools/test_tractor_facilities.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/tractor_facilities.cpp)
  target_compile_features(test_tractor_facilities PRIVATE cxx_std_20)
  set_target_properties(test_tractor_facilities PROPERTIES CXX_EXTENSIONS OFF)
  target_include_directories(test_tractor_facilities
                             PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
  target_link_libraries(
    test_tractor_facilities PRIVATE isobus::Isobus isobus::HardwareIntegration
                                    isobus::Utility)
  add_test(NAME tractor_facilities_encode_decode
           COMMAND test_tractor_facilities)
endif()

if(WIN32)
  add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMENT "Copying icon.ico to the binary directory"
    COMMAND
      "${CMAKE_COMMAND}" -E copy_if_different
      ${CMAKE_CURRENT_LIST_DIR}/resources/icon.ico
      "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    VERBATIM)

  install(
    FILES ${CMAKE_CURRENT_LIST_DIR}/resources/icon.ico
    DESTINATION bin
    COMPONENT applications)

  add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMENT "Copying PCANBasic.dll to the build directory"
    COMMAND
      "${CMAKE_COMMAND}" -E copy_if_different
      ${CMAKE_CURRENT_LIST_DIR}/lib/PCANBasic.dll
      "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    VERBATIM)

  install(
    FILES ${CMAKE_CURRENT_LIST_DIR}/lib/PCANBasic.dll
    DESTINATION bin
    COMPONENT applications)

  add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMENT "Copying InnoMakerUsb2CanLib.dll to the build directory"
    COMMAND
      "${CMAKE_COMMAND}" -E copy_if_different
      ${CMAKE_CURRENT_LIST_DIR}/lib/InnoMakerUsb2CanLib.dll
      "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    VERBATIM)

  install(
    FILES ${CMAKE_CURRENT_LIST_DIR}/lib/InnoMakerUsb2CanLib.dll
    DESTINATION bin
    COMPONENT applications)

  add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMENT "Copying canal.dll to the build directory"
    COMMAND
      "${CMAKE_COMMAND}" -E copy_if_different
      ${CMAKE_CURRENT_LIST_DIR}/lib/canal.dll
      "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    VERBATIM)

  install(
    FILES ${CMAKE_CURRENT_LIST_DIR}/lib/canal.dll
    DESTINATION bin
    COMPONENT applications)

  add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMENT "Copying Usbcan64.dll to the build directory"
    COMMAND
      "${CMAKE_COMMAND}" -E copy_if_different
      ${CMAKE_CURRENT_LIST_DIR}/lib/Usbcan64.dll
      "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    VERBATIM)

  install(
    FILES ${CMAKE_CURRENT_LIST_DIR}/lib/Usbcan64.dll
    DESTINATION bin
    COMPONENT applications)
endif()

set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "ISOBUS-TC for AOG")
set(CPACK_PACKAGE_VENDOR "")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_INSTALL_DIRECTORY ${PROJECT_NAME})
set(CPACK_COMPONENTS_ALL applications)
set(CPACK_COMPONENT_APPLICATIONS_DISPLAY_NAME ${PROJECT_NAME})
set(CPACK_COMPONENT_APPLICATIONS_DESCRIPTION
    "An ISOBUS task controller for the AgOpenGPS project")
set(CPACK_PACKAGE_CONTACT "github.com/GwnDaan/${PROJECT_NAME}")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_LIST_DIR}/LICENSE")
set(CPACK_PACKAGE_EXECUTABLES ${PROJECT_NAME} ${PROJECT_NAME})
set(CPACK_ALL_INSTALL_TYPES Full)
set(CPACK_COMPONENT_LIBRARIES_INSTALL_TYPES Full)
set(CPACK_COMPONENT_APPLICATIONS_INSTALL_TYPES Full)
set(CPACK_NSIS_INSTALLED_ICON_NAME "bin/${PROJECT_NAME}.exe")
set(CPACK_NSIS_CONTACT ${CPACK_PACKAGE_CONTACT})
set(CPACK_NSIS_MUI_ICON ${CMAKE_CURRENT_LIST_DIR}/resources/icon.ico)
set(CPACK_NSIS_MUI_UNICON ${CPACK_NSIS_MUI_ICON})
set(CPACK_NSIS_DISPLAY_NAME ${PROJECT_NAME})
set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS
    "Delete $SMPROGRAMS/$STARTMENU_FOLDER/${PROJECT_NAME}.lnk")
set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
set(CMAKE_INSTALL_SYSTEM_RUNTIME_COMPONENT applications)
include(InstallRequiredSystemLibraries)

# Must be after the last CPACK macros
include(CPack)

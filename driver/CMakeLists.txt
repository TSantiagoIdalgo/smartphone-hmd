cmake_minimum_required(VERSION 3.20)
project(smartphone_hmd LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Static runtime so the DLL does not depend on the Visual C++ redistributable.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

if(WIN32)
  set(PLATFORM_DIR win64)
elseif(APPLE)
  set(PLATFORM_DIR osx32)
else()
  set(PLATFORM_DIR linux64)
endif()

# SteamVR expects: <driver>/driver.vrdrivermanifest, <driver>/resources, <driver>/bin/<platform>/driver_<name>.dll
set(DRIVER_OUTPUT_DIR ${CMAKE_BINARY_DIR}/smartphone_hmd)

add_library(driver_smartphone_hmd SHARED
  src/driver_main.cpp
  src/device_provider.cpp
  src/hmd_device.cpp
)
target_include_directories(driver_smartphone_hmd SYSTEM PRIVATE third_party/openvr)
# "$<0:>" stops multi-config generators (Visual Studio) from adding a Release/ subfolder.
set_target_properties(driver_smartphone_hmd PROPERTIES
  PREFIX ""
  RUNTIME_OUTPUT_DIRECTORY ${DRIVER_OUTPUT_DIR}/bin/${PLATFORM_DIR}$<0:>
  LIBRARY_OUTPUT_DIRECTORY ${DRIVER_OUTPUT_DIR}/bin/${PLATFORM_DIR}$<0:>
)
if(MSVC)
  target_compile_options(driver_smartphone_hmd PRIVATE /W4 /utf-8)
else()
  target_compile_options(driver_smartphone_hmd PRIVATE -Wall -Wextra)
endif()

add_custom_command(TARGET driver_smartphone_hmd POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/smartphone_hmd ${DRIVER_OUTPUT_DIR}
)

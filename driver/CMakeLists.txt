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

# Tracking logic without SteamVR dependencies, shared by the driver and the tests.
add_library(tracking STATIC
  src/protocol.cpp
  src/head_tracker.cpp
  src/video_protocol.cpp
  src/color_convert.cpp
  src/vsync_clock.cpp
)
target_include_directories(tracking PUBLIC src)
find_package(Threads REQUIRED)
target_link_libraries(tracking PUBLIC Threads::Threads)
set_target_properties(tracking PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(driver_smartphone_hmd SHARED
  src/driver_main.cpp
  src/device_provider.cpp
  src/hmd_device.cpp
  src/udp_socket.cpp
)
target_include_directories(driver_smartphone_hmd SYSTEM PRIVATE third_party/openvr)
target_link_libraries(driver_smartphone_hmd PRIVATE tracking)
if(WIN32)
  # Video streaming: D3D11 frame capture and Media Foundation H.264 encoding.
  target_sources(driver_smartphone_hmd PRIVATE
    src/frame_capture.cpp
    src/h264_encoder.cpp
    src/video_streamer.cpp
    src/virtual_display.cpp
  )
  target_link_libraries(driver_smartphone_hmd PRIVATE ws2_32 d3d11 dxgi mfplat mfuuid strmiids ole32 oleaut32 winmm)
endif()
# "$<0:>" stops multi-config generators (Visual Studio) from adding a Release/ subfolder.
set_target_properties(driver_smartphone_hmd PROPERTIES
  PREFIX ""
  RUNTIME_OUTPUT_DIRECTORY ${DRIVER_OUTPUT_DIR}/bin/${PLATFORM_DIR}$<0:>
  LIBRARY_OUTPUT_DIRECTORY ${DRIVER_OUTPUT_DIR}/bin/${PLATFORM_DIR}$<0:>
)
foreach(target tracking driver_smartphone_hmd)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /utf-8)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra)
  endif()
endforeach()

add_custom_command(TARGET driver_smartphone_hmd POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/smartphone_hmd ${DRIVER_OUTPUT_DIR}
)

enable_testing()
add_executable(driver_tests tests/tests.cpp)
target_link_libraries(driver_tests PRIVATE tracking)
add_test(NAME driver_tests COMMAND driver_tests)

cmake_minimum_required(VERSION 3.22)

include(FetchContent)

project(vsgCs
    VERSION 0.2.0
    DESCRIPTION "VSG library for rendering 3D Tiles and Cesium ion assets"
    LANGUAGES CXX C
)
set(VSGCS_SOVERSION 0)
SET(VSGCS_RELEASE_CANDIDATE 0)

# We have some custom .cmake scripts not in the official distribution.
SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${vsgCs_SOURCE_DIR}/CMakeModules")

include("vsgCsMacros")

# set the use of C++17 globally as all examples require it
set(CMAKE_CXX_STANDARD 17)

if (MSVC)
  # template madness
  add_compile_options(/bigobj)
  # windows.h madness
  add_compile_definitions(NOGDI)
endif()

option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)

option(RUN_CLANG_TIDY "run the clang-tidy checker" OFF)

# Find Vulkan and the VSG
if (VULKAN_SDK)
    set(ENV{VULKAN_SDK} ${VULKAN_SDK})
endif()

message("Fetching and make-available vsg ...")

set(VSG_MIN_VERSION 1.1.0)
add_subdirectory(third_party/vsg)  # vsg-dev/VulkanSceneGraph master | tag/1.1.0

# find_package(vsg) includes vsgMacros
if (vsg_SOURCE_DIR)
  message("Fetching and make-available vsgMacros ...")
  SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${vsg_SOURCE_DIR}/cmake")
  include("vsgMacros")
endif()

message("Fetching and make-available vsg finished")

find_package(vsgXchange)

message("Fetching and make-available vsgImGui ...")

set(VSGIMGUI_MIN_VERSION 0.1.0)
add_subdirectory(third_party/vsgImGui)   # vsg-dev/vsgImGui.git tag/v0.1.0 

message("Fetching and make-available vsgImGui finished")

message("Fetching and make-available cesium-native ...")

option(CESIUM_SPDLOG_HEADER_ONLY "Whether to use the header-only version of spdlog." OFF)
option(GSL_INSTALL "Generate and install GSL target" ON)

add_subdirectory(third_party/cesium-native)  # timoore/cesium-native 32309b09de1a9e0860e647fa4900c1491d74c85b

message("Fetching and make-available cesium-native finished")

find_package(PROJ REQUIRED)

if (RUN_CLANG_TIDY)
  find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
  if(CLANG_TIDY_EXE)
    set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=google-*,performance-*,portability-*,readability-*,modernize-*,-google-build-using-namespace,-readability-identifier-length,-readability-uppercase-literal-suffix,-google-readability-namespace-comments,-readability-named-parameter,-readability-implicit-bool-conversion,-readability-magic-numbers,-modernize-use-trailing-return-type)
  endif()
endif()

vsgCs_setup_build_vars()
# This does include(GNUInstallDirs), which sets up all the correct install directories.
vsg_setup_dir_vars()

set(GENERATED_HEADERS_DIR "${PROJECT_BINARY_DIR}/include")

set(VSGCS_DATA_DIR "${CMAKE_INSTALL_DATADIR}/vsgCs")
set(VSGCS_FULL_DATA_DIR "${CMAKE_INSTALL_FULL_DATADIR}/vsgCs")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/vsgCs/Config.h.in"
  "${GENERATED_HEADERS_DIR}/vsgCs/Config.h")

vsg_add_target_clobber()
vsg_add_target_docs(
    FILES
        ${CMAKE_SOURCE_DIR}/include
)
vsg_add_target_uninstall()

vsg_add_option_maintainer(
    PREFIX ${PROJECT_NAME}
    RCLEVEL ${VSGCS_RELEASE_CANDIDATE}
)

# Make the headers visible to everything. This is not OSG / VSG style,
# but we prefer that include files live next to their source files.
include_directories(${vsgCs_SOURCE_DIR}/src ${GENERATED_HEADERS_DIR})

if(WIN32)
  set(CMAKE_INSTALL_UCRT_LIBRARIES YES)

  include(InstallRequiredSystemLibraries)
endif()

# source directory for library and applications

add_subdirectory(src)
add_subdirectory(data)

vsg_add_feature_summary()

option(BUILD_TRACY "build with tracy profiling and server" OFF)

if (BUILD_TRACY)
  option ( TRACY_ENABLE "" ON)
  option ( TRACY_ON_DEMAND "" ON)
  add_subdirectory(3rdparty/tracy)
  set_target_properties(TracyClient PROPERTIES COMPILE_WARNING_AS_ERROR OFF)
endif()


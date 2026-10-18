# Third-party dependencies, fetched at configure time at pinned versions and checked against a
# SHA-256 of the archive. To change a version, change the URL and the hash together.
#
# Eigen and yaml-cpp stay out of core/format, which is compiled into the Unreal plugin;
# scripts/check_core_invariants.sh enforces this.

include(FetchContent)

# Eigen: header-only. SOURCE_SUBDIR points at a directory that does not exist, so
# FetchContent_MakeAvailable downloads and unpacks the sources but does not configure Eigen's own
# build (its tests, docs, and many targets). We expose the headers through one interface target.
FetchContent_Declare(
  eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR swarm_no_cmake_here)
FetchContent_MakeAvailable(eigen)

add_library(swarm_eigen INTERFACE)
add_library(swarm::eigen ALIAS swarm_eigen)
# SYSTEM, so the project's strict warnings and -Werror do not apply to Eigen's headers.
target_include_directories(swarm_eigen SYSTEM INTERFACE "${eigen_SOURCE_DIR}")
target_compile_definitions(swarm_eigen INTERFACE EIGEN_MPL2_ONLY)

if(SWARM_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    URL_HASH SHA256=1f357c27ca988c3f7c6b4bf68a9395005ac6761f034046e9dde0896e3aba00e4
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    FIND_PACKAGE_ARGS NAMES GTest)

  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

# yaml-cpp: used only by the scenario library (core/scenario), which takes document structure from
# it and parses every scalar itself. Built as a static library without its tests, tools, or install
# rules. CMAKE_POLICY_VERSION_MINIMUM lets CMake 4 configure a project that declares an old
# minimum version.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
set(YAML_CPP_FORMAT_SOURCE OFF CACHE BOOL "" FORCE)
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  yaml-cpp
  URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz
  URL_HASH SHA256=fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(yaml-cpp)

# Consumers see yaml-cpp's headers as system headers, so the project's strict warnings and -Werror
# do not apply to them.
set_target_properties(yaml-cpp PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "")
add_library(swarm_yamlcpp INTERFACE)
add_library(swarm::yamlcpp ALIAS swarm_yamlcpp)
target_link_libraries(swarm_yamlcpp INTERFACE yaml-cpp)
target_include_directories(swarm_yamlcpp SYSTEM INTERFACE "${yaml-cpp_SOURCE_DIR}/include")

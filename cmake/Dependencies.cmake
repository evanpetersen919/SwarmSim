# Third-party dependencies, fetched at configure time with pinned versions.
# Only what the current code needs is fetched.
#
# Add when the code first uses them (pin these versions):
#   Eigen     3.4.0   https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
#   yaml-cpp  0.8.0   https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz
# Eigen and yaml-cpp stay out of core/format, which is compiled into the Unreal plugin.

include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  FIND_PACKAGE_ARGS NAMES GTest)

set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

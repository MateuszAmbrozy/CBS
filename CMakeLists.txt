cmake_minimum_required(VERSION 3.16)
project(CBS VERSION 0.1.0 LANGUAGES CXX)

add_library(cbs STATIC
    lib/ConstraintTree.cpp
    lib/HighLevel.cpp
    lib/LowLevel.cpp
    lib/ConstraintTree.h
    lib/HighLevel.h
    lib/LowLevel.h
    lib/util.h
)
target_compile_features(cbs PUBLIC cxx_std_14)
set_target_properties(cbs PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(cbs PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lib")

option(CBS_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
if(CBS_ENABLE_SANITIZERS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND NOT WIN32)
        target_compile_options(cbs PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(cbs PUBLIC -fsanitize=address,undefined)
    else()
        message(FATAL_ERROR "CBS_ENABLE_SANITIZERS requires GCC or Clang on a Unix platform.")
    endif()
endif()

add_executable(cbs_demo main.cpp)
target_link_libraries(cbs_demo PRIVATE cbs)

include(CTest)
if(BUILD_TESTING)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/googletest/CMakeLists.txt")
        set(gtest_force_shared_crt ON CACHE BOOL "Use the shared runtime" FORCE)
        add_subdirectory(googletest EXCLUDE_FROM_ALL)
    else()
        find_package(GTest REQUIRED)
    endif()
    add_subdirectory(gtest)
endif()

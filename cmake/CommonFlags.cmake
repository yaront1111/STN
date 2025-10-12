# Common compiler flags and options for AION project

# Compiler warnings
if(MSVC)
  add_compile_options(/W4 /permissive- /WX-)
else()
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wcast-align
    -Wcast-qual
    -Wconversion
    -Wformat=2
    -Wnull-dereference
    -Wold-style-cast
    -Woverloaded-virtual
    -Wshadow
    -Wsign-conversion
    -Wunused
    -Wno-unknown-pragmas
    $<$<CXX_COMPILER_ID:GNU>:-Wlogical-op>
    $<$<CXX_COMPILER_ID:GNU>:-Wuseless-cast>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-gnu-zero-variadic-macro-arguments>
  )
endif()

# Build type specific flags
if(MSVC)
  set(CMAKE_CXX_FLAGS_DEBUG "/Od /Zi /DDEBUG /DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG")
  set(CMAKE_CXX_FLAGS_RELEASE "/O2 /DNDEBUG")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/O2 /Zi /DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG")
  set(CMAKE_CXX_FLAGS_MINSIZEREL "/O1 /DNDEBUG")
else()
  set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -DDEBUG -DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG")
  set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG")
endif()

# Architecture optimizations for Release builds
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
  if(COMPILER_SUPPORTS_MARCH_NATIVE)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
  endif()

  CHECK_CXX_COMPILER_FLAG("-mtune=native" COMPILER_SUPPORTS_MTUNE_NATIVE)
  if(COMPILER_SUPPORTS_MTUNE_NATIVE)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mtune=native")
  endif()
endif()

# Sanitizers for Debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  option(ENABLE_ASAN "Enable Address Sanitizer" OFF)
  option(ENABLE_UBSAN "Enable Undefined Behavior Sanitizer" OFF)
  option(ENABLE_TSAN "Enable Thread Sanitizer" OFF)

  if(ENABLE_ASAN)
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
  endif()

  if(ENABLE_UBSAN)
    add_compile_options(-fsanitize=undefined)
    add_link_options(-fsanitize=undefined)
  endif()

  if(ENABLE_TSAN)
    add_compile_options(-fsanitize=thread)
    add_link_options(-fsanitize=thread)
  endif()
endif()

# Set RPATH for installed binaries
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Helper function to add AION library
function(add_aion_library name)
  add_library(${name} ${ARGN})
  target_compile_features(${name} PUBLIC cxx_std_20)
  target_include_directories(${name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
  )
  set_target_properties(${name} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
  )
endfunction()

# Helper function to add AION executable
function(add_aion_executable name)
  add_executable(${name} ${ARGN})
  target_compile_features(${name} PRIVATE cxx_std_20)
endfunction()
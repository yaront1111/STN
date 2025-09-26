# Unit tests for AION

# Find GTest
find_package(GTest QUIET)

if(GTest_FOUND)
  enable_testing()

  # Add test subdirectories as they become available
  add_subdirectory(unit)
  # add_subdirectory(integration)
  # add_subdirectory(scenario)

  message(STATUS "GTest found - tests will be built")
else()
  message(WARNING "GTest not found - tests will be skipped")
endif()
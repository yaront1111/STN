add_library(aion_core SHARED
  # Fusion
  fusion/state.cpp
  fusion/sr_ukf.cpp
  fusion/process_model.cpp
  fusion/imu_preintegration.cpp

  # Utils
  utils/logging.cpp
  utils/config.cpp
  utils/time_utils.cpp
  utils/binary_logger.cpp
  utils/binary_reader.cpp
)

target_include_directories(aion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_BINARY_DIR}/msgs  # For generated protobuf headers
)

target_link_libraries(aion_core PUBLIC
  Eigen3::Eigen
  ${YAML_CPP_LIBRARIES}
  spdlog::spdlog
  Threads::Threads
  aion_msgs
  absl::check
  absl::log
  absl::log_internal_check_op
)

target_include_directories(aion_core SYSTEM PUBLIC
  ${YAML_CPP_INCLUDE_DIRS}
)

target_link_directories(aion_core PUBLIC
  ${YAML_CPP_LIBRARY_DIRS}
)

# Optional dependencies
if(GTSAM_FOUND)
  target_link_libraries(aion_core PUBLIC gtsam)
  target_compile_definitions(aion_core PUBLIC USE_GTSAM)
endif()

if(Sophus_FOUND)
  target_link_libraries(aion_core PUBLIC Sophus::Sophus)
  target_compile_definitions(aion_core PUBLIC USE_SOPHUS)
endif()

# Install targets
install(TARGETS aion_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY fusion/ DESTINATION include/core/fusion FILES_MATCHING PATTERN "*.h")
install(DIRECTORY utils/ DESTINATION include/core/utils FILES_MATCHING PATTERN "*.h")
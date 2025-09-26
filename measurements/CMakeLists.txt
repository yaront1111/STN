add_library(measurements STATIC
    trn_measurement.cpp
    trn_ukf_measurement.cpp
    trn_manager.cpp
)

target_include_directories(measurements PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(measurements PUBLIC
    aion_core
    terrain
    Eigen3::Eigen
)

target_compile_features(measurements PUBLIC cxx_std_20)
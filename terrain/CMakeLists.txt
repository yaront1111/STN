add_library(terrain STATIC
    srtm_reader.cpp
    terrain_service.cpp
)

target_include_directories(terrain PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(terrain PUBLIC
    aion_core
    Eigen3::Eigen
    Threads::Threads
)

target_compile_features(terrain PUBLIC cxx_std_20)
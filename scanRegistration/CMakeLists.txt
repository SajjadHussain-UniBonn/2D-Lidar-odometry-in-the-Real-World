target_compile_features(dataloader PUBLIC cxx_std_17)
add_library(scanRegistration STATIC scanRegistration.cpp)
target_link_libraries(scanRegistration PUBLIC Open3D::Open3D)

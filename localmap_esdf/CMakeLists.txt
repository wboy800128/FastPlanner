
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -std=c++17")
include_directories( 
    include 
)

add_library( 
        local_map STATIC
        src/sdf_map.cpp 
    )

# Find Eigen3 (vcpkg provides this via toolchain)
find_package(Eigen3 REQUIRED NO_MODULE)
target_link_libraries(local_map PUBLIC Eigen3::Eigen)



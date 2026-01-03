set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -std=c++17")



include_directories(../localmap_esdf/include)
include_directories(/matplotlib-cpp)
include_directories(./inlcude/grad_replanner)
include_directories(trajopt_bspline/include/quadrotor_msgs)

## remove hardcoded python2.7 path and use FindPython3

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories( 
    include 
    ${Eigen3_INCLUDE_DIRS} 
)

add_library(        trajopt  STATIC
                    #src/main.cpp
                    #src/replanner_node
                    src/PlannerInterface.cpp 
                    src/bspline_replanner.cpp
                    src/grad_band_optimizer.cpp
                    src/non_uniform_bspline.cpp 
                    src/bezier_base.cpp
            )

target_link_libraries(
                        trajopt
                        local_map
                        ${NLOPT_LIBRARIES}
                        Python3::Python
                     )

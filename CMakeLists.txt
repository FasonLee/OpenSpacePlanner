cmake_minimum_required(VERSION 2.8)
project(OpenSpacePlanner)
set(CMAKE_CXX_STANDARD 17)
message("cmake OpenSpacePlanner...")

set(CMAKE_BUILD_TYPE Debug)

# set(project_link_directories
#     Ipopt_x86pc/lib/
# #    ${CMAKE_FIND_ROOT_PATH}/usr/lib
# )

# link_directories (
#     ${project_link_directories}
# )

include_directories(
    Ipopt_x86pc/include/coin-or
    include/open_space_planner
    src
)

link_directories(
    ${PROJECT_SOURCE_DIR}/Ipopt_x86pc/lib
    # ${PROJECT_SOURCE_DIR}/../../Ipopt_x86pc/lib
)

add_library(open_space_planner SHARED
    src/Grid.cpp
    src/Point.cpp
    src/Pose.cpp
    src/GridRect.cpp
    src/Rect.cpp
    src/Map.cpp
    src/GeometryFunc.cpp
    src/AStarNode.cpp
    src/AStarPlanner.cpp
    src/HybridAStarNode.cpp
    src/ReedsSheppPath.cpp
    src/HybridAStarPlanner.cpp
    src/PathOptIpoptNlp.cpp
    src/PathSmoother.cpp
    src/IaOptimizer.cpp
)

target_link_libraries(open_space_planner
    # pthread
    ipopt
)

# if (NOT ALL)
#     set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/${BUILD_PROJECT}/${BUILD_PLATFORM})
# endif()

find_package(Boost REQUIRED COMPONENTS filesystem)

# if ((TEST) AND (NOT ALL))
find_package(OpenCV REQUIRED)
    set(OpenCV_LIBS
        opencv_core
        opencv_video
        opencv_imgproc
        opencv_calib3d
        opencv_videoio
        opencv_highgui
        opencv_imgcodecs)
    # FILE(GLOB_RECURSE OpenCV_LIBS "../apa_sdk/lib/libopencv*")
    
    include_directories(
        # ../apa_sdk/include 
        /usr/local/include
    )
    # set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin/${BUILD_PROJECT}/${BUILD_PLATFORM})
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)
    add_executable(test_open_space_planner
        test/TestHybridAStarPlanner.cpp
        # test/TestAStarPlanner.cpp
    )
    target_link_libraries(test_open_space_planner
        open_space_planner
        # ../apa_sdk/lib/${OpenCV_LIBS}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        # pthread
        # ipopt
    )
# endif()

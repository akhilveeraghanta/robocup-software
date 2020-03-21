cmake_minimum_required(VERSION 3.5)
project(robocup_common)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})

find_package(Protobuf REQUIRED)

add_subdirectory(protobuf)

# Headers and sources
file(GLOB ROBOCUP_COMMON_HDRS include/robocup_common/*.hpp)

set(ROBOCUP_COMMON_SRC
    "src/Field_Dimensions.cpp"
    "src/Arc.cpp"
    "src/Circle.cpp"
    "src/Line.cpp"
    "src/Rect.cpp"
    "src/TransformMatrix.cpp"
    "src/CompositeShape.cpp"
    "src/Point.cpp"
    "src/Polygon.cpp"
    "src/Segment.cpp"
    "src/multicast.cpp"
    "src/Utils.cpp"
)

add_library(robocup_common SHARED ${ROBOCUP_COMMON_SRC} ${ROBOCUP_COMMON_HDRS})
target_link_libraries(robocup_common proto_messages)
target_include_directories(robocup_common PUBLIC include/robocup_common/)

# Include output directory. Necessary for protobuf
target_include_directories(robocup_common PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

find_package(Qt5Widgets REQUIRED)
qt5_use_modules(robocup_common Core Network Widgets)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    enable_testing()
    find_package(GTest REQUIRED)

    set(ROBOCUP_COMMON_TEST_SRC
        "src/LineTest.cpp"
        "src/PointTest.cpp"
        "src/RectTest.cpp"
        "src/SegmentTest.cpp"
        "src/CompositeShapeTest.cpp"
        "src/ArcTest.cpp"
        "src/TransformMatrixTest.cpp"
        "src/PoseTest.cpp"
    )

    ament_add_gtest(test-common ${ROBOCUP_COMMON_TEST_SRC})
    if (TARGET test-common)
        target_link_libraries(test-common robocup_common ${GTEST_MAIN_LIBRARY})
    endif()
endif()

install(TARGETS ${PROJECT_NAME} proto_messages
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

ament_export_libraries(${PROJECT_NAME})
ament_export_libraries(proto_messages)
ament_export_include_directories(include/robocup_common/)
ament_package()

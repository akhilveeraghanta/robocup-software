cmake_minimum_required(VERSION 3.8)
project(soccer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON) # Don't fall back to older versions
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robocup_common REQUIRED)
find_package(robocup_fshare REQUIRED)
find_package(rrt REQUIRED)
find_package(grSim REQUIRED)
find_package(libusb-1.0 REQUIRED)

# ------------------------------------------------------------------------------

# Because we use ninja, we have to explicitly turn on color output for the compiler
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics -Werror=return-stack-address")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-local-addr")
endif()

# Use compiler optimization if we are making a release build.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=native")

# Build in debug mode
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Og")

# Turn some compiler warnings into errors
# TODO determine if we want these flags in the release build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Werror=return-type -Werror=delete-non-virtual-dtor")

# Packages
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)
add_definitions(-DQT_NO_KEYWORDS)

find_package(PythonInterp 3.2 REQUIRED)
find_package(PythonLibs 3.2 REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})

find_package(libusb-1.0 REQUIRED)

# All sources for the robocup library used for the python module, soccer, and log log viewer
# Note that main.cpp and LogViewer.cpp aren't present in this list because they're executable-specific
set(ROBOCUP_LIB_SRC
    "BatteryProfile.cpp"
    "Configuration.cpp"
    "gameplay/GameplayModule.cpp"
    "gameplay/robocup-py.cpp"
    "joystick/Joystick.cpp"
    "joystick/GamepadController.cpp"
    "joystick/GamepadJoystick.cpp"
    "joystick/SpaceNavJoystick.cpp"
    "KickEvaluator.cpp"
    "Logger.cpp"
    "motion/MotionControl.cpp"
    "motion/MotionControlNode.cpp"
    "motion/TrapezoidalMotion.cpp"
    "NewRefereeModule.cpp"
    "optimization/GradientAscent1D.cpp"
    "optimization/ParallelGradientAscent1D.cpp"
    "optimization/NelderMead2D.cpp"
    "optimization/PythonFunctionWrapper.cpp"
    "planning/CompositePath.cpp"
    "planning/DynamicObstacle.cpp"
    "planning/DirectTargetPathPlanner.cpp"
    "planning/TuningPathPlanner.cpp"
    "planning/EscapeObstaclesPathPlanner.cpp"
    "planning/InterpolatedPath.cpp"
    "planning/IndependentMultiRobotPathPlanner.cpp"
    "planning/InterceptPlanner.cpp"
    "planning/MotionConstraints.cpp"
    "planning/RotationConstraints.cpp"
    "planning/Path.cpp"
    "planning/RRTPlanner.cpp"
    "planning/RRTUtil.cpp"
    "planning/PivotPathPlanner.cpp"
    "planning/SettlePathPlanner.cpp"
    "planning/CollectPathPlanner.cpp"
    "planning/LineKickPlanner.cpp"
    "planning/SingleRobotPathPlanner.cpp"
    "planning/TargetVelPathPlanner.cpp"
    "planning/TrapezoidalPath.cpp"
    "Processor.cpp"
    "radio/NetworkRadio.cpp"
    "radio/PacketConvert.cpp"
    "radio/RadioNode.cpp"
    "radio/SimRadio.cpp"
    "Robot.cpp"
    "RobotConfig.cpp"
    "SystemState.cpp"
    "DebugDrawer.cpp"
    "ui/BatteryWidget.cpp"
    "ui/FieldView.cpp"
    "ui/MainWindow.cpp"
    "ui/ProtobufTree.cpp"
    "ui/RefereeTab.cpp"
    "ui/RobotStatusWidget.cpp"
    "ui/RobotWidget.cpp"
    "ui/SimFieldView.cpp"
    "ui/StripChart.cpp"
    "ui/StyleSheetManager.cpp"
    "GrSimCommunicator.cpp"
    "VisionReceiver.cpp"
    "vision/ball/BallBounce.cpp"
    "vision/ball/CameraBall.cpp"
    "vision/ball/KalmanBall.cpp"
    "vision/ball/WorldBall.cpp"
    "vision/camera/Camera.cpp"
    "vision/camera/World.cpp"
    "vision/filter/KalmanFilter.cpp"
    "vision/filter/KalmanFilter2D.cpp"
    "vision/filter/KalmanFilter3D.cpp"
    "vision/kick/detector/FastKickDetector.cpp"
    "vision/kick/detector/SlowKickDetector.cpp"
    "vision/kick/estimator/ChipKickEstimator.cpp"
    "vision/kick/estimator/FlatKickEstimator.cpp"
    "vision/kick/KickEvent.cpp"
    "vision/robot/CameraRobot.cpp"
    "vision/robot/KalmanRobot.cpp"
    "vision/robot/WorldRobot.cpp"
    "vision/util/VisionFilterConfig.cpp"
    "vision/VisionFilter.cpp"
    "WindowEvaluator.cpp")


if(APPLE)
    # help CMake find gl.h, glu.h, etc on OS X
    link_directories(/opt/X11/lib/)
    include_directories(SYSTEM /opt/X11/include/)
    # lib dir for spnav
    link_directories(/usr/local/lib/)
endif()

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB SOCCER_UIS "${CMAKE_CURRENT_SOURCE_DIR}/ui/qt/*.ui")
qt5_wrap_ui(SOCCER_UIS ${SOCCER_UIS})

qt5_add_resources(SOCCER_RSRC ui/qt/main_icons.qrc)


# Boost Python
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system REQUIRED)

# find boost python
# This package is named 'python3' on Arch, 'python-py34' on Ubuntu 14, and 'python' on OS X
set(FOUND_A_BOOST_PYTHON FALSE)
# Find the matching boost python implementation
set(version ${PYTHONLIBS_VERSION_STRING})

if(APPLE)
    set(boost_py_version "")
else()
    STRING( REGEX REPLACE "[^0-9]" "" boost_py_version "${version}" )
    # Get first two numbers of python string
    STRING( REGEX MATCH "^[0-9][0-9]" boost_py_version "${boost_py_version}" )
endif()

foreach(possible_name python3 python-py${boost_py_version} python)
    find_package(Boost COMPONENTS ${possible_name} QUIET)
    if (Boost_FOUND)
        set(FOUND_A_BOOST_PYTHON TRUE)
        break()
    endif()
endforeach()
if (NOT FOUND_A_BOOST_PYTHON)
    message(FATAL_ERROR "Unable to find a suitable version of boost python")
endif()

# libusb
include_directories(SYSTEM ${LIBUSB_1_INCLUDE_DIRS})

# SDL2
include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)

# SDL
include_directories(SYSTEM ${SDL2_INCLUDE_DIRS})

# Build stand-alone soccer dylib
# This is linked into soccer and our unit tests, as well as being a python module
add_library(robocup SHARED ${ROBOCUP_LIB_SRC} ${SOCCER_UIS} ${SOCCER_RSRC})
set_target_properties(robocup PROPERTIES PREFIX "")
set_target_properties(robocup PROPERTIES SUFFIX ".so")

target_link_libraries(robocup ${robocup_common_LIBRARIES} ${robocup_fshare_LIBRARIES} ${rrt_LIBRARIES})
qt5_use_modules(robocup Widgets Xml Core OpenGL Network Svg)
target_link_libraries(robocup ${LIBUSB_1_LIBRARIES})
target_link_libraries(robocup ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
target_link_libraries(robocup GL GLU glut)
target_link_libraries(robocup pthread)
target_link_libraries(robocup spnav)
target_link_libraries(robocup ${SDL2_LIBRARIES})

include_directories(${robocup_common_INCLUDE_DIRS})
include_directories(${rrt_INCLUDE_DIRS})
include_directories(${robocup_fshare_INCLUDE_DIRS})

ament_target_dependencies(robocup ament_index_cpp rrt robocup_common robocup_fshare)

# python
# note: these are set in the root CMakeLists.txt file
include_directories(SYSTEM ${PYTHON_INCLUDE_DIRS})
link_directories(${PYTHON_LINK_DIRS})
target_link_libraries(robocup ${PYTHON_LIBRARIES})

# 'soccer' program
add_executable(soccer main.cpp)
qt5_use_modules(soccer Widgets Xml Core OpenGL Network Svg)
target_link_libraries(soccer robocup)

# ------------------------------------------------------------------------------

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    # Testing
    include_directories(${GTEST_INCLUDE_DIRS})

    # Add a test runner target "test-soccer" to run all tests in this directory
    set(SOCCER_TEST_SRC
        "BatteryProfileTest.cpp"
        "KickEvaluatorTest.cpp"
        "motion/TrapezoidalMotionTest.cpp"
        "optimization/GradientAscent1DTest.cpp"
        "optimization/ParallelGradientAscent1DTest.cpp"
        "optimization/NelderMead2DTest.cpp"
        "planning/PathTest.cpp"
        "planning/EscapeObstaclesPathPlannerTest.cpp"
        "planning/TargetVelPathPlannerTest.cpp"
        "TestMain.cpp"
        "vision/tests/CameraBallTest.cpp"
        "vision/tests/KalmanBallTest.cpp"
        "vision/tests/WorldBallTest.cpp"
        "vision/tests/CameraRobotTest.cpp"
        "vision/tests/KalmanRobotTest.cpp"
        "vision/tests/WorldRobotTest.cpp"
        "vision/tests/BallBounceTest.cpp"
        "vision/tests/CameraTest.cpp"
        "WindowEvaluatorTest.cpp"
    )

    ament_add_gtest(test-soccer ${SOCCER_TEST_SRC})
    if (TARGET test-soccer)
        target_link_libraries(test-soccer robocup ${GTEST_LIBRARIES})
        qt5_use_modules(test-soccer Core Widgets Xml)
    endif()
endif()

# ------------------------------------------------------------------------------

# Installation
install(TARGETS robocup
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(TARGETS soccer
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
    gameplay
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.py")

install(FILES
    config/realShopField.xml
    DESTINATION share/${PROJECT_NAME}/config
    RENAME soccer-real.cfg)

install(FILES
    config/sim.xml
    DESTINATION share/${PROJECT_NAME}/config
    RENAME soccer-sim.cfg)

install(FILES
    sdlcontrollerdb/gamecontrollerdb.txt
    DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rrt robocup_common robocup_fshare grSim)
ament_package()

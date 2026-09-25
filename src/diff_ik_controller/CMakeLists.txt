cmake_minimum_required(VERSION 3.8)
project(diff_ik_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hand_teleop_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- MuJoCo C library ----
# 1) an installed MuJoCo CMake package, 2) a release tarball in MUJOCO_ROOT or
# third_party/mujoco, 3) the headers and library shipped in the `mujoco` pip wheel (the same
# version the Python simulator uses).
find_package(mujoco QUIET)
if(TARGET mujoco::mujoco)
  set(MUJOCO_TARGET mujoco::mujoco)
else()
  set(_repo_root "${CMAKE_CURRENT_SOURCE_DIR}/../..")
  if(NOT MUJOCO_ROOT AND DEFINED ENV{MUJOCO_ROOT})
    set(MUJOCO_ROOT $ENV{MUJOCO_ROOT})
  endif()
  if(NOT MUJOCO_ROOT AND EXISTS "${_repo_root}/third_party/mujoco/include/mujoco/mujoco.h")
    set(MUJOCO_ROOT "${_repo_root}/third_party/mujoco")
  endif()
  if(MUJOCO_ROOT)
    set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
    file(GLOB MUJOCO_LIBRARY "${MUJOCO_ROOT}/lib/libmujoco.so*")
  else()
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
      OUTPUT_VARIABLE _mj_py OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _mj_rc)
    if(NOT _mj_rc EQUAL 0)
      message(FATAL_ERROR "MuJoCo not found: set MUJOCO_ROOT or pip install mujoco")
    endif()
    set(MUJOCO_INCLUDE_DIR "${_mj_py}/include")
    file(GLOB MUJOCO_LIBRARY "${_mj_py}/libmujoco.so*")
  endif()
  list(GET MUJOCO_LIBRARY 0 MUJOCO_LIBRARY)
  get_filename_component(MUJOCO_LIB_DIR "${MUJOCO_LIBRARY}" DIRECTORY)
  message(STATUS "MuJoCo: ${MUJOCO_LIBRARY}")
  add_library(mujoco_imported SHARED IMPORTED)
  set_target_properties(mujoco_imported PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
  set(MUJOCO_TARGET mujoco_imported)
  set(CMAKE_BUILD_RPATH "${MUJOCO_LIB_DIR}")
  set(CMAKE_INSTALL_RPATH "${MUJOCO_LIB_DIR}")
endif()

# ---- pure library (no ROS) ----
add_library(diff_ik STATIC src/kinematics.cpp src/diff_ik.cpp)
target_include_directories(diff_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(diff_ik PUBLIC ${MUJOCO_TARGET} Eigen3::Eigen)
set_target_properties(diff_ik PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- node ----
add_executable(diff_ik_node src/diff_ik_node.cpp)
target_link_libraries(diff_ik_node diff_ik)
ament_target_dependencies(diff_ik_node rclcpp ament_index_cpp sensor_msgs std_msgs std_srvs
  geometry_msgs diagnostic_msgs hand_teleop_msgs)
install(TARGETS diff_ik_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_diff_ik test/test_diff_ik.cpp TIMEOUT 300)
  target_link_libraries(test_diff_ik diff_ik)
  target_compile_definitions(test_diff_ik PRIVATE REPO_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../config")
  ament_target_dependencies(test_diff_ik ament_index_cpp)
  ament_add_gtest(test_estop_latch test/test_estop_latch.cpp)
  target_link_libraries(test_estop_latch diff_ik)
  add_launch_test(test/test_square_launch.py TIMEOUT 180)
  add_launch_test(test/test_estop_launch.py TIMEOUT 120)
endif()

ament_package()

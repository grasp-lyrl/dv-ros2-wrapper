cmake_minimum_required(VERSION 3.14)
project(dv_ros2_monodepth VERSION 1.3.0)

# dv-processing needs C++20; Torch is happy to be built against it.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dv_ros2_msgs REQUIRED)
find_package(dv_ros2_messaging REQUIRED)
find_package(dv-processing 2.0.0 REQUIRED)
find_package(OpenCV REQUIRED)

# Torch ships its CMake config inside the python package on both the Orin wheel and a
# conda install, so ask python where it is unless the caller already pointed us at one.
if(NOT DEFINED Torch_DIR)
  execute_process(
    COMMAND python3 -c "import torch; print(torch.utils.cmake_prefix_path)"
    OUTPUT_VARIABLE TORCH_CMAKE_PREFIX_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE TORCH_PREFIX_RESULT
  )
  if(NOT TORCH_PREFIX_RESULT EQUAL 0)
    message(FATAL_ERROR
      "Could not locate Torch. Install pytorch for python3, or configure with "
      "-DTorch_DIR=/path/to/libtorch/share/cmake/Torch")
  endif()
  list(APPEND CMAKE_PREFIX_PATH "${TORCH_CMAKE_PREFIX_PATH}")
endif()
# Caffe2Config resolves nvcc gencode flags before it will load, and its GPU
# auto-detection compiles and runs a probe -- which fails in a container that has the
# CUDA libraries but not a full toolchain. Nothing here compiles CUDA, so the value only
# has to exist. 8.7 is the Orin, 8.9 is Ada; override the env var for anything else.
if(NOT DEFINED ENV{TORCH_CUDA_ARCH_LIST})
  set(ENV{TORCH_CUDA_ARCH_LIST} "8.7;8.9")
endif()

# Caffe2Config enables the CUDA language before it will load, so nvcc has to be findable
# even though nothing here compiles CUDA. Locate it ourselves rather than depending on
# whatever PATH the build was invoked with.
if(NOT DEFINED CMAKE_CUDA_COMPILER)
  find_program(NVCC_EXECUTABLE nvcc
    HINTS ${CUDAToolkit_ROOT} $ENV{CUDA_HOME} /usr/local/cuda /usr/local/cuda-12.8
    PATH_SUFFIXES bin)
  if(NVCC_EXECUTABLE)
    set(CMAKE_CUDA_COMPILER "${NVCC_EXECUTABLE}")
    message(STATUS "Using nvcc at ${NVCC_EXECUTABLE}")
  endif()
endif()

find_package(Torch REQUIRED)
# For cudaSetDeviceFlags: the node picks the driver's GPU-wait strategy explicitly.
find_package(CUDAToolkit REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(monodepth_node_component SHARED src/monodepth_node.cpp)

ament_target_dependencies(monodepth_node_component
  rclcpp rclcpp_components std_msgs sensor_msgs dv_ros2_msgs dv_ros2_messaging
)

target_link_libraries(monodepth_node_component
  dv::processing
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
  CUDA::cudart
)

rclcpp_components_register_nodes(monodepth_node_component "dv_monodepth_node::MonoDepthNode")

install(TARGETS monodepth_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

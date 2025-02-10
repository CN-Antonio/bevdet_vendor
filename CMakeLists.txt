# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Copyright 2024 AutoCore, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.14.0)
project(bevdet_vendor LANGUAGES CXX)# CUDA)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================

# set(CMAKE_CXX_FLAGS_RELEASE "-Wno-deprecated-declarations -O2")
# set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_BUILD_TYPE "DEBUG")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++17)  
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-w) # only output error, ignore warining
  # set(CMAKE_CXX_FLAGS_RELEASE "-Wno-deprecated-declarations -O2") # warining: deprecated usage
endif()

#========================
# Dependencies Setup
#========================
option(CUDA_VERBOSE "Verbose output of CUDA modules" ON)

# ROS2 #
find_package(rclcpp REQUIRED)

if(rclcpp_FOUND)
  message(=============================================================)
  message("-- ROS2 Found. ROS2 Support is turned On.")
  message(=============================================================)

  add_definitions(-DROS2_FOUND)
  include_directories(${rclcpp_INCLUDE_DIRS})
  set(CMAKE_CXX_STANDARD 14)

  ### all ros2 function package ###
  # find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_auto REQUIRED)
  # find_package(ament_cmake_ros REQUIRED)

  set(DEPENDENCIES
    rclcpp
    message_filters
    sensor_msgs
    geometry_msgs
    visualization_msgs

    cv_bridge
    pcl_conversions
    pcl_ros 

    # not related to ROS

  )

  foreach (dependency IN LISTS DEPENDENCIES)
    find_package(${dependency} REQUIRED)
  endforeach()

  ament_auto_find_build_dependencies(REQUIRED
    rclcpp
    ${DEPENDENCIES}
  )
endif()

# not related to ROS
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# TODO: remove
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# Nvidia BEGIN ##############################################################################
find_package(cudnn_cmake_module REQUIRED)
find_package(tensorrt_cmake_module REQUIRED)

find_package(CUDAToolkit)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  # Note: cublas_device was depreciated in CUDA version 9.2
  #       https://forums.developer.nvidia.com/t/where-can-i-find-libcublas-device-so-or-libcublas-device-a/67251/4
  #       In LibTorch, CUDA_cublas_device_LIBRARY is used.
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
find_package(TENSORRT)
if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL  "aarch64")
  set(TENSORRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu/)
  set(TENSORRT_LIBRARY_DIRS /usr/lib/aarch64-linux-gnu/)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(TENSORRT_INCLUDE_DIRS /root/TensorRT-8.5.2.2/include)  # your tensorrt path
  set(TENSORRT_LIBRARY_DIRS /root/TensorRT-8.5.2.2/lib)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_package(CUDNN)

if(NOT (CUDAToolkit_FOUND AND CUDNN_FOUND AND TENSORRT_FOUND))
  message(WARNING "cuda, cudnn, tensorrt libraries are not found")
  return()
endif()
# Nvidia END ##############################################################################

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/bevdet

  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIRS}
  # ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}

  # TODO: remove
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
	${OpenCV_LIBRARY_DIRS}
  # ${CMAKE_CUDA_INCLUDE_DIRS}
  ${TENSORRT_LIBRARY_DIRS}
)

#========================
# Build Setup
#========================

# Build non-CUDA bevdet dependent parts
ament_auto_add_library(bevdet_lib SHARED
  # unsure to remove
  # src/bevdet/cpu_jpegdecoder.cpp
  # src/bevdet/nvjpegdecoder.cpp

  # TODO:remove
  # src/bevdet/data.cpp # include class "DataLoader"，加载数据，cpu/nvjpegdec解码，

  src/test.cpp
)

# Build CUDA bevdet dependent parts
cuda_add_library(bevdet_cuda_lib SHARED
  # confirm to keep
  src/bevdet/bevdet.cpp   # 包含class bevdet

  src/bevdet/preprocess.cu
  src/bevdet/bevpool.cu 
  src/bevdet/iou3d_nms.cu
  src/bevdet/postprocess.cu
  src/bevdet/grid_sampler.cu

#   src/bevdet/preprocess_plugin.cu
#   src/bevdet/bevpool_plugin.cu
#   src/bevdet/alignbev_plugin.cu
#   src/bevdet/gatherbev_plugin.cu

  # TODO:remove
  src/bevdet/data.cpp
)

# Build bevdet ros parts
ament_auto_add_library(bevdet_node SHARED
  src/bevdet_node.cpp
)

target_link_libraries(bevdet_node
  bevdet_lib
  bevdet_cuda_lib

  # ${CUDA_LIBRARIES}
  # ${CUDA_CUDA_LIBRARY}
  # ${CUDA_CUDART_LIBRARY}
  
  yaml-cpp 
  libnvinfer.so
  libjpeg.so
  ${OpenCV_LIBS}
  # ${PCL_LIBRARIES}
)

rclcpp_components_register_node(bevdet_node
  PLUGIN "bevdet::bevdet_ros::BEVDet_Node"
  EXECUTABLE ${PROJECT_NAME}_node
)

# remove
# set(bevdet_source_files
#   src/bevdet.cpp

#   src/preprocess.cu
#   src/iou3d_nms.cu
#   src/postprocess.cu
#   src/data.cpp
#   src/cpu_jpegdecoder.cpp
#   src/nvjpegdecoder.cpp

#   src/preprocess_plugin.cu
#   src/bevpool_plugin.cu
#   src/alignbev_plugin.cu
#   src/gatherbev_plugin.cu
# )

# remove
# cuda_add_library(${PROJECT_NAME} SHARED
#   ${bevdet_source_files}
# )

# remove
# target_include_directories(
#   ${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
#   ${YAML_CPP_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
# )

# remove
# target_link_libraries(
#   ${PROJECT_NAME}
#   CUDA::cudart
#   yaml-cpp
#   ${NVINFER}
#   ${NVONNXPARSER}
#   ${TENSORRT_LIBRARIES}
#   ${CUDA_LIBRARIES}
#   ${CUBLAS_LIBRARIES}
#   ${CUDNN_LIBRARY}
# )

### ROS2 ###
if(rclcpp_FOUND)
  ament_target_dependencies(
    ${PROJECT_NAME}_node 

    rclcpp
    ${DEPENDENCIES}
  )

  set_target_properties(${PROJECT_NAME}_node
    PROPERTIES
      CXX_STANDARD 17
      CXX_STANDARD_REQUIRED YES
      CXX_EXTENSIONS NO
  )

  # install(
  #   DIRECTORY include/
  #   DESTINATION include/${PROJECT_NAME}
  # )

  install(TARGETS
    ${PROJECT_NAME}_node

    DESTINATION lib/${PROJECT_NAME}
    # EXPORT export_${PROJECT_NAME}
  )

  ament_auto_package(INSTALL_TO_SHARE
    config
    launch
    model
  )

  # ament_export_targets(export_${PROJECT_NAME})

  # ament_export_dependencies(
  #   "CUDA"
  #   "CUDAToolkit"
  #   "cudnn_cmake_module"
  #   "CUDNN"
  #   "rclcpp"
  #   "tensorrt_cmake_module"
  #   "TENSORRT"
  # )

endif()
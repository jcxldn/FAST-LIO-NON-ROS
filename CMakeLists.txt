cmake_minimum_required(VERSION 3.10) # min value for modern cmake


set(CMAKE_POLICY_VERSION_MINIMUM 3.5) # cjson has no min version

project(fast_lio)


set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")


message("Current CPU arch: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

if(YAMLCPP_STATIC_LIBRARY)
    set(YAMLCPP_STATIC libyaml-cpp.a)
endif()


find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

include(deps.cmake)

#add_executable(fastlio_mapping src/fast_lio.cpp src/main.cpp src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
#target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} yaml-cpp nlohmann_json::nlohmann_json)
#target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

add_library(fastlio SHARED
    src/common_lib.cpp
    src/fast_lio.cpp
    src/IMU_Processing.cpp
    src/laserMapping.cpp 
    src/preprocess.cpp
    src/use-ikfom.cpp
    include/ikd-Tree/ikd_Tree.cpp
)
target_link_libraries(fastlio  pcl_common pcl_kdtree pcl_filters yaml-cpp nlohmann_json::nlohmann_json)
target_include_directories(fastlio PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src)
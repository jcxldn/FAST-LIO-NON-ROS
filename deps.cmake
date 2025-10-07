include(FetchContent)


FetchContent_Declare(
  eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen/
  GIT_TAG        3.4.1
)



option(ENABLE_CJSON_UNINSTALL OFF)
option(ENABLE_CJSON_TEST OFF)

FetchContent_Declare(
  cjson
  GIT_REPOSITORY https://github.com/DaveGamble/cJSON
  GIT_TAG        v1.7.19
)

FetchContent_Declare(
  pcl
  GIT_REPOSITORY https://github.com/PointCloudLibrary/pcl
  GIT_TAG        pcl-1.15.1
)

FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json
  GIT_TAG        v3.12.0
)

FetchContent_Declare(
  yaml
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp
  GIT_TAG        0.8.0
)

FetchContent_MakeAvailable(eigen3 cjson pcl json yaml)

target_include_directories(pcl_common PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/_deps/pcl-build/include)
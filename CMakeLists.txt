cmake_minimum_required(VERSION 2.6)

project(datmo)

# External Libraries
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package (OpenCV REQUIRED)

# stereo
FILE(
  GLOB
  stereo_SOURCES
  stereo/*.cpp
  common/*.cpp
  )

FILE(
  GLOB
  stereo_HEADERS
  stereo/*.hpp
  common/*.hpp
  )

ADD_EXECUTABLE(stereo
    ${stereo_SOURCES}
    ${stereo_HEADRERS}
  )

TARGET_LINK_LIBRARIES(stereo
    ${OpenCV_LIBRARIES}
)

# lidar
FILE(
  GLOB
  lidar_SOURCES
  lidar/*.cpp
  common/*.cpp
  )

FILE(
  GLOB
  lidar_HEADERS
  lidar/*.hpp
  common/*.hpp
  )

ADD_EXECUTABLE(lidar
    ${lidar_SOURCES}
    ${lidar_HEADRERS}
  )

TARGET_LINK_LIBRARIES(lidar
    ${OpenCV_LIBRARIES}
)

# run
# stereo
ADD_CUSTOM_TARGET(run_stereo
    COMMAND stereo data/img_stereo/left_250.png
    DEPENDS stereo
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
)

# lidar
ADD_CUSTOM_TARGET(run_lidar
    COMMAND lidar data/lidarData.xml
    DEPENDS lidar
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
)


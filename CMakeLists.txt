cmake_minimum_required(VERSION 3.5)

project(yolomech)

set(CMAKE_CXX_STANDARD 14)

# Mech EYE
find_package(PkgConfig)
if(NOT PkgConfig_FOUND)
    message(FATAL_ERROR "PkgConfig not found.")
else()
    pkg_check_modules(MECHEYEAPI REQUIRED MechEyeApi)
    if(NOT MECHEYEAPI_FOUND)
      message(
        FATAL_ERROR "MechEyeApi not found. Please install MechEyeApi first.")
    endif()
endif()
include_directories(${MECHEYEAPI_INCLUDE_DIRS})
link_directories(${MECHEYEAPI_LIBRARY_DIRS})

# pcl
find_package(PCL REQUIRED)
if(NOT PCL_FOUND)
  message(
    FATAL_ERROR
      "Point Cloud Library (PCL) not found. Please point PCL_DIR to the directory of your PCL installation (containing the file PCLConfig.cmake)."
  )
endif()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(-DBOOST_ALL_DYN_LINK)

# TBB
find_package(TBB REQUIRED)

add_library (template_alignment SHARED template_alignment.cpp)
# add_executable (template_alignment template_alignment.cpp)
add_executable (shoot_template shoot_template.cpp)

target_link_libraries (template_alignment pthread ${MECHEYEAPI_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} TBB::tbb)
target_link_libraries (shoot_template pthread ${MECHEYEAPI_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} TBB::tbb)
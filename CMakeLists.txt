cmake_minimum_required(VERSION 3.5)

cmake_policy(SET CMP0048 NEW) # CMake 3.6 required

if(WIN32)
  cmake_policy(SET CMP0074 NEW) # CMake 3.12 required
endif(WIN32)

project(ag_driver VERSION 1.0.0)

#========================
#  cmake_build_type
#========================
if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()
message("-- Build ${CMAKE_BUILD_TYPE}")

execute_process(
  COMMAND lsb_release -rs
  OUTPUT_VARIABLE UBUNTU_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(UBUNTU_VERSION STREQUAL "22.04")
  message("This is Ubuntu 22.04, need to use /pcd_io.h instead of /io.h")
  add_definitions(-DPCL_USE_PCDIO)
endif()

#=============================
#  Compile Features
#=============================
option(DISABLE_PCAP_PARSE         "Disable PCAP file parse" OFF) 
option(ENABLE_TRANSFORM           "Enable transform functions" OFF)

option(ENABLE_OPENMP              "Enable omp feature" OFF)
option(ENABLE_DOUBLE_RCVBUF       "Enable double size of RCVBUF" ON)
option(ENABLE_THREAD_PRIORITY     "Enable increase the priority of recv and process thread" OFF)
option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)

option(ENABLE_STAMP_WITH_LOCAL    "Enable stamp point cloud with local time" OFF)
option(ENABLE_PCL_POINTCLOUD      "Enable PCL Point Cloud" OFF)
option(ENABLE_CRC32_CHECK         "Enable CRC32 Check on MSOP Packet" OFF)
option(ENABLE_DIFOP_PARSE         "Enable parsing DIFOP Packet" OFF)
option(ENABLE_POINT_EXTEND_FIELD  "Enable point extended field output" OFF)

#=============================
#  Compile Demos, Tools
#=============================
option(COMPILE_DEMOS "Build lidar driver demos" ON)
option(COMPILE_TOOLS "Build lidar driver tools" ON)
option(COMPILE_TOOL_VIEWER "Build point cloud visualization tool" OFF)
option(COMPILE_TOOL_PCDSAVER "Build point cloud pcd saver tool" OFF)
option(COMPILE_TOOL_CSVSAVER "Build point cloud csv saver tool" OFF)

#========================
#  Platform cross setup
#========================
if(MSVC)

  set(COMPILER "MSVC Compiler")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /Zi")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
  set(CompilerFlags
    CMAKE_CXX_FLAGS                       CMAKE_C_FLAGS
    CMAKE_CXX_FLAGS_DEBUG                 CMAKE_C_FLAGS_DEBUG
    CMAKE_CXX_FLAGS_RELEASE               CMAKE_C_FLAGS_RELEASE
	  CMAKE_CXX_FLAGS_MINSIZEREL            CMAKE_C_FLAGS_MINSIZEREL
	  CMAKE_CXX_FLAGS_RELWITHDEBINFO        CMAKE_C_FLAGS_RELWITHDEBINFO)

  foreach(CompilerFlag ${CompilerFlags})
    string(REPLACE "/MT" "/MD" ${CompilerFlag} "${${CompilerFlag}}")
  endforeach()

  add_compile_definitions(_DISABLE_EXTENDED_ALIGNED_STORAGE) # to disable a msvc error
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:100000000")

elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")

  if(UNIX)
    set(COMPILER "UNIX GNU Compiler")
  elseif(MINGW)
    set(COMPILER "MINGW Compiler")
  else()
    message(FATAL_ERROR "Unsupported compiler.")
  endif()

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-unused-parameter")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()

message(=============================================================)
message("-- CMake run for ${COMPILER}")
message(=============================================================)

#========================
#  Path Setup
#========================
add_definitions(-DDRIVER_PROJECT_PATH="${PROJECT_SOURCE_DIR}")

set(DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
set(DRIVER_CMAKE_ROOT ${CMAKE_CURRENT_LIST_DIR}/cmake)

set(CMAKE_INSTALL_PREFIX /usr/local)
set(INSTALL_DRIVER_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_PROJECT_NAME}/include)
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake)

if(WIN32)
else()
  if (CMAKE_SYSTEM_NAME STREQUAL "QNX")
    list(APPEND EXTERNAL_LIBS socket)
  else()
    list(APPEND EXTERNAL_LIBS pthread)
  endif()
endif(WIN32)

#========================
#  Build Features
#========================
if(${DISABLE_PCAP_PARSE})

  message(=============================================================)
  message("-- Disable PCAP parse")
  message(=============================================================)

  add_definitions("-DDISABLE_PCAP_PARSE")

else()

  if(WIN32)
    set(COMPILE_TOOLS ON)
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
    find_package(PCAP REQUIRED)
    include_directories(${PCAP_INCLUDE_DIR})
    list(APPEND EXTERNAL_LIBS ${PCAP_LIBRARY} ws2_32)
    add_definitions(-DNOMINMAX)
  else()
    list(APPEND EXTERNAL_LIBS pcap)
  endif(WIN32)

endif(${DISABLE_PCAP_PARSE})


if(${ENABLE_TRANSFORM})
  message(=============================================================)
  message("-- Enable Transform Fucntions")
  message(=============================================================)

  add_definitions("-DENABLE_TRANSFORM")

  # Eigen
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(${ENABLE_TRANSFORM})

if(${ENABLE_OPENMP})
    add_definitions("-DENABLE_OPENMP")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()

if(${ENABLE_DOUBLE_RCVBUF})
  add_definitions("-DENABLE_DOUBLE_RCVBUF")
endif(${ENABLE_DOUBLE_RCVBUF})

if(${ENABLE_THREAD_PRIORITY})
  add_definitions("-DENABLE_THREAD_PRIORITY")
  add_definitions("-DHIGH_THREAD_PRIORITY=11")
endif(${ENABLE_THREAD_PRIORITY})

if(${ENABLE_WAIT_IF_QUEUE_EMPTY})
  add_definitions("-DENABLE_WAIT_IF_QUEUE_EMPTY")
endif(${ENABLE_WAIT_IF_QUEUE_EMPTY})

if(${ENABLE_STAMP_WITH_LOCAL})
  add_definitions("-DENABLE_STAMP_WITH_LOCAL")
endif(${ENABLE_STAMP_WITH_LOCAL})

if(${ENABLE_PCL_POINTCLOUD})
  add_definitions("-DENABLE_PCL_POINTCLOUD")
endif(${ENABLE_PCL_POINTCLOUD})

if(${ENABLE_CRC32_CHECK})
  add_definitions("-DENABLE_CRC32_CHECK")
endif(${ENABLE_CRC32_CHECK})

if(${ENABLE_DIFOP_PARSE})
  add_definitions("-DENABLE_DIFOP_PARSE")
endif(${ENABLE_DIFOP_PARSE})

if(${ENABLE_POINT_EXTEND_FIELD})
  add_definitions("-DENABLE_POINT_EXTEND_FIELD")
endif(${ENABLE_POINT_EXTEND_FIELD})

if(${COMPILE_DEMOS})
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/demo)
endif(${COMPILE_DEMOS})

if (${COMPILE_TOOLS})
  add_definitions("-DCOMPILE_TOOLS")
  set(COMPILE_TOOL_VIEWER ON)
  set(COMPILE_TOOL_PCDSAVER ON)
  set(COMPILE_TOOL_CSVSAVER ON)
endif (${COMPILE_TOOLS})

if(${COMPILE_TOOL_VIEWER} OR ${COMPILE_TOOL_PCDSAVER} OR ${COMPILE_TOOL_CSVSAVER})
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tool)
endif(${COMPILE_TOOL_VIEWER} OR ${COMPILE_TOOL_PCDSAVER} OR ${COMPILE_TOOL_CSVSAVER})

#========================
#  Cmake
#========================  
configure_file(
  ${DRIVER_CMAKE_ROOT}/ag_driverConfig.cmake.in
  ${DRIVER_CMAKE_ROOT}/ag_driverConfig.cmake 
  @ONLY)


configure_file (
  ${DRIVER_CMAKE_ROOT}/version.hpp.in
  ${CMAKE_CURRENT_LIST_DIR}/src/ag_driver/common/version.hpp
  @ONLY)

if(NOT ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  set(ag_driver_DIR ${DRIVER_CMAKE_ROOT} PARENT_SCOPE)
endif()

#========================
#  Install & Uninstall
#========================
if(UNIX  AND  ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

  install(FILES ${DRIVER_CMAKE_ROOT}/ag_driverConfig.cmake
          DESTINATION ${INSTALL_CMAKE_DIR}/${CMAKE_PROJECT_NAME})

  install(DIRECTORY src/
          DESTINATION ${INSTALL_DRIVER_DIR}
          FILES_MATCHING PATTERN "*.h")

  install(DIRECTORY src/
          DESTINATION ${INSTALL_DRIVER_DIR}
          FILES_MATCHING PATTERN "*.hpp")

  if(NOT TARGET uninstall)
    configure_file(
      ${DRIVER_CMAKE_ROOT}/cmake_uninstall.cmake.in
      ${DRIVER_CMAKE_ROOT}/cmake_uninstall.cmake 
      @ONLY)
    add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${DRIVER_CMAKE_ROOT}/cmake_uninstall.cmake)
  endif(NOT TARGET uninstall)

endif(UNIX AND ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})


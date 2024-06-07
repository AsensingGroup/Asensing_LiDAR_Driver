cmake_minimum_required(VERSION 3.5)

project(ag_driver_tools)

message(=============================================================)
message("-- Ready to compile tools")
message(=============================================================)

include_directories(${DRIVER_INCLUDE_DIRS})

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

if(${COMPILE_TOOL_VIEWER})

if(WIN32)
  cmake_policy(SET CMP0074 NEW)
  set(OPENNI_ROOT "C:\\Program Files\\OpenNI2")
  set(OPENNI_LIBRARY "${OPENNI_ROOT}\\Lib\\OpenNI2.lib")
  set(OPENNI_INCLUDE_DIRS "${OPENNI_ROOT}\\Include\\")
  file(COPY ${OPENNI_ROOT}\\Redist\\OpenNI2.dll DESTINATION ${PROJECT_BINARY_DIR}\\Release)
  file(COPY ${OPENNI_ROOT}\\Redist\\OpenNI2.dll DESTINATION ${PROJECT_BINARY_DIR}\\Debug)
endif(WIN32)

find_package(PCL COMPONENTS common visualization io QUIET REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

if(PCL_FOUND)

add_executable(simple_viewer
               simple_viewer.cpp)

target_link_libraries(simple_viewer
                    ${EXTERNAL_LIBS}    
                    ${PCL_LIBRARIES})

else()

message("PCL Not found! Can not compile simple_viewer!")

endif()

install(TARGETS simple_viewer
        RUNTIME DESTINATION /usr/bin)     

endif (${COMPILE_TOOL_VIEWER})


if(${COMPILE_TOOL_PCDSAVER})

add_executable(simple_pcdsaver
               simple_pcdsaver.cpp)

target_link_libraries(simple_pcdsaver
                    ${EXTERNAL_LIBS})

endif(${COMPILE_TOOL_PCDSAVER})

if(${COMPILE_TOOL_CSVSAVER})

add_executable(simple_csvsaver
               simple_csvsaver.cpp)

target_link_libraries(simple_csvsaver
                    ${EXTERNAL_LIBS})

endif(${COMPILE_TOOL_CSVSAVER})

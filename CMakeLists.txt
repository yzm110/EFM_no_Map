
# cmake 版本
cmake_minimum_required(VERSION 3.5.1)

# inner variables
set(PROJECT_NAME efm_exe)
set(BINARY_NAME  efm_exe)
# add_definitions(-DLM_LANEGROUP_RES)
# add_definitions(-DLM_LANEGROUP)
# add_definitions(-DRELOCATE_INNER)
# add_definitions(-DALL_LINE)
# add_definitions(-DLM_CONNECT_RES)
#add_definitions(-DLM_SORT)
# add_definitions(-DWINPUT)
# add_definitions(-DWOUT)
# add_definitions(-DLOC)
#add_definitions(-DLM_CONNECT)

project(${BINARY_NAME})

string(REGEX REPLACE ".*/([^/]+)$" "\\1" TOOLCHAIN_NAME ${CMAKE_PLATFORM_TYPE})
set(FRAMEWORK_APP_CMAKE_PLATFORM_TYPE "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/libs/${TOOLCHAIN_NAME}")
message(">> framework App New Toolchain Path: ${FRAMEWORK_APP_CMAKE_PLATFORM_TYPE}")

set(INCLUDE_DIRECTORIES
    ./include
    ../../framework/efm_exe/abstract
    ../../framework/efm_exe/datatype
    ../../framework/datatype_common
    ${FRAMEWORK_APP_CMAKE_PLATFORM_TYPE}/ap_framework/frrt/include/spdlog
    ${FRAMEWORK_APP_CMAKE_PLATFORM_TYPE}/ap_framework/frrt/include
    ${CMAKE_PLATFORM_TYPE}/esr/fc_linux_pack/opt/erte/include
)

#set(SOURCE_FILE
#    ComponentEFMTaskApp.cpp
#)

set(SOURCE_DIRS 
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

foreach(DIR ${SOURCE_DIRS})
    file(GLOB DIR_SOURCES "${DIR}/*.cpp")
    list(APPEND SOURCE_FILE ${DIR_SOURCES})
endforeach()

set(LINK_DIRECTORIES

)

set(LINK_LIBRARIES 

)

set(C_FLAGS            "-s")
set(C_FLAGS_DEBUG      "-g")
set(C_FLAGS_RELEASE    "-g")
set(CXX_FLAGS          "-std=c++17")
set(CXX_FLAGS_DEBUG    "-g -std=c++17")
#set(CXX_FLAGS_RELEASE  "-g -std=c++17")
set(CXX_FLAGS_RELEASE  "-O3 -Wall -std=c++17")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG    "${CMAKE_EXE_LINKER_FLAGS_DEBUG}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE  "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Wl,--strip-debug")

# output messages
message(">> project name         : ${PROJECT_NAME}")
message(">> CC                   : ${CMAKE_C_COMPILER}")
message(">> CXX                  : ${CMAKE_CXX_COMPILER}")
message(">> SYSROOT              : ${CMAKE_SYSROOT}")

message(">> AP_DEVEL_LIBS        : ${CMAKE_AP_DEVEL_LIBS}")
message(">> AP_DEVEL_PATH        : ${CMAKE_AP_DEVEL_PATH}")
message(">> AP_MODEL_GEN_PATH    : ${CMAKE_BINARY_SRC_GEN_PATH}")

message(">> include directories  : ${INCLUDE_DIRECTORIES}")
message(">> link directories     : ${LINK_DIRECTORIES}")
message(">> source files         : ${SOURCE_FILE}")
message(">> link library         : ${LINK_LIBRARIES}")
message("")

# build
#set(CMAKE_C_FLAGS                "${C_FLAGS}")
#set(CMAKE_C_FLAGS_DEBUG          "${C_FLAGS_DEBUG}")
#set(CMAKE_C_FLAGS_RELEASE        "${C_FLAGS_RELEASE}")
#set(CMAKE_CXX_FLAGS              "${CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS_DEBUG        "${CXX_FLAGS_DEBUG}")
#set(CMAKE_CXX_FLAGS_RELEASE      "${CXX_FLAGS_RELEASE}")
set(CMAKE_C_FLAGS                "${CMAKE_C_FLAGS} ${C_FLAGS}")
set(CMAKE_C_FLAGS_DEBUG          "${CMAKE_C_FLAGS_DEBUG} ${C_FLAGS_DEBUG}")
set(CMAKE_C_FLAGS_RELEASE        "${CMAKE_C_FLAGS_RELEASE} ${C_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS              "${CMAKE_CXX_FLAGS} ${CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG        "${CMAKE_CXX_FLAGS_DEBUG} ${CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE      "${CMAKE_CXX_FLAGS_RELEASE} ${CXX_FLAGS_RELEASE} ")

link_directories(${LINK_DIRECTORIES})
include_directories(${INCLUDE_DIRECTORIES})
add_library( ${BINARY_NAME} SHARED ${SOURCE_FILE})
target_link_libraries(${BINARY_NAME} ${LINK_LIBRARIES})

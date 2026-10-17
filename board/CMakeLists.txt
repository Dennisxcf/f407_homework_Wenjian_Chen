cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME board)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# ===========================================================================
# [AI] 以下内容由 agent 追加，未修改上面任何原有行。
#      目的：启用 C++，并把框架的 bsp / libs / modules / threads 纳入编译。
#      说明：CubeMX 的 stm32cubemx 是 INTERFACE 库，已经把
#            Core/Inc、Drivers、ThreadX 的头文件路径和 STM32F407xx /
#            USE_HAL_DRIVER / TX_INCLUDE_USER_DEFINE_FILE 这些宏通过
#            target_link_libraries + target_include_directories(INTERFACE)
#            传递过来了，所以下面只需要补 C++ 标准、源文件和本项目自己的 include。
# ===========================================================================

enable_language(CXX)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# [AI] 用 GLOB + CONFIGURE_DEPENDS：以后在 bsp/libs/modules/threads 里新增 .cpp
#      会自动纳入编译，不用回来改这个文件（新增文件后重新 configure 由
#      CONFIGURE_DEPENDS 自动触发）。若想改成显式清单，把下面 4 行 GLOB 换成
#      逐个列出文件名即可。
file(GLOB BSP_SRC     CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../bsp/src/*.cpp)
file(GLOB LIBS_SRC    CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../libs/src/*.cpp)
file(GLOB MODULES_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../modules/src/*.cpp)
file(GLOB THREADS_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../threads/src/*.cpp)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${BSP_SRC}
    ${LIBS_SRC}
    ${MODULES_SRC}
    ${THREADS_SRC}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../bsp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../threads/include
)

# [AI] 配置时打印一下抓到了哪些源文件，方便确认 GLOB 生效
list(LENGTH BSP_SRC     BSP_CNT)
list(LENGTH LIBS_SRC    LIBS_CNT)
list(LENGTH MODULES_SRC MODULES_CNT)
list(LENGTH THREADS_SRC THREADS_CNT)
message(STATUS "[AI] framework sources: bsp=${BSP_CNT} libs=${LIBS_CNT} modules=${MODULES_CNT} threads=${THREADS_CNT}")

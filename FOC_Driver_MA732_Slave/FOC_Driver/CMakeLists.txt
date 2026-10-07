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


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FOC_Driver)


 
# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Set the source directory
set(CMSIS_CORE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include)
include_directories(${CMSIS_CORE_INCLUDE_DIR})

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add CMSIS DSP library
add_subdirectory(${CMAKE_SOURCE_DIR}/Drivers/CMSIS-DSP)   # Foo/CMakeLists.txt 실행

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB APPLICATION_SOURCES
    "${CMAKE_SOURCE_DIR}/Application/*/Src/*.cpp"

)
list(APPEND APPLICATION_SOURCES
    "${CMAKE_SOURCE_DIR}/Application/Control/Src/control_params.cpp"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APPLICATION_SOURCES}
)

file(GLOB APPLICATION_HEADERS_LOC
    "${CMAKE_SOURCE_DIR}/Application/*/Inc"
)



# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${APPLICATION_HEADERS_LOC}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSISDSP
    # Add user defined libraries
)

#Set compiler options
target_compile_options(CMSISDSP PRIVATE
    $<$<AND:$<CONFIG:Release>,$<C_COMPILER_ID:GNU,Clang,ARM>>:-Ofast>)

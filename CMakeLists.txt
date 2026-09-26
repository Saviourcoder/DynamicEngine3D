 
 #DYNAMICENGINE3D
 #AI-Assisted 3D Physics Engine
 #By: Elitmers

cmake_minimum_required(VERSION 3.16)
project(DynamicEngine3D VERSION 1.0 LANGUAGES CXX)

if(MSVC)
    add_compile_definitions(NOMINMAX)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(DE3D_SOURCES
    src/Solver.cpp
    src/Deformer.cpp
    src/Collisions.cpp
    src/SoftBodyInstance.cpp
    src/RigidBodyInstance.cpp
    src/Registry.cpp 
    src/Error.cpp
)

# Use ${PROJECT_NAME} for consistency
add_library(${PROJECT_NAME} SHARED ${DE3D_SOURCES})

# Enforce Position Independent Code
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/public
)

if(MSVC)
    # /O2: Maximize speed
    # /arch:SSE4.2: Target the SSE4 instruction set (no AVX/AVX2/FMA).
    #               Covers the SSE4.1 intrinsics used in MathUtils.h.
    # /fp:precise: Precise floating point math (crucial for stability and IEEE-754 finite checks)
    # Note: /GL is removed because INTERPROCEDURAL_OPTIMIZATION handles it automatically.
    target_compile_options(${PROJECT_NAME} PRIVATE
        "$<$<CONFIG:Release>:/O2;/arch:SSE4.2;/fp:precise>"
    )
else()
    # Apply aggressive optimizations only to Release builds without breaking IEEE-754 checks
    target_compile_options(${PROJECT_NAME} PRIVATE
        "$<$<CONFIG:Release>:-O3;-fno-math-errno>"
        -msse4.2
        -Wall
        -Wextra
    )
endif()

# ---------------------------------------------------------
# Link Time Optimization (LTO) / Interprocedural Optimization.
# ---------------------------------------------------------
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported)
if(ipo_supported)
    set_property(TARGET ${PROJECT_NAME} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        $<$<CONFIG:Release>:/Zi>
    )

    target_link_options(${PROJECT_NAME} PRIVATE
        $<$<CONFIG:Release>:/DEBUG>
    )
endif()

message(STATUS "Release flags: ${CMAKE_CXX_FLAGS_RELEASE}")
cmake_minimum_required(VERSION 3.16)
project(DynamicEngine3D VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(DE3D_SOURCES
    src/Solver.cpp
    src/Deformer.cpp
    src/Collisions.cpp
    src/SoftBodyInstance.cpp
    src/RigidBodyInstance.cpp
)

# Use ${PROJECT_NAME} for consistency
add_library(${PROJECT_NAME} SHARED ${DE3D_SOURCES})

# Enforce Position Independent Code (Good for Unity .so / .dylib targets)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(MSVC)
    # /O2: Maximize speed
    # /arch:AVX2: Enables Advanced Vector Extensions 2 (includes SSE4.1)
    # /fp:fast: Fast floating point math (crucial for physics)
    target_compile_options(${PROJECT_NAME} PRIVATE
        /O2 
        /arch:AVX2 
        /fp:fast 
        /wd4244 
    )
else()
    # -O3: Aggressive optimizations
    # -mavx2 -mfma: Enable AVX2 and Fused Multiply-Add
    # -ffast-math: Breaks IEEE strictness for performance
    target_compile_options(${PROJECT_NAME} PRIVATE
        -O3 
        -mavx2 
        -mfma 
        -fno-math-errno
-ffinite-loops
-fassociative-math
        -Wall 
        -Wextra
    )
endif()

# ---------------------------------------------------------
# Link Time Optimization (LTO) / Interprocedural Optimization
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
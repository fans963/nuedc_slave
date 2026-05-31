# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world LANGUAGES C CXX)

find_program(FLATC_EXECUTABLE flatc REQUIRED)

set(GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})

file(GLOB FBS_SCHEMAS schemas/*.fbs)
set(GENERATED_HEADERS)
foreach(fbs ${FBS_SCHEMAS})
    get_filename_component(name ${fbs} NAME_WE)
    list(APPEND GENERATED_HEADERS ${GENERATED_DIR}/${name}_generated.h)
endforeach()

add_custom_command(
    OUTPUT ${GENERATED_HEADERS}
    COMMAND ${FLATC_EXECUTABLE} --cpp --gen-mutable --scoped-enums
            -o ${GENERATED_DIR}
            ${FBS_SCHEMAS}
    DEPENDS ${FBS_SCHEMAS}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating FlatBuffers headers"
)
add_custom_target(generate_fbs_headers DEPENDS ${GENERATED_HEADERS})
add_dependencies(app generate_fbs_headers)

file(GLOB_RECURSE app_sources src/*.cpp src/*.c)
target_sources(app PRIVATE ${app_sources})
target_include_directories(app PRIVATE
    src
    ${CMAKE_CURRENT_SOURCE_DIR}/../deps/fmt/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../deps/flatbuffers/include
    ${GENERATED_DIR}
)

# fmt header-only mode — 只编译用到的函数
target_compile_definitions(app PRIVATE
    FMT_HEADER_ONLY
    FMT_OPTIMIZE_SIZE=2
    FMT_USE_FULL_CACHE_DRAGONBOX=0
)

# Additional aggressive compiler flags
target_compile_options(app PRIVATE
    -fomit-frame-pointer
    -ffast-math
    -fno-math-errno
    -fno-trapping-math
    -funroll-loops
    -falign-functions=4
    -falign-loops=4
)
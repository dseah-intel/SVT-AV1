#
# Copyright(c) 2018 Intel Corporation
# SPDX - License - Identifier: BSD - 2 - Clause - Patent
#

cmake_minimum_required(VERSION 3.5)
set(project_name "svt-av1")

project(${project_name} C CXX ASM_NASM)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_STANDARD 99)
set(CAN_USE_ASSEMBLER TRUE)
set(CMAKE_NASM_LINK_EXECUTABLE "ld -o <TARGET> <OBJECTS>")
set(CMAKE_ASM_NASM_SOURCE_FILE_EXTENSIONS asm)


if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CMAKE_ASM_NASM_OBJECT_FORMAT macho64)
    set(CMAKE_ASM_NASM_FLAGS "-DUNIX64")
    set(CMAKE_MACOSX_RPATH 1)
    set(CMAKE_C_ARCHIVE_CREATE   "<CMAKE_AR> Scr <TARGET> <LINK_FLAGS> <OBJECTS>")
    set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> Scr <TARGET> <LINK_FLAGS> <OBJECTS>")
    set(CMAKE_C_ARCHIVE_FINISH   "<CMAKE_RANLIB> -no_warning_for_no_symbols -c <TARGET>")
    set(CMAKE_CXX_ARCHIVE_FINISH "<CMAKE_RANLIB> -no_warning_for_no_symbols -c <TARGET>")
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_C_FLAGS "-Wall -Wextra -Wformat -Wformat-security -mavx -fstack-protector-strong -fPIE -fPIC -D_FORTIFY_SOURCE=2 ")
    set(CMAKE_C_FLAGS_RELEASE "-O2")
    set(CMAKE_C_FLAGS_DEBUG "-O0 -g")
	SET(CMAKE_EXE_LINKER_FLAGS "-z noexecstack -z relro -z now -pie ")
	SET(CMAKE_SHARED_LINKER_FLAGS "-z noexecstack -z relro -z now -pie ")
    set(CMAKE_ASM_NASM_FLAGS "-DUNIX64")
    set(CMAKE_ASM_NASM_FLAGS_DEBUG "-g null")
    set(CMAKE_ASM_NASM_OBJECT_FORMAT elf64)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CMAKE_EXE_LINKER_FLAGS " ")
    set(CMAKE_SHARED_LINKER_FLAGS " ")
    set(CMAKE_C_FLAGS_RELEASE "-O2")
    set(CMAKE_ASM_NASM_FLAGS "-DWIN64")
    set(CMAKE_ASM_NASM_FLAGS_DEBUG "")
    if (MSVC)
        set(CMAKE_C_FLAGS " /GS /sdl ")
        set(CMAKE_CXX_FLAGS " /GS /sdl ")
        set(CMAKE_EXE_LINKER_FLAGS " /NXCompat /DynamicBase ")
        set(CMAKE_SHARED_LINKER_FLAGS " /NXCompat /DynamicBase ")
    endif()
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fstack-protector-strong -fPIE -fPIC -D_FORTIFY_SOURCE=2 -O2 -flto -Wall -Wextra -Wformat -Wformat-security -fvisibility=hidden -march=native")
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows") #for Mingw64 support, see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65782
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-asynchronous-unwind-tables")
    endif()
    set(CMAKE_C_FLAGS_RELEASE "-O2")
    set(CMAKE_C_FLAGS_DEBUG "-O0 -g")
endif(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")

include(CTest)

set(COVERAGE false CACHE BOOL "For use with coveralls and GTest")

# Prepare for Coveralls
if(COVERAGE)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} --coverage")
        set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --coverage")
    endif()
endif()

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)
#if(USE_CODE_COVERAGE AND CMAKE_COMPILER_IS_GNUCXX)
#    include(CodeCoverage)
#    setup_target_for_coverage(${PROJECT_NAME}_coverage ${PROJECT_TEST_NAME} coverage)
#endif()

# Add Subdirectories
add_subdirectory (Source/Lib/Common)
add_subdirectory (Source/Lib/Encoder)
add_subdirectory (Source/App/EncApp)
add_subdirectory (Source/Lib/Decoder)
add_subdirectory (Source/App/DecApp)
if (BUILD_TESTING)
    add_subdirectory (test)
    add_subdirectory (third_party/googletest)
endif()

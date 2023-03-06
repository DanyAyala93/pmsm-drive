cmake_minimum_required(VERSION 3.13)

# update CMake module directory list
list(APPEND CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_LIST_DIR}/proj/cmake"
)

project(pas-pmsm-drive
    VERSION 0.1
    DESCRIPTION "This project defines the pas-pmsm-drive project"
    LANGUAGES
        C
        ASM
)

# Initialize properties on all targets
set(CMAKE_C_STANDARD_REQUIRED OFF)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_C_OUTPUT_EXTENSION_REPLACE 1)
set(CMAKE_C_OUTPUT_EXTENSION ".o")

include(CTest)

add_subdirectory(firmware/src)

# Generic is for cross compilation target, so add application.
if(${CMAKE_SYSTEM_NAME} STREQUAL "Generic")

else()
    add_subdirectory(firmware/tests)
endif()

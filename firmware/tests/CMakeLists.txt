cmake_minimum_required(VERSION 3.13)

project(Unit_Tests
    VERSION 0.1
    DESCRIPTION "This project defines the unit testing library using Catch2."
    LANGUAGES
        C
        CXX
)

# Finds and loads settings from the external project Catch2 (needs to be installed)
# REQUIRED - stops processing with an error message if not found
find_package(Catch2 REQUIRED)

add_library(${PROJECT_NAME}
    ../src/mc_app.c
    ../src/adc.c
    ../src/hall.c
    ../src/board_id.c
    ../src/fault_handler.c
    main.cpp
    )


target_compile_definitions( ${PROJECT_NAME} PUBLIC UNIT_TEST)


target_link_libraries(${PROJECT_NAME}
    PUBLIC
        PmsmDriveApplication_i
        Catch2::Catch2
        pthread
)

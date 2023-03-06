cmake_minimum_required(VERSION 3.13)

project(PmsmDriveApplication
    VERSION 0.1
    DESCRIPTION "This project defines the PMSM Drive target application."
    LANGUAGES
        C CXX
)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/packs/ATSAMD51J18A_DFP
    ${CMAKE_CURRENT_SOURCE_DIR}/packs/CMSIS/CMSIS/Core/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/packs/CMSIS/
)


if(${CMAKE_SYSTEM_NAME} STREQUAL "Generic")
    include(linker-script)
    include(map-file)
    include(compile-definitions)

    add_subdirectory(config/cgi_proto)
    
    add_executable(${PROJECT_NAME}.elf
        dummy.c # TODO - find the reason why the first source file is not linked. Added dummy file
        mc_app.c
        mc_Lib.c
        PowerControl.c
        BatteryCharger.c
        thermistors.c
        adc.c
        fault_handler.c
        hall.c
        main.c
        thermistors.c
	    board_id.c
    )


    target_include_directories(${PROJECT_NAME}.elf
        PRIVATE 
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/packs/ATSAMD51J18A_DFP
            ${CMAKE_CURRENT_SOURCE_DIR}/packs/CMSIS/CMSIS/Core/Include
            ${CMAKE_CURRENT_SOURCE_DIR}/packs/CMSIS/
)


    #Setting memory map so the application will work with the bootloader.
    target_linker_script(${PROJECT_NAME}.elf
        ${CMAKE_CURRENT_SOURCE_DIR}/config/cgi_proto/ATSAMD51J18A.ld
    )

    target_map_file(${PROJECT_NAME}.elf)

    add_compile_definitions(${PROJECT_NAME}.elf)

    target_link_libraries(${PROJECT_NAME}.elf
        PUBLIC
            Mplab_Cgi_Proto_Lib
            ${CMAKE_CURRENT_SOURCE_DIR}/libCORTEXM4_SAMD51_X2CScope.a
            ${CMAKE_CURRENT_SOURCE_DIR}/config/cgi_proto/X2CCode/X2CScope/lib/libCORTEXM4_X2CScope.a
    )

    target_link_options( ${PROJECT_NAME}.elf
        PRIVATE
            "LINKER:--gc-sections"
            "LINKER:--defsym=__MPLAB_BUILD=1"
            "LINKER:--defsym=_min_heap_size=512"
            "LINKER:--memorysummary"
    )

    set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <LINK_FLAGS> <OBJECTS>  -o <TARGET> <LINK_LIBRARIES>")

    add_custom_command(TARGET ${PROJECT_NAME}.elf
        POST_BUILD
            COMMAND xc32-size ${PROJECT_NAME}.elf
            COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
            COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    )

elseif(BUILD_TESTING AND (NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Generic"))

    # Build Saturn tests
    include(cgi-test)
    include(coverage)

    
    string(JOIN " " CMAKE_EXE_LINKER_FLAGS
      -Wl,--whole-archive  # proper static linkage of pthread lib
      -lpthread
      -Wl,--no-whole-archive
      )

    add_cgi_test( ${PROJECT_NAME}_Tests
        tests/temp.cpp
    )

    add_library(${PROJECT_NAME}_i INTERFACE)

    target_include_directories( ${PROJECT_NAME}_i
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/packs/ATSAMD51J18A_DFP
            ${CMAKE_CURRENT_SOURCE_DIR}/packs/CMSIS/CMSIS/Core/Include
            ${CMAKE_CURRENT_SOURCE_DIR}/packs/CMSIS/
            ${CMAKE_CURRENT_SOURCE_DIR}/config/cgi_proto
            ${CMAKE_CURRENT_SOURCE_DIR}/config/cgi_proto/X2CCode
)

    target_include_directories( ${PROJECT_NAME}_Tests
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/config/cgi_proto
            ${CMAKE_CURRENT_SOURCE_DIR}/config/cgi_proto/X2CCode
    )

    add_coverage( ${PROJECT_NAME}_Tests )

endif()

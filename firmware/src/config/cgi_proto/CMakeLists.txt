cmake_minimum_required( VERSION 3.13 )

project( Mplab_Cgi_Proto_Lib
         VERSION 0.1
         DESCRIPTION "This project defines the MPLAB support library."
         LANGUAGES ASM C CXX )

##############################################################################
add_library( ${PROJECT_NAME}
    peripheral/adc/plib_adc0.c
    peripheral/adc/plib_adc1.c
    peripheral/clock/plib_clock.c
    peripheral/cmcc/plib_cmcc.c
    peripheral/eic/plib_eic.c
    peripheral/evsys/plib_evsys.c
    peripheral/nvic/plib_nvic.c
    peripheral/nvmctrl/plib_nvmctrl.c
    peripheral/pdec/plib_pdec.c
    peripheral/pm/plib_pm.c
    peripheral/port/plib_port.c
    peripheral/sercom/usart/plib_sercom2_usart.c
    peripheral/sercom/usart/plib_sercom5_usart.c
    peripheral/tc/plib_tc0.c
    peripheral/tc/plib_tc2.c
    peripheral/tc/plib_tc4.c
    peripheral/tcc/plib_tcc1.c
    peripheral/tcc/plib_tcc0.c
    peripheral/wdt/plib_wdt.c
    peripheral/sercom/usart/plib_sercom3_usart.c
    stdio/xc32_monitor.c
    system/cache/sys_cache.c
    X2CCode/X2CScope/src/X2CScope.c
    X2CCode/X2CScope/src/X2CScopeCommunication.c
    initialization.c
    interrupts.c
    exceptions.c
    startup_xc32.c
    libc_syscalls.c
)

include(compile-definitions)

add_compile_definitions( ${PROJECT_NAME} )

target_include_directories( ${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/X2CCode
)

# Remove checks from third party code.
get_property( SUB_TARGETS DIRECTORY "${_dir}" PROPERTY BUILDSYSTEM_TARGETS )
foreach( LOCAL_TARGET ${SUB_TARGETS} )
    set_property( TARGET ${LOCAL_TARGET} PROPERTY C_CPPCHECK "" )
    set_property( TARGET ${LOCAL_TARGET} PROPERTY C_CPPLINT "" )
    set_property( TARGET ${LOCAL_TARGET} PROPERTY C_INCLUDE_WHAT_YOU_USE "" )
    set_property( TARGET ${LOCAL_TARGET} PROPERTY LINK_WHAT_YOU_USE FALSE )
endforeach()

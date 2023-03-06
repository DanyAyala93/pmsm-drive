find_program( LLVM_PROFDATA_BIN NAMES llvm-profdata )
find_program( LLVM_COV_BIN NAMES llvm-cov )

function(add_coverage target)

    if( LLVM_PROFDATA_BIN AND LLVM_COV_BIN )
        add_custom_target( ${target}_Ccov_Preprocessing ALL
            COMMAND LLVM_PROFILE_FILE=${target}.profraw $<TARGET_FILE:${target}>
            COMMAND ${LLVM_PROFDATA_BIN} merge ${target}.profraw --output ${target}.profdata
            DEPENDS ${target} )


        add_custom_target( ${target}_Ccov ALL
            COMMAND ${LLVM_COV_BIN} show -j=0 $<TARGET_FILE:${target}> -instr-profile=${target}.profdata -use-color=false -output-dir=${target}_llvm_cov
            DEPENDS ${target}_Ccov_Preprocessing )

    endif()
endfunction()
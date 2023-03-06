function(add_compile_definitions target)
    target_compile_definitions( ${target}
        PRIVATE
            XPRJ_cgi_proto=cgi_proto
    )
endfunction()
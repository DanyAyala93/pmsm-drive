function(target_map_file target)
    target_link_options(${target}
        PRIVATE
            "LINKER:-Map,$<TARGET_FILE:${target}>.map"
            )
endfunction()

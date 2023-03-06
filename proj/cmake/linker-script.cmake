function(target_linker_script target script)
    # We need an absolute path for the script.
    get_filename_component(script "${script}" ABSOLUTE)
    # Pass the script to the linker.
    target_link_options(${target} PRIVATE LINKER:--script="${script}")
    # Relink if the script changes.
    set_target_properties(${target} PROPERTIES LINK_DEPENDS "${script}")
endfunction()

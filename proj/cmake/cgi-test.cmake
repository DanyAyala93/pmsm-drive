# Creates an executable an discovers all Catch2 tests
# The source files should be passed as arguments after the target in the function call
function(add_cgi_test target)

    # Finds and loads settings from the external project Catch2 (needs to be installed)
    # REQUIRED - stops processing with an error message if found
    find_package(Catch2 REQUIRED)

    # ${ARGN} holds the list of arguments past the last expected argument.
    add_executable(${target} ${ARGN})

    target_link_libraries(${target}
        PRIVATE
	    Unit_Tests
    )

    include(Catch) # CMake script provided by Catch2 which registers each TEST_CASE with CTest

    catch_discover_tests(${target}) # Discovers all tests from target

endfunction()

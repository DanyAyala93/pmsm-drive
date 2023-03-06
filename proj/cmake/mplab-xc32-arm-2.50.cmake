set(CMAKE_SYSTEM_NAME Generic)   # "Generic" means bare metal
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Define the compilers
set(CMAKE_C_COMPILER xc32-gcc)
set(CMAKE_CXX_COMPILER xc32-g++)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})

# Disable compiler checks.
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

string( JOIN " " ARCH_FLAGS
    -mprocessor=ATSAMD51J18A
)

string(JOIN " " WARNING_FLAGS
    #-pedantic              # disable compiler extensions
    #-Werror                # promote warnings to errors
    -Wall                  # standard warnings
    #-Wextra                # more warnings
    #-Wtrigraphs
    #-Wparentheses          # missing parentheses
    #-Wcast-align           # pointer cast increases alignment
    #-Wcast-qual            # casting away a type qualifier
    #-Wuninitialized        # uninitialized local variable
    #-Wsign-compare         # comparison between signed and unsigned values
    #-Wsign-conversion      # conversions that may change the sign of a value
    #-Wshadow               # symbol name shadows another symbol
    #-Wmissing-declarations # global function defined without previous declaration

)

# Initialize assembly flags
string(JOIN " " CMAKE_ASM_FLAGS_INIT
    ${ARCH_FLAGS}
    #-fno-common            # Don't place undefined global variables
)

# Initialize C flags
string(JOIN " " CMAKE_C_FLAGS_INIT
    -g
    -x c
    -ffunction-sections     # Place each function into its own section
    -fdata-sections         # Place each data item into its own section
    -O1
    ${ARCH_FLAGS}
    ${WARNING_FLAGS}
)

# Initialize linker flags
string(JOIN " " CMAKE_EXE_LINKER_FLAGS_INIT
    ${ARCH_FLAGS}
    -mno-device-startup-code
    -DXPRJ_cgi_proto=cgi_proto
    #--specs=nano.specs                              # Link small standard library
)

set(CMAKE_FIND_ROOT_PATH /opt/microchip/xc32/v2.50)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
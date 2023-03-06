set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")

string(JOIN " " CMAKE_C_FLAGS_INIT
    -std=c99
    -Wall
    -fprofile-instr-generate
    -fcoverage-mapping
)

string(JOIN " " CMAKE_CXX_FLAGS_INIT
    -std=c++11
    -Wall
    -fprofile-instr-generate
    -fcoverage-mapping
)

string(JOIN " " CMAKE_EXE_LINKER_FLAGS_INIT
    -Wl,--gc-sections   # Enable garbage collection of unused input sections
    --coverage
)

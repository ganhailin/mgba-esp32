add_compile_flags(LD
        -mlongcalls
        -nostartfiles
        -static
        -Wl,--gc-sections
        -Wl,-static
        -Wl,--start-group
        -Wl,--whole-archive
        -Wl,--no-whole-archive
        -Wl,--end-group
        -Wl,-EL
        -Wl,--no-relax
        )

# C Flags Settings
add_compile_flags(BOTH
        -mlongcalls
        -mfix-esp32-psram-cache-issue
        -fno-common
        -ffunction-sections
        -fdata-sections
        -fstrict-volatile-bitfields
        -fno-zero-initialized-in-bss
        -Os
        -g
        )

add_compile_flags(C -std=gnu11 -Wno-pointer-to-int-cast)
add_compile_flags(CXX -std=gnu++17)



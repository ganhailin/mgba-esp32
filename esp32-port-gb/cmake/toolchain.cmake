if (WIN32)
    set(EXT ".exe")
else ()
    set(EXT "")
endif ()

message(STATUS "Check for ESP32 toolchain ...")
if(NOT TOOLCHAIN)
    find_path(_TOOLCHAIN xtensa-esp32-elf-gcc${EXT})
    global_set(TOOLCHAIN "${_TOOLCHAIN}")
elseif(NOT "${TOOLCHAIN}" MATCHES "/$")
    global_set(TOOLCHAIN "${TOOLCHAIN}")
endif()

if (NOT TOOLCHAIN)
    message(FATAL_ERROR "TOOLCHAIN must be set, to absolute path of esp32-toolchain dist/bin folder.")
endif ()

message(STATUS "Using ${TOOLCHAIN} ESP32 toolchain")

global_set(CMAKE_C_COMPILER "${TOOLCHAIN}/xtensa-esp32-elf-gcc${EXT}")
global_set(CMAKE_CXX_COMPILER "${TOOLCHAIN}/xtensa-esp32-elf-g++${EXT}")
global_set(CMAKE_LINKER "${TOOLCHAIN}/xtensa-esp32-elf-ld${EXT}")
global_set(CMAKE_AR "${TOOLCHAIN}/xtensa-esp32-elf-ar${EXT}")
global_set(CMAKE_OBJCOPY "${TOOLCHAIN}/xtensa-esp32-elf-objcopy${EXT}")
global_set(CMAKE_SIZE "${TOOLCHAIN}/xtensa-esp32-elf-size${EXT}")
global_set(CMAKE_OBJDUMP "${TOOLCHAIN}/xtensa-esp32-elf-objdump${EXT}")
if (WIN32)
    global_set(CMAKE_MAKE_PROGRAM "${TOOLCHAIN}/mingw32-make${EXT}")
endif ()

execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=crt0.o OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE CRT0_OBJ)
execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=crtbegin.o OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE CRTBEGIN_OBJ)
execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=crtend.o OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE CRTEND_OBJ)
execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=crti.o OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE CRTI_OBJ)
execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=crtn.o OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE CRTN_OBJ)

global_set(CMAKE_C_LINK_EXECUTABLE
        "<CMAKE_C_COMPILER>  <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> \"${CRTI_OBJ}\" \"${CRTBEGIN_OBJ}\" <OBJECTS> \"${CRTEND_OBJ}\" \"${CRTN_OBJ}\" -o <TARGET> <LINK_LIBRARIES>")

global_set(CMAKE_CXX_LINK_EXECUTABLE
        "<CMAKE_CXX_COMPILER>  <FLAGS> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> \"${CRTI_OBJ}\" \"${CRTBEGIN_OBJ}\" <OBJECTS> \"${CRTEND_OBJ}\" \"${CRTN_OBJ}\" -o <TARGET> <LINK_LIBRARIES>")

get_filename_component(_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
if (NOT "${TOOLCHAIN}" STREQUAL "${_BIN_DIR}")
    message(FATAL_ERROR "CMAKE_C_COMPILER ${CMAKE_C_COMPILER} is not in kendryte-toolchain dist/bin folder.")
endif ()

set(MGBA_DIR "/home/tinylib/mgba-k210")
set(COMPONENT_SRCS "hello_world_main.c")
set(COMPONENT_ADD_INCLUDEDIRS ${MGBA_DIR}/include)
include_directories(${MGBA_DIR}/include)


register_component()
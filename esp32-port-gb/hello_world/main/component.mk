#
# "main" pseudo-component makefile.
#-DMINIMAL_CORE=ON
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)
MGBA_BUILD_DIR=/home/tinylib/mgba-k210/build
COMPONENT_ADD_LDFLAGS :=${MGBA_BUILD_DIR}/libmgba.a build/main/libmain.a
MGBA_DIR =/home/tinylib/mgba-k210
COMPONENT_ADD_INCLUDEDIRS :=${MGBA_DIR}/include
CPPFLAGS +=-DK210 -DM_CORE_GBA -DM_CORE_GB -DENABLE_SCRIPTING
# How to build esp32-port

Step 1: build libmgba.a
```
mkdir build
cd build
cmake -DTOOLCHAIN=/opt/kendryte-toolchain/bin -DUSE_MINIZIP=OFF -DUSE_ZLIB=OFF -DUSE_LZMA=OFF ..
make -j8
```


Step 2: build mgba

```
setup ESP-IDF 3.x and tool
cd esp-port/helloworld
make flash monitor
```

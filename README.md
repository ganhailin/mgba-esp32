# How to build k210-port

Step 1: build libmgba.a
```
mkdir build
cd build
cmake -DTOOLCHAIN=/opt/kendryte-toolchain/bin -DUSE_MINIZIP=OFF -DUSE_ZLIB=OFF -DUSE_LZMA=OFF ..
make -j8
```


Step 2: build mgba.bin
```
cd k210-port
mkdir build
cd build
cmake -DTOOLCHAIN=/opt/kendryte-toolchain/bin -DPROJ=mgba ..
make -j8
```

Step 3: flash to board
```
Linux:
python(3) kflash.py -b 921600 -p /dev/ttyUSBX -B goE -t YOUR_REPO_PATH/k210-port/build/mgba.bin

Windows:
python kflash.py -b 921600 -p COMXX -B goE -t YOUR_REPO_PATH/k210-port/build/mgba.bin
```

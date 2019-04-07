# Step 1: build libmgba.a
mkdir build
cd build
cmake -DTOOLCHAIN=/opt/kendryte-toolchain/bin -DUSE_MINIZIP=OFF -DUSE_ZLIB=OFF -DUSE_LZMA=OFF ..
make -j8

# Step 2: build mgba.bin
cd k210-port
mkdir build
cd build
cmake -DTOOLCHAIN=/opt/kendryte-toolchain/bin -DPROJ=mgba ..
make -j8

# Step 3: flash to board
python(3) kflash.py -b 921600 -p /dev/ttyUSBXXX -B goE -t YOUR_REPO_PATH/k210-port/build/mgba.bin

python kflash.py -b 921600 -p COM15 -B goE -t mgba/k210-port/build/mgba.bin
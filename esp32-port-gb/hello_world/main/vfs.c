/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"


#include <mgba/core/core.h>
#include <mgba/gba/core.h>
#include <mgba/feature/commandline.h>
#include <mgba/core/blip_buf.h>

#include <mgba-util/memory.h>
#include <mgba-util/string.h>
#include <mgba-util/vfs.h>
#include <mgba/internal/gba/gba.h>
#include "ff.h"

#include "smmu.h"

#define TRACE_1 printf
typedef uint8_t u8;

struct VDir* VDirOpen(const char* path) {
	return 0;
}
struct VFileESP {
	struct VFile d;

	FIL fileObj;

};


static bool _vfESPClose(struct VFile* vf);
static off_t _vfESPSeek(struct VFile* vf, off_t offset, int whence);
static ssize_t _vfESPRead(struct VFile* vf, void* buffer, size_t size);
static ssize_t _vfESPWrite(struct VFile* vf, const void* buffer, size_t size);
static void* _vfESPMap(struct VFile* vf, size_t size, int flags);
static void _vfESPUnmap(struct VFile* vf, void* memory, size_t size);
static void _vfESPTruncate(struct VFile* vf, size_t size);
static ssize_t _vfESPSize(struct VFile* vf);
static bool _vfESPSync(struct VFile* vf, const void* buffer, size_t size);

extern uint32_t cacheBuffer[];

struct VFile* VFileOpenESPFatFs(const char* path, u8 mode) {
	TRACE_1("VFileOpenESPFatFs: %p, %s\r\n", __builtin_return_address(0), path);

	struct VFileESP* vfESP = malloc(sizeof(struct VFileESP));
	if (!vfESP) {
		return 0;
	}
	FRESULT ret = f_open(&(vfESP->fileObj), path, mode) ;
	if (ret != FR_OK) {
		printf("VFileOpenESPFatFs: fopen failed, %d\n", ret);
		free(vfESP);
		return 0;
	}

	vfESP->d.close = _vfESPClose;
	vfESP->d.seek = _vfESPSeek;
	vfESP->d.read = _vfESPRead;
	vfESP->d.readline = VFileReadline;
	vfESP->d.write = _vfESPWrite;
	vfESP->d.map = _vfESPMap;
	vfESP->d.unmap = _vfESPUnmap;
	vfESP->d.truncate = _vfESPTruncate;
	vfESP->d.size = _vfESPSize;
	vfESP->d.sync = _vfESPSync;
	return &vfESP->d;
}

bool _vfESPClose(struct VFile* vf) {
	free(vf);
	return true;
}

off_t _vfESPSeek(struct VFile* vf, off_t offset, int whence) {
	assert(whence == SEEK_SET);

	struct VFileESP* vfESP = (struct VFileESP*) vf;
	f_lseek(&(vfESP->fileObj), offset);
	return offset;
}

ssize_t _vfESPRead(struct VFile* vf, void* buffer, size_t size) {
	TRACE_1("_vfESPRead: %p, %d\r\n", __builtin_return_address(0), (int) size);

	struct VFileESP* vfESP = (struct VFileESP*) vf;
	UINT bytesRead = 0;
	if (f_read(&(vfESP->fileObj), buffer, size, &bytesRead) != FR_OK) {
		return 0;
	}
	return bytesRead;
}

ssize_t _vfESPWrite(struct VFile* vf, const void* buffer, size_t size) {
	TRACE_1("_vfESPWrite: %p, %d\r\n", __builtin_return_address(0), (int) size);

	struct VFileESP* vfESP = (struct VFileESP*) vf;
	UINT bytesWritten = 0;
	if (f_write(&(vfESP->fileObj), buffer, size, &bytesWritten) != FR_OK) {
		return 0;
	}
	return bytesWritten;
}

int vfs_readcb(void* id,uint32_t addr,uint32_t size,void* buffer){
    struct VFileESP* vfESP = (struct VFileESP*) id;
	f_lseek(&(vfESP->fileObj), addr);
    UINT bytesRead = 0;
	if (f_read(&(vfESP->fileObj), buffer, size, &bytesRead) != FR_OK) {
		return 0;
	}
	return bytesRead;
}

static void* _vfESPMap(struct VFile* vf, size_t size, int flags) {
	TRACE_1("_vfESPMap: %p, %d,flag:%d\r\n", __builtin_return_address(0), (int) size,flags);
	struct VFileESP* vfESP = (struct VFileESP*) vf;
    ext_read32=smmu_read32;
    ext_read16=smmu_read16;
    ext_read8=smmu_read8;
	return smmu_map(vf,vfs_readcb,NULL,size,flags);
}

static void _vfESPUnmap(struct VFile* vf, void* memory, size_t size) {}

static void _vfESPTruncate(struct VFile* vf, size_t size) {}

ssize_t _vfESPSize(struct VFile* vf) {
	struct VFileESP* vfESP = (struct VFileESP*) vf;

	return f_size(&(vfESP->fileObj));
}

static bool _vfESPSync(struct VFile* vf, const void* buffer, size_t size) {
	return true;
}



char *getcwd(char *buf, size_t size)
{
FRESULT result = f_getcwd(buf, size);
return result == FR_OK ? buf : NULL;
}
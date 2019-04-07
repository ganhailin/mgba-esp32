#include <k9.h>

#include <mgba/core/core.h>
#include <mgba/gba/core.h>
#include <mgba/feature/commandline.h>
#include <mgba/core/blip_buf.h>

#include <mgba-util/memory.h>
#include <mgba-util/string.h>
#include <mgba-util/vfs.h>
#include <mgba/internal/gba/gba.h>

#define TRACE_1 printk

struct VFileK210 {
	struct VFile d;

	FIL fileObj;

};


static bool _vfK210Close(struct VFile* vf);
static off_t _vfK210Seek(struct VFile* vf, off_t offset, int whence);
static ssize_t _vfK210Read(struct VFile* vf, void* buffer, size_t size);
static ssize_t _vfK210Write(struct VFile* vf, const void* buffer, size_t size);
static void* _vfK210Map(struct VFile* vf, size_t size, int flags);
static void _vfK210Unmap(struct VFile* vf, void* memory, size_t size);
static void _vfK210Truncate(struct VFile* vf, size_t size);
static ssize_t _vfK210Size(struct VFile* vf);
static bool _vfK210Sync(struct VFile* vf, const void* buffer, size_t size);

extern uint32_t cacheBuffer[];
struct VFile* VFileOpenK210FatFs(const char* path, u8 mode) {
	TRACE_1("VFileOpenK210FatFs: %p, %s\r\n", __builtin_return_address(0), path);

	struct VFileK210* vfK210 = malloc(sizeof(struct VFileK210));
	if (!vfK210) {
		return 0;
	}
	FRESULT ret = f_open(&(vfK210->fileObj), path, mode) ;
	if (ret != FR_OK) {
		printk("VFileOpenK210FatFs: fopen failed, %d\n", ret);
		free(vfK210);
		return 0;
	}

	vfK210->d.close = _vfK210Close;
	vfK210->d.seek = _vfK210Seek;
	vfK210->d.read = _vfK210Read;
	vfK210->d.readline = VFileReadline;
	vfK210->d.write = _vfK210Write;
	vfK210->d.map = _vfK210Map;
	vfK210->d.unmap = _vfK210Unmap;
	vfK210->d.truncate = _vfK210Truncate;
	vfK210->d.size = _vfK210Size;
	vfK210->d.sync = _vfK210Sync;
	return &vfK210->d;
}

bool _vfK210Close(struct VFile* vf) {
	free(vf);
	return true;
}

off_t _vfK210Seek(struct VFile* vf, off_t offset, int whence) {
	assert(whence == SEEK_SET);

	struct VFileK210* vfK210 = (struct VFileK210*) vf;
	f_lseek(&(vfK210->fileObj), offset);
	return offset;
}

ssize_t _vfK210Read(struct VFile* vf, void* buffer, size_t size) {
	TRACE_1("_vfK210Read: %p, %d\r\n", __builtin_return_address(0), (int) size);

	struct VFileK210* vfK210 = (struct VFileK210*) vf;
	UINT bytesRead = 0;
	if (f_read(&(vfK210->fileObj), buffer, size, &bytesRead) != FR_OK) {
		return 0;
	}
	return bytesRead;
}

ssize_t _vfK210Write(struct VFile* vf, const void* buffer, size_t size) {
	TRACE_1("_vfK210Write: %p, %d\r\n", __builtin_return_address(0), (int) size);

	struct VFileK210* vfK210 = (struct VFileK210*) vf;
	UINT bytesWritten = 0;
	if (f_write(&(vfK210->fileObj), buffer, size, &bytesWritten) != FR_OK) {
		return 0;
	}
	return bytesWritten;
}

static void* _vfK210Map(struct VFile* vf, size_t size, int flags) {
	TRACE_1("_vfK210Map: %p, %d\r\n", __builtin_return_address(0), (int) size);
	struct VFileK210* vfK210 = (struct VFileK210*) vf;
	return ceMapFileFatFs(&(vfK210->fileObj));
}

static void _vfK210Unmap(struct VFile* vf, void* memory, size_t size) {}

static void _vfK210Truncate(struct VFile* vf, size_t size) {}

ssize_t _vfK210Size(struct VFile* vf) {
	struct VFileK210* vfK210 = (struct VFileK210*) vf;

	return f_size(&(vfK210->fileObj));
}

static bool _vfK210Sync(struct VFile* vf, const void* buffer, size_t size) {
	return true;
}

struct VDir* VDirOpen(const char* path) {
	return 0;
}

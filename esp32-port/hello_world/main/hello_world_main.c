/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#include "driver/i2s.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "rom/uart.h"
#include "sdmmc_cmd.h"

#include <mgba/core/blip_buf.h>
#include <mgba/core/core.h>
#include <mgba/feature/commandline.h>
#include <mgba/gb/core.h>
#include <mgba/gba/core.h>

#include "st7735.h"
#include "st7789.h"
#include <mgba-util/memory.h>
#include <mgba-util/string.h>
#include <mgba-util/vfs.h>
#include <mgba/internal/gb/gb.h>
#include <mgba/internal/gba/gba.h>

static const char* TAG = "mgba";

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

#define TRACE_1 printf
static int _logLevel = 0x0f;

#define AUDIO_SAMPLE_LEN (2048)
#define LCD_X_MAX (240)
#define LCD_Y_MAX (320)

#define __DEBUG__
#ifdef __DEBUG__
#define __DEBUG_LINEFUNC__ printf("%s:%d:%s\r\n", __FILE__, __LINE__, __func__)
#endif

// This example can use SDMMC and SPI peripherals to communicate with SD card.
// By default, SDMMC peripheral is used.
// To enable SPI mode, uncomment the following line:

//#define USE_SPI_MODE

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.

#ifdef USE_SPI_MODE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 4
#define PIN_NUM_CLK 14
#define PIN_NUM_CS 13
#endif // USE_SPI_MODE

#define MGBA_I2S_NUM (0)
#define MGBA_I2S_SAMPLE_RATE   (48000)
#define MGBA_I2S_SAMPLE_BITS   (16)
#define MGBA_I2S_FORMAT        (I2S_CHANNEL_FMT_RIGHT_LEFT)
#define MGBA_I2S_CHANNEL_NUM   ((MGBA_I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))


struct mCore* gbaCore;
struct mAVStream avStream;

uint64_t* videoOutputBuffer; //[240 * 160 / 4];
// u64 *lcdWorkBuffer;//[LCD_Y_MAX * LCD_X_MAX / 4];
int16_t audioBuffer[2][AUDIO_SAMPLE_LEN * 2];
volatile int currentAudioBuffer = 0;
volatile int audioBufferCount = 0;
int isPS2Connected = 0;
struct VFile* VFileOpenESPFatFs(const char* path, u8 mode);

static int on_irq_dma3(void* ctx) {
	/*if (audioBufferCount > 0) {
	    audioBufferCount --;
	    currentAudioBuffer = !currentAudioBuffer;
	}
	int16_t* samples = audioBuffer[currentAudioBuffer];
	i2s_play(I2S_DEVICE_0, DMAC_CHANNEL3, samples, sizeof(audioBuffer[0]), sizeof(audioBuffer[0]), 16, 2);
	*/
	return 0;
}

int initSDCard() {
	/*int ret = 0;
	for (int i = 0; i < 3; i++) {
	    ret = sd_init();
	    if (ret == 0) {
	        break;
	    }
	    printk("sdcard init failed, retry...\r\n");
	    msleep(500);
	}
	if (ret != 0) {
	    printk("sdcard init failed!\r\n");
	    return -1;
	}
	uint8_t buf[512] = { 0 };
	ret = sd_read_sector(buf, 0, 1);
	if (ret != 0) {
	    printk("sdcard test read sector failed!\r\n");
	    return -2;
	}
	for (int i = 0; i < 512; i++) {
	    printk("%02x ", buf[i]);
	}*/
	return 0;
}

int initAudio() {
	/*i2s_init(I2S_DEVICE_0, I2S_TRANSMITTER, 0x03); //mask of ch
	i2s_tx_channel_config(I2S_DEVICE_0, I2S_CHANNEL_0,
	                      RESOLUTION_16_BIT, SCLK_CYCLES_32,
	                       TRIGGER_LEVEL_4,//TRIGGER_LEVEL_1
	                      RIGHT_JUSTIFYING_MODE);
	printk("Real sample rate: %d\n", (int) i2s_set_sample_rate(I2S_DEVICE_0, 32768));

	dmac_set_irq(DMAC_CHANNEL3, on_irq_dma3, NULL, 1);
	on_irq_dma3(0);*/
	return 0;
}

static void _postAudioBuffer(struct mAVStream* stream, blip_t* left, blip_t* right) {
	//while (audioBufferCount >= 2) {
	//}
	/*sysctl_disable_irq();
	int16_t* samples = audioBuffer[!currentAudioBuffer];
	blip_read_samples(left, samples, AUDIO_SAMPLE_LEN, true);
	blip_read_samples(right, samples + 1, AUDIO_SAMPLE_LEN, true);
	audioBufferCount ++;
	sysctl_enable_irq();*/
	//while (audioBufferCount >= 2) {
	//}
	int16_t* samples = audioBuffer[currentAudioBuffer];
	blip_read_samples(left, samples, AUDIO_SAMPLE_LEN, true);
	blip_read_samples(right, samples + 1, AUDIO_SAMPLE_LEN, true);
	//currentAudioBuffer =!currentAudioBuffer;
	size_t bytes_written;
	int audiosample=AUDIO_SAMPLE_LEN;
	while(audiosample--)
	{
		int sampleL=samples[audiosample*2];
		int sampleR=samples[audiosample*2+1];
		samples[audiosample*2]=sampleL-32768;
		samples[audiosample*2+1]=sampleR-32768;
	}
	i2s_write(MGBA_I2S_NUM, samples, AUDIO_SAMPLE_LEN*4, &bytes_written, portMAX_DELAY);
	//printf("write:%ul;\r\n",bytes_written);
}

int mountFatFs() {
	/*FRESULT fret = f_mount(&sdFatFs, "", 1);
	if (fret == FR_OK) {
	    printk("mount fat partition ok!\r\n");
	    return 0;
	} else {
	    printk("mount fat partition failed: %d\n", fret);
	}
	return -1;*/
	return 0;
}

void _log(struct mLogger* log, int category, enum mLogLevel level, const char* format, va_list args) {
	if (level & _logLevel) {
		// Categories are registered at runtime, but the name can be found
		// through a simple lookup.
		printf("%s: ", mLogCategoryName(category));

		// We get a format string and a varargs context from the core, so we
		// need to use the v* version of printk.
		vprintf(format, args);

		// The format strings do NOT include a newline, so we need to
		// append it ourself.
		printf("\r\n");
	}
}

void initMgba() {
	__DEBUG_LINEFUNC__;
	//malloc(160 * 144 *2);//MALLOC_CAP_SPIRAM
	videoOutputBuffer = heap_caps_malloc(160 * 144 *2,MALLOC_CAP_DMA);
	printf("video buffer:%p\r\n",videoOutputBuffer);
	__DEBUG_LINEFUNC__;
	// lcdWorkBuffer=calloc(1,LCD_Y_MAX * LCD_X_MAX / 4*sizeof(u64));
	__DEBUG_LINEFUNC__;
	// Set up a logger. The default logger prints everything to STDOUT, which is not usually desirable.
	static struct mLogger logger = { .log = _log };
	mLogSetDefaultLogger(&logger);
	__DEBUG_LINEFUNC__;
	gbaCore = GBCoreCreate();
	printf("maxpath=%d", PATH_MAX);
	printf("gbacorepoint=%p\r\n", gbaCore);
	printf("gbacoreinit=%p\r\n", gbaCore->init);
	printf("gbacoreinit=%p\r\n", gbaCore->deinit);
	printf("offset=%d\r\n", ((uint32_t) & (gbaCore->init)) - (uint32_t) gbaCore);
	__DEBUG_LINEFUNC__;
	gbaCore->init(gbaCore);
	__DEBUG_LINEFUNC__;
	gbaCore->setAudioBufferSize(gbaCore, AUDIO_SAMPLE_LEN);
	__DEBUG_LINEFUNC__;
	avStream.postAudioBuffer = _postAudioBuffer;
	gbaCore->setAVStream(gbaCore, &avStream);
	__DEBUG_LINEFUNC__;

	blip_set_rates(gbaCore->getAudioChannel(gbaCore, 0), gbaCore->frequency(gbaCore), MGBA_I2S_SAMPLE_RATE);
	__DEBUG_LINEFUNC__;
	blip_set_rates(gbaCore->getAudioChannel(gbaCore, 1), gbaCore->frequency(gbaCore), MGBA_I2S_SAMPLE_RATE);
	__DEBUG_LINEFUNC__;
}

void blitFromGbaToLcd320240P2P(uint8_t* srcPtr) {
	// uint8_t* dstPtr  = lcdWorkBuffer;
	// //for(int i=0;i<160;i++)
	// //	memcpy(dstPtr+240*2*i,srcPtr+240*i*2,240*2);
	// for(int y=0;y<144;y++)
	// 	for(int x=0;x<160;x++)
	// 	{
	// 		dstPtr[(y*160+x)*2]=srcPtr[(y*160+x)*2+1];
	// 		dstPtr[(y*160+x)*2+1]=srcPtr[(y*160+x)*2];
	// 	}

	ST7789_Flush(0, 0, 160, 144, srcPtr);
	// ST7789_Flush(0,0,160,144,dstPtr);
}

void blitFromGbaToLcd320240Upscaled(u64* srcPtr) {
	/*u64* dstPtr  = lcdWorkBuffer;
	const u32 upscaledWidth = 240 + (240 / 3); // 320
	const u32 upscaledHeight = 160 + (160 / 3); // 213
	const u32 offX = (320 - upscaledWidth) / 2;
	const u32 offY = (240 - upscaledHeight) / 2;

	lcd_set_area(offX, offY, offX + upscaledWidth - 1, offY + upscaledHeight - 1);
	for (u32 y = 0; y < 160; y++) {
	    for (u32 x = 0; x < 240; x += 12) {
	        u64 src4Pix = *srcPtr;
	        srcPtr++;
	        u16 pix0 = (u16) src4Pix;
	        u16 pix1 = (u16)(src4Pix >> 16);
	        u16 pix2 = (u16)(src4Pix >> 32);
	        u16 pix3 = (u16)(src4Pix >> 48);

	        src4Pix = *srcPtr;
	        srcPtr++;
	        u16 pix4 = (u16) src4Pix;
	        u16 pix5 = (u16)(src4Pix >> 16);
	        u16 pix6 = (u16)(src4Pix >> 32);
	        u16 pix7 = (u16)(src4Pix >> 48);

	        src4Pix = *srcPtr;
	        srcPtr++;
	        u16 pix8 = (u16) src4Pix;
	        u16 pix9 = (u16)(src4Pix >> 16);
	        u16 pix10 = (u16)(src4Pix >> 32);
	        u16 pix11 = (u16)(src4Pix >> 48);

	        *dstPtr = ((u64) pix0 << 16) | ((u64) pix1) | ((u64) pix2 << 48) | ((u64) pix2 << 32);
	        dstPtr++;
	        *dstPtr = ((u64) pix3 << 16) | ((u64) pix4) | ((u64) pix5 << 48) | ((u64) pix5 << 32);
	        dstPtr++;
	        *dstPtr = ((u64) pix6 << 16) | ((u64) pix7) | ((u64) pix8 << 48) | ((u64) pix8 << 32);
	        dstPtr++;
	        *dstPtr = ((u64) pix9 << 16) | ((u64) pix10) | ((u64) pix11 << 48) | ((u64) pix11 << 32);
	        dstPtr++;
	    }
	    if (y % 3 == 2) {
	        memcpy(dstPtr, dstPtr - (upscaledWidth / 4), upscaledWidth * 2);
	        dstPtr += (upscaledWidth / 4);
	    }
	}
	tft_write_word(lcdWorkBuffer, upscaledWidth * upscaledHeight / 2);*/
}

u16 translateKey(int ch) {
	enum GBAKey {
		GBA_KEY_A = 0,
		GBA_KEY_B = 1,
		GBA_KEY_SELECT = 2,
		GBA_KEY_START = 3,
		GBA_KEY_RIGHT = 4,
		GBA_KEY_LEFT = 5,
		GBA_KEY_UP = 6,
		GBA_KEY_DOWN = 7,
		GBA_KEY_R = 8,
		GBA_KEY_L = 9,
		GBA_KEY_MAX,
		GBA_KEY_NONE = -1
	};

	switch (ch) {
	case 'w':
		return 1 << 6;
	case 's':
		return 1 << 7;
	case 'a':
		return 1 << 5;
	case 'd':
		return 1 << 4;
	case 'z':
		return 1 << 0;
	case 'x':
		return 1 << 1;
	case 'q':
		return 1 << 9;
	case 'e':
		return 1 << 8;
	case '1':
		return 1 << 2;
	case '2':
		return 1 << 3;
	}
	return 0;
}
volatile int keyson[8];
const int keypins[8] = { 0, 5, 23, 27, 34, 35, 32, 33 };
void keyscan(void* __p) {
	for (int i = 0; i < 8; i++) {
		gpio_set_direction(keypins[i], GPIO_MODE_INPUT);
		gpio_pullup_en(keypins[i]);
	}
	while (true) {
		for (int i = 0; i < 8; i++) {
			keyson[i] = gpio_get_level(keypins[i]);
		}
		if (!(keyson[0] + keyson[1] + keyson[2] + keyson[3])) {
			struct GB* gb = gbaCore->board;
			printf("sramsize:%d,sramp:%p\r\n", gb->sramSize, gb->memory.sram);
			FILE* fp = fopen("/sdcard/testgb.save", "wb");
			if (fp) {
				fwrite(gb->memory.sram, gb->sramSize, 1, fp);
				fclose(fp);
			}
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
volatile int isneedflash=0;
void screenflash(void* __p) {
	while (true) {
		if(isneedflash)
		{
			ST7789_Flush(0, 0, 160, 144, videoOutputBuffer);
			isneedflash=0;
		}
		vTaskDelay(1);
	}
}

void mgbaMainLoop(void* __p) {
	struct mCoreOptions opts = {
		.useBios = true,
		.volume = 0x040,
	};

	const uint32_t desiredFps = 60;
	const uint64_t usPerFrame = 1000000UL / desiredFps;
	initMgba();

	struct VFile* romFile = VFileOpenESPFatFs("test.gb", FA_READ);
	// struct VFile* savFile = VFileOpenESPFatFs("boot.sav", FA_WRITE | FA_OPEN_ALWAYS);
	__DEBUG_LINEFUNC__;
	struct mCore* core = gbaCore;
	core->setVideoBuffer(core, (color_t*) videoOutputBuffer, 160);
	mCoreConfigInit(&core->config, "ESP32");
	mCoreConfigLoad(&core->config);
	mCoreConfigSetDefaultValue(&core->config, "idleOptimization", "remove");//remove//detect//ignore
	mCoreConfigSetDefaultIntValue(&core->config, "sgb.borders", 0);
	//mCoreConfigSetDefaultIntValue(&core->config, "mute",0);
	//mCoreConfigSetDefaultUIntValue(&core->config, "frameskip",5);
	// mCoreConfigSetDefaultValue(&core->config, "gb.model", "DMG");
	// mCoreConfigSetDefaultValue(&core->config, "cgb.model", "CGB");
	// mCoreConfigSetDefaultValue(&core->config, "sgb.model", "SGB2");
	opts.frameskip = 3;
	//opts.mute=1;
	//opts.videoSync=true;
	mCoreConfigLoadDefaults(&core->config, &opts);
	mCoreConfigSetUIntValue(&core->config, "frameskip", opts.frameskip);
	mCoreLoadConfig(core);
	__DEBUG_LINEFUNC__;
	core->loadROM(core, romFile);
	// core->loadSave(core, savFile);
	core->reset(core);
	printf("core reset done.\r\n");
	struct GB* gb = core->board;
	printf("sramsize:%d,sramp:%p\r\n", gb->sramSize, gb->memory.sram);
	FILE* fp = fopen("/sdcard/testgb.save", "rb");
	if (fp) {
		fseek(fp, 0, SEEK_END);
		if (ftell(fp) == gb->sramSize){
			fseek(fp, 0, SEEK_SET);
			int readed=fread(gb->memory.sram,1, gb->sramSize, fp);
			printf("readed%dbyres\r\n",readed);
		}
		fclose(fp);
	}

	int framecount = 0;
	u16 keyState = 0;
	int ch;
	uint64_t prevCycle100Frame = 1000; // read_cycle();
	while (1) {
		/*uint8_t myChar;
		STATUS s = uart_rx_one_char(&myChar);
		if (s == OK) {
			keyState |= translateKey(myChar);
		}*/
		enum GBAKey {
			GBA_KEY_A = 0,
			GBA_KEY_B = 1,
			GBA_KEY_SELECT = 2,
			GBA_KEY_START = 3,
			GBA_KEY_RIGHT = 4,
			GBA_KEY_LEFT = 5,
			GBA_KEY_UP = 6,
			GBA_KEY_DOWN = 7,
			GBA_KEY_R = 8,
			GBA_KEY_L = 9,
			GBA_KEY_MAX,
			GBA_KEY_NONE = -1
		};
		if (!keyson[0])
			keyState |= 1 << GBA_KEY_UP;
		if (!keyson[1])
			keyState |= 1 << GBA_KEY_DOWN;
		if (!keyson[2])
			keyState |= 1 << GBA_KEY_LEFT;
		if (!keyson[3])
			keyState |= 1 << GBA_KEY_RIGHT;
		if (!keyson[4])
			keyState |= 1 << GBA_KEY_SELECT;
		if (!keyson[5])
			keyState |= 1 << GBA_KEY_START;
		if (!keyson[6])
			keyState |= 1 << GBA_KEY_A;
		if (!keyson[7])
			keyState |= 1 << GBA_KEY_B;
		/*while ((ch = uarths_getc()) != -1) {
		    printk("received: %d\n", ch);
		    keyState |= translateKey(ch);
		}
		if (isPS2Connected) {
		    if (PS2X_read_gamepad(0, 0)) {
		        keyState = 0;
		        static const int keyMap[10] = {
		            PSB_CIRCLE, //GBA_KEY_A = 0,
		            PSB_CROSS, // GBA_KEY_B = 1,
		            PSB_SELECT, // GBA_KEY_SELECT = 2,
		            PSB_START, // GBA_KEY_START = 3,
		            PSB_PAD_RIGHT, // GBA_KEY_RIGHT = 4,
		            PSB_PAD_LEFT, // GBA_KEY_LEFT = 5,
		            PSB_PAD_UP, // GBA_KEY_UP = 6,
		            PSB_PAD_DOWN, // GBA_KEY_DOWN = 7,
		            PSB_R1, // GBA_KEY_R = 8,
		            PSB_L1, // GBA_KEY_L = 9,
		        };
		        for (int i = 0; i < 10; i++) {
		            keyState <<= 1;
		            keyState |= PS2X_Button(keyMap[9-i]);
		        }
		    }
		    else {
		        printf("read ps2 failed!\n");
		    }
		}*/
		core->setKeys(core, keyState);
		keyState = 0;
		core->runFrame(core);
		framecount++;
		if (framecount % 100 == 0) {
			uint64_t usPassed100Frame = xTaskGetTickCountFromISR() - prevCycle100Frame;
			printf("fps: %d\r\n", (int) (100 / ((double) usPassed100Frame / 100)));
			prevCycle100Frame = xTaskGetTickCountFromISR(); // read_cycle();
			//printf("SPIRAM:\r\n");
			//heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
			//printf("RAM:\r\n");
			//heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);
		}
		static int framecount;
		framecount--;
		if (framecount < 0) {
			isneedflash=1;
			//blitFromGbaToLcd320240P2P(videoOutputBuffer);
			framecount = opts.frameskip;
		}

		// vTaskDelay(1 / portTICK_PERIOD_MS);
		// blitFromGbaToLcd320240Upscaled(videoOutputBuffer);
		// ceUpdateBlockAge();
	}
}

void app_main() {
	printf("Hello world!\n");
	void testsd_main(void);
	void testlcd_main(void);
	void testi2s_main(void);
	/// testlcd_main();
	ST7789_Init();
	testsd_main();
	testi2s_main();
	/* Print chip information */
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
	       (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	printf("silicon revision %d, ", chip_info.revision);

	printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
	       (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	initAudio();
	// PS2X_confg_io(GPIOHS_PS2_CS, GPIOHS_PS2_CLK, GPIOHS_PS2_MOSI, GPIOHS_PS2_MISO);
	printf("hello world!\r\n");
	// lcd_init(15000000);
	// lcd_clear(BLACK);
	if (initSDCard() != 0) {
		return 0;
	}
	if (mountFatFs() != 0) {
		return 0;
	}

	// ceSetupMMU();
	/*for (int i = 0; i < 10; i++) {
	    msleep(200);
	    if (PS2X_read_gamepad(0, 0)) {
	        isPS2Connected = 1;
	        break;
	    }
	}*/
	// printf("isPS2Connected: %d\n", PS2X_read_gamepad(0,0));
	xTaskCreatePinnedToCore(&mgbaMainLoop, "mgba", 16384, NULL, 5, NULL, 1);
	xTaskCreatePinnedToCore(&keyscan, "keyscan", 2048, NULL, 4, NULL, 0);
	xTaskCreatePinnedToCore(&screenflash, "screenflash", 16384, NULL, 3, NULL, 0);

	/*
	    for (int i = 10; i >= 0; i--) {
	        printf("Restarting in %d seconds...\n", i);
	        vTaskDelay(1000 / portTICK_PERIOD_MS);
	    }
	    printf("Restarting now.\n");
	    fflush(stdout);
	    esp_restart();*/
}
void* anonymousMemoryMap(size_t size) {
	__DEBUG_LINEFUNC__;
	TRACE_1("anonymousMemoryMap: %p, %d\r\n", __builtin_return_address(0), (int) size);
	__DEBUG_LINEFUNC__;
	void* ptr = calloc(1, size);
	assert(ptr);
	return ptr;
}

void mappedMemoryFree(void* memory, size_t size) {
	free(memory);
}

void mgba_i2s_init()
{
	 int i2s_num = MGBA_I2S_NUM;
	 i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
        .sample_rate =  MGBA_I2S_SAMPLE_RATE,
        .bits_per_sample = MGBA_I2S_SAMPLE_BITS,
	    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
	    .channel_format = MGBA_I2S_FORMAT,
	    .intr_alloc_flags = 0,
	    .dma_buf_count = 4,
	    .dma_buf_len = 1024
	 };
	 printf("init i2s\r\n");
	 //install and start i2s driver
	 i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
	 //init DAC pad
	 i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
	 //init ADC pad

	i2s_set_clk(MGBA_I2S_NUM, MGBA_I2S_SAMPLE_RATE, MGBA_I2S_SAMPLE_BITS, MGBA_I2S_CHANNEL_NUM);

}

void testi2s_main(void)
{
	mgba_i2s_init();
}

void testlcd_main(void) {
	ST7735_Init();
	ST7735_FillScreen(ST7735_BLACK);

	for (int x = 0; x < ST7735_WIDTH; x++) {
		ST7735_DrawPixel(x, 0, ST7735_RED);
		ST7735_DrawPixel(x, ST7735_HEIGHT - 1, ST7735_RED);
	}

	for (int y = 0; y < ST7735_HEIGHT; y++) {
		ST7735_DrawPixel(0, y, ST7735_RED);
		ST7735_DrawPixel(ST7735_WIDTH - 1, y, ST7735_RED);
	}

	// Check fonts
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_WriteString(0, 0, "Font_7x10, red on black, lorem ipsum dolor sit amet", Font_7x10, ST7735_RED,
	                   ST7735_BLACK);
	ST7735_WriteString(0, 3 * 10, "Font_11x18, green, lorem ipsum", Font_11x18, ST7735_GREEN, ST7735_BLACK);
	ST7735_WriteString(0, 3 * 10 + 3 * 18, "Font_16x26", Font_16x26, ST7735_BLUE, ST7735_BLACK);

	// Check colors
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_WriteString(0, 0, "BLACK", Font_11x18, ST7735_WHITE, ST7735_BLACK);

	ST7735_FillScreen(ST7735_BLUE);
	ST7735_WriteString(0, 0, "BLUE", Font_11x18, ST7735_BLACK, ST7735_BLUE);

	ST7735_FillScreen(ST7735_RED);
	ST7735_WriteString(0, 0, "RED", Font_11x18, ST7735_BLACK, ST7735_RED);

	ST7735_FillScreen(ST7735_GREEN);
	ST7735_WriteString(0, 0, "GREEN", Font_11x18, ST7735_BLACK, ST7735_GREEN);

	ST7735_FillScreen(ST7735_CYAN);
	ST7735_WriteString(0, 0, "CYAN", Font_11x18, ST7735_BLACK, ST7735_CYAN);

	ST7735_FillScreen(ST7735_MAGENTA);
	ST7735_WriteString(0, 0, "MAGENTA", Font_11x18, ST7735_BLACK, ST7735_MAGENTA);

	ST7735_FillScreen(ST7735_YELLOW);
	ST7735_WriteString(0, 0, "YELLOW", Font_11x18, ST7735_BLACK, ST7735_YELLOW);

	ST7735_FillScreen(ST7735_WHITE);
	ST7735_WriteString(0, 0, "WHITE", Font_11x18, ST7735_BLACK, ST7735_WHITE);
	ST7735_InvertColors(1);
}
void testsd_main(void) {
	ESP_LOGI(TAG, "Initializing SD card");

#ifndef USE_SPI_MODE
	ESP_LOGI(TAG, "Using SDMMC peripheral");
	sdmmc_host_t host = SDMMC_HOST_DEFAULT();

	// This initializes the slot without card detect (CD) and write protect (WP) signals.
	// Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
	sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

	// To use 1-line SD mode, uncomment the following line:
	slot_config.width = 1;

	// GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
	// Internal pull-ups are not sufficient. However, enabling internal pull-ups
	// does make a difference some boards, so we do that here.
	gpio_set_pull_mode(15, GPIO_PULLUP_ONLY); // CMD, needed in 4- and 1- line modes
	gpio_set_pull_mode(2, GPIO_PULLUP_ONLY); // D0, needed in 4- and 1-line modes
	gpio_set_pull_mode(4, GPIO_PULLUP_ONLY); // D1, needed in 4-line mode only
	gpio_set_pull_mode(12, GPIO_PULLUP_ONLY); // D2, needed in 4-line mode only
	gpio_set_pull_mode(13, GPIO_PULLUP_ONLY); // D3, needed in 4- and 1-line modes

#else
	ESP_LOGI(TAG, "Using SPI peripheral");

	sdmmc_host_t host = SDSPI_HOST_DEFAULT();
	sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
	slot_config.gpio_miso = PIN_NUM_MISO;
	slot_config.gpio_mosi = PIN_NUM_MOSI;
	slot_config.gpio_sck = PIN_NUM_CLK;
	slot_config.gpio_cs = PIN_NUM_CS;
	host.max_freq_khz = 10000;
	// This initializes the slot without card detect (CD) and write protect (WP) signals.
	// Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
#endif // USE_SPI_MODE

	// Options for mounting the filesystem.
	// If format_if_mount_failed is set to true, SD card will be partitioned and
	// formatted in case when mounting fails.
	esp_vfs_fat_sdmmc_mount_config_t mount_config = { .format_if_mount_failed = false,
		                                              .max_files = 5,
		                                              .allocation_unit_size = 16 * 1024 };

	// Use settings defined above to initialize SD card and mount FAT filesystem.
	// Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
	// Please check its source code and implement error recovery when developing
	// production applications.
	sdmmc_card_t* card;
	esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG,
			         "Failed to mount filesystem. "
			         "If you want the card to be formatted, set format_if_mount_failed = true.");
		} else {
			ESP_LOGE(TAG,
			         "Failed to initialize the card (%s). "
			         "Make sure SD card lines have pull-up resistors in place.",
			         esp_err_to_name(ret));
		}
		return;
	}

	// Card has been initialized, print its properties
	sdmmc_card_print_info(stdout, card);

	// // Use POSIX and C standard library functions to work with files.
	// // First create a file.
	// ESP_LOGI(TAG, "Opening file");
	// FILE* f = fopen("/sdcard/hello.txt", "w");
	// if (f == NULL) {
	// 	ESP_LOGE(TAG, "Failed to open file for writing");
	// 	return;
	// }
	// fprintf(f, "Hello %s!\n", card->cid.name);
	// fclose(f);
	// ESP_LOGI(TAG, "File written");

	// // Check if destination file exists before renaming
	// struct stat st;
	// if (stat("/sdcard/foo.txt", &st) == 0) {
	// 	// Delete it if it exists
	// 	unlink("/sdcard/foo.txt");
	// }

	// // Rename original file
	// ESP_LOGI(TAG, "Renaming file");
	// if (rename("/sdcard/hello.txt", "/sdcard/foo.txt") != 0) {
	// 	ESP_LOGE(TAG, "Rename failed");
	// 	return;
	// }

	// // Open renamed file for reading
	// ESP_LOGI(TAG, "Reading file");
	// f = fopen("/sdcard/foo.txt", "r");
	// if (f == NULL) {
	// 	ESP_LOGE(TAG, "Failed to open file for reading");
	// 	return;
	// }
	// char line[64];
	// fgets(line, sizeof(line), f);
	// fclose(f);
	// // strip newline
	// char* pos = strchr(line, '\n');
	// if (pos) {
	// 	*pos = '\0';
	// }
	// ESP_LOGI(TAG, "Read from file: '%s'", line);

	// // All done, unmount partition and disable SDMMC or SPI peripheral
	// // esp_vfs_fat_sdmmc_unmount();
	// ESP_LOGI(TAG, "Card unmounted");
}

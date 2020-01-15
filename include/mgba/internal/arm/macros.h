/* Copyright (c) 2013-2014 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef MACROS_H
#define MACROS_H

#include <mgba-util/common.h>
extern uint32_t (*ext_read32)(void* addr);
extern uint16_t (*ext_read16)(void* addr);
extern uint8_t (*ext_read8)(void* addr);

#define LOAD_64 LOAD_64LE
//#define LOAD_32 LOAD_32LE
//#define LOAD_16 LOAD_16LE
#define LOAD_32(DEST, ADDR, ARR) if(((uint32_t)ARR)&0x80000000)if(ext_read32){(DEST)=ext_read32((void*)((uintptr_t) (ARR) + (size_t) (ADDR)));}else LOAD_32LE((DEST), (ADDR), (ARR));else LOAD_32LE((DEST), (ADDR), (ARR))
#define LOAD_16(DEST, ADDR, ARR) if(((uint32_t)ARR)&0x80000000)if(ext_read16){(DEST)=ext_read16((void*)((uintptr_t) (ARR) + (size_t) (ADDR)));}else LOAD_16LE((DEST), (ADDR), (ARR));else LOAD_16LE((DEST), (ADDR), (ARR))

#define STORE_64 STORE_64LE
#define STORE_32 STORE_32LE
#define STORE_16 STORE_16LE

#endif

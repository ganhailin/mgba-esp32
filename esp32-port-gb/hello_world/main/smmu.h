#ifndef SMMU_H
#define SMMU_H
#include <stdint.h> 

#define SMMU_ADDR_BEGIN (0x80000000)
#define SMMU_CACHE_SIZE (0x200000)
#define SMMU_PAGE_SIZE (0x10000)
#define SMMU_PAGE_MASK (~(SMMU_PAGE_SIZE-1))
#define SMMU_PAGE_NUM (SMMU_CACHE_SIZE/SMMU_PAGE_SIZE)
#define SMMU_MAX_MAPCOUNT (SMMU_PAGE_NUM)

typedef int (*smmu_readcb)(void* id,uint32_t addr,uint32_t size,void* buffer);
typedef int (*smmu_writecb)(void* id,uint32_t addr,uint32_t size,void* buffer);

typedef struct{
    void* id;
    uint32_t* vaddr;
    uint32_t* paddr;
    uint32_t lastaccess;
} smmu_page_t;
       
typedef struct{
    void* id;
    smmu_readcb r;
    smmu_writecb w;
    uint32_t mapsize;
    void* vaddr;
} smmu_map_t;


enum SMMUFLAG_t{
    SMMU_READ=0,
    SMMU_READ_WRITE=1
};
void* smmu_map(void* id,smmu_readcb r,smmu_writecb w,uint32_t size,enum SMMUFLAG_t flag);
uint32_t smmu_read32(void* addr);
uint16_t smmu_read16(void* addr);
uint8_t smmu_read8(void* addr);
 
#endif
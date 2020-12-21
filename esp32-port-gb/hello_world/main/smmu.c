#include "smmu.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
static int isinited=0;
static smmu_page_t smmu_pages[SMMU_PAGE_NUM];
static smmu_map_t smmu_maps[SMMU_MAX_MAPCOUNT];
static void* smmu_pool=NULL;
static uint32_t smmu_oc=0;
volatile int lastaccessindex=0;
#define smmu_printf printf
int smmu_init()
{
    smmu_oc++;
    memset(smmu_pages,0,sizeof(smmu_page_t)*SMMU_PAGE_NUM);
    memset(smmu_maps,0,sizeof(smmu_map_t)*SMMU_MAX_MAPCOUNT);
    smmu_pool=malloc(SMMU_CACHE_SIZE);
    if(!smmu_pool)
    {
        smmu_printf("Alloc pool failed\r\n");
        return -1;
    }
    smmu_printf("Alloc pool succeed\r\n");
    isinited=1;
    lastaccessindex=0;
    return 0;
}
void* smmu_map(void* id,smmu_readcb r,smmu_writecb w,uint32_t size,enum SMMUFLAG_t flag)
{
    smmu_oc++;
    if(!isinited)
        smmu_init();
    if(!isinited)
        return NULL;
    int i=0;
    for(;i<SMMU_MAX_MAPCOUNT;i++)
    {
        if(smmu_maps[i].vaddr==0)
            break;
    }
    if(i==SMMU_MAX_MAPCOUNT)
    {
        smmu_printf("no map left failed");
        return NULL;
    }
    //TODO:now I just need one file
    smmu_maps[0].id=id;
    smmu_maps[0].r=r;
    smmu_maps[0].w=w;
    smmu_maps[0].vaddr=(void*)SMMU_ADDR_BEGIN;
    smmu_maps[0].mapsize=size;
    return smmu_maps[0].vaddr;
}

uint32_t smmu_read32(void* addr)
{
    smmu_oc++;
    uint32_t mapid=0;//TODO only one map
    void * pageaddr=(uint32_t)addr&SMMU_PAGE_MASK;
    int i=0;
    if(smmu_pages[lastaccessindex].vaddr==pageaddr)
        i=lastaccessindex;
    else
    for(;i<SMMU_PAGE_NUM;i++)
    {
        if(smmu_pages[i].vaddr==pageaddr)
            break;
    }
    if(i==SMMU_MAX_MAPCOUNT)
    {
        i=0;
        for(;i<SMMU_PAGE_NUM;i++)
        {
            if(smmu_pages[i].vaddr==0)
                break;
        }
        if(i==SMMU_MAX_MAPCOUNT)
        {
            i=0;
            uint32_t minoc=smmu_pages[0].lastaccess;
            uint32_t minindex=0;
            for(;i<SMMU_PAGE_NUM;i++)
            {
                if(smmu_pages[i].lastaccess<minoc)
                {
                    minoc=smmu_pages[i].lastaccess;
                    minindex=i;
                }
            }
            i=minindex;
        }
        smmu_printf("map %p to %p at page%d\r\n",pageaddr,(void*)((uint32_t)smmu_pool+SMMU_PAGE_SIZE*i),i);
        smmu_pages[i].id=smmu_maps[mapid].id;
        smmu_pages[i].vaddr=pageaddr;
        smmu_pages[i].paddr=(uint32_t)smmu_pool+SMMU_PAGE_SIZE*i;
        smmu_pages[i].lastaccess=smmu_oc;
        smmu_maps[mapid].r(smmu_maps[mapid].id,pageaddr-smmu_maps[mapid].vaddr,SMMU_PAGE_SIZE,smmu_pages[i].paddr);
    }  
    if(i!=SMMU_MAX_MAPCOUNT)   
        {
            smmu_pages[i].lastaccess=smmu_oc;
            lastaccessindex=i;
            return *((uint32_t*)(((uint32_t)smmu_pages[i].paddr)+(addr-pageaddr)));
        }
    else
        return 0x5aa55aa5;
}
uint16_t smmu_read16(void* addr)
{
    uint32_t addr32=((uint32_t)addr)&0xfffffffc;
    uint32_t v=smmu_read32(addr32);
    uint8_t *vp=&v;
    return  *(uint16_t*)(vp+(((uint32_t)addr)&3));
}
uint8_t smmu_read8(void* addr)
{
    uint32_t addr32=((uint32_t)addr)&0xfffffffc;
    uint32_t v=smmu_read32(addr32);
    uint8_t *vp=&v;
    return  vp[((uint32_t)addr)&3];
}
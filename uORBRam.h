#ifndef __UORBRAM_H
#define __UORBRAM_H

#include <cstdint>
#include <StartUP.h>

class uORBRam
{
  public:
    
    uORBRam();
    ~uORBRam();
    
    void        uORBStackCheck(void);
    uint16_t    uORBGetStackFree(const char *stackName);
    void        *uORBCalloc(size_t count, size_t size);
    void        uORBFree(void *ptr, size_t count, size_t size);
    void        *uORBDataCalloc(uint16_t count, uint16_t size);
    size_t      *uORBStackInit(uint16_t size, char *name);
    
    
  private:
    static const int CCM_HEAP_SIZE=12*1024/4;      //12Kb
    static const int STACK_CHECK=12;
    
    uint32_t heapUsed, heapHighWater, dataSramUsed;

    uint32_t *ccmHeap[CCM_HEAP_SIZE];

    int32_t numStacks;
    void *stackPointers[STACK_CHECK];
    uint16_t stackSizes[STACK_CHECK];
    uint16_t stackFrees[STACK_CHECK];
    char *stackNames[STACK_CHECK];
    
};


#endif
#include "uORBRam.h"
#include <cstring>
#include <cstdlib>
#include <log_printf.h>

uORBRam::uORBRam()
{
    dataSramUsed=0;
}

uORBRam::~uORBRam()
{
    
}

void uORBRam::uORBStackCheck(void) {
    int i, j;

    for (i = 0; i < numStacks; i++) {
        for (j = 0; j < stackSizes[i]; j++)
            if (*((unsigned char *)(&stackPointers[i]+j)) != 0xFF)
                break;
        if (stackFrees[i] > j)
            stackFrees[i] = j;
        if (j < 16) {
            WARN("Possible stack overflow [%s]!\n", stackNames[i]);
        }
    }
}

uint16_t uORBRam::uORBGetStackFree(const char *stackName) {
    uint16_t stkFree = 0, i;

    for (i=0; i < numStacks; i++) {
        if ( !std::strncmp(stackName, stackNames[i], 20) ) {
            stkFree = stackFrees[i];
            break;
        }
    }

    return stkFree;
}


void *uORBRam::uORBCalloc(size_t count, size_t size) {
    char *addr = 0;

    if (count * size) {
        addr = (char*)std::calloc(count, size);

        heapUsed += count * size;
        if (heapUsed > heapHighWater)
            heapHighWater = heapUsed;

        if (addr == 0)
            printf("Out of heap memory!\n");
    }

    return addr;
}

void uORBRam::uORBFree(void *ptr, size_t count, size_t size) {
    if (ptr) {
        std::free(ptr);
        heapUsed -= count * size;
    }
}

// allocates memory from 64KB CCM
void *uORBRam::uORBDataCalloc(uint16_t count, uint16_t size) {
    uint32_t words;

    // round up to word size
    words = (count*size + sizeof(int)-1) / sizeof(int);

    if ((dataSramUsed + words) > CCM_HEAP_SIZE) {
        printf("Out of data SRAM!\n");
    }
    else {
        dataSramUsed += words;
    }

    return (void *)(ccmHeap + dataSramUsed - words);
}

// size in words
size_t *uORBRam::uORBStackInit(uint16_t size, char *name) {
    size_t *sp;

    // use memory in the CCM
    sp = (size_t *)uORBDataCalloc(1, size*4);

    // fill memory with pattern to ease overflow detection
    std::memset(sp, 0x00, size*4);

    stackPointers[numStacks] = sp;
    stackSizes[numStacks] = size*4;
    stackFrees[numStacks] = stackSizes[numStacks];
    stackNames[numStacks] = name;
    numStacks++;

    return sp;
}
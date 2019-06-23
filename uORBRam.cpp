/****************************************************************************
 *
 *   Copyright (c) 2016 Jin Wu. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name AMOV nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/



/* @Author: Jin Wu
   
   E-mail: jin_wu_uestc@hotmail.com
   Website: www.jinwu.science
*/


#include "uORBRam.h"

#include <cstring>
#include <cstdlib>
#include <log_printf.h>

uORBRam::uORBRam()
{
    dataSramUsed = 0;
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
    words = (count * size + sizeof(int) - 1) / sizeof(int);

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
    sp = (size_t *)uORBDataCalloc(1, size * 4);

    // fill memory with pattern to ease overflow detection
    std::memset(sp, 0x00, size * 4);

    stackPointers[numStacks] = sp;
    stackSizes[numStacks] = size * 4;
    stackFrees[numStacks] = stackSizes[numStacks];
    stackNames[numStacks] = name;
    numStacks++;

    return sp;
}

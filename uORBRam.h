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
    static const int CCM_HEAP_SIZE = 48 * 1024 / 4;      //48Kb
    static const int STACK_CHECK = 12;
    
    uint32_t heapUsed, heapHighWater, dataSramUsed;

    uint32_t *ccmHeap[CCM_HEAP_SIZE];

    int32_t numStacks;
    void *stackPointers[STACK_CHECK];
    uint16_t stackSizes[STACK_CHECK];
    uint16_t stackFrees[STACK_CHECK];
    char *stackNames[STACK_CHECK];
    
};


#endif

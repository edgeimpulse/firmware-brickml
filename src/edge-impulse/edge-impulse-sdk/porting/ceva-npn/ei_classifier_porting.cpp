/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Include ----------------------------------------------------------------- */
#include "../ei_classifier_porting.h"
#if EI_PORTING_CEVA_NPN == 1

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include "ceva-time.h"

#define CEVA_CLOCK_SPEED 400000000

static bool clock_ready = false;

__attribute__((weak)) EI_IMPULSE_ERROR ei_run_impulse_check_canceled()
{
    return EI_IMPULSE_OK;
}

__attribute__((weak)) EI_IMPULSE_ERROR ei_sleep(int32_t time_ms)
{
    return EI_IMPULSE_OK;
}

uint64_t ei_read_timer_ms()
{
    return ei_read_timer_us() / 1000;
}

uint64_t ei_read_timer_us()
{
    if (!clock_ready) {
        start_clock();
        reset_clock();
        clock_ready = true;
    }
    uint64_t cycles = clock();
    return (cycles / (CEVA_CLOCK_SPEED / 1000000));
}

__attribute__((weak)) void ei_printf(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	vprintf(format, args);
	va_end(args);
}

__attribute__((weak)) void ei_printf_float(float f)
{
    ei_printf("%f", f);
}

__attribute__((weak)) void *ei_malloc(size_t size)
{
    void *ptr = malloc(size);
    return ptr;
}

__attribute__((weak)) void *ei_calloc(size_t nitems, size_t size)
{
    void *ptr = calloc(nitems, size);
    return ptr;
}

__attribute__((weak)) void ei_free(void *ptr)
{
    free(ptr);
}

#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C"
#endif
__attribute__((weak)) void DebugLog(const char* s) {
    ei_printf("%s", s);
}

#endif // EI_PORTING_CEVA_NPN == 1

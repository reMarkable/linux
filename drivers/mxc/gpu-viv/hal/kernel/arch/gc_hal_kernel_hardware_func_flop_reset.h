/****************************************************************************
*
*    Copyright (c) 2005 - 2020 by Vivante Corp.  All rights reserved.
*
*    The material in this file is confidential and contains trade secrets
*    of Vivante Corporation. This is proprietary information owned by
*    Vivante Corporation. No part of this work may be disclosed,
*    reproduced, copied, transmitted, or used in any way for any purpose,
*    without the express written permission of Vivante Corporation.
*
*****************************************************************************/


#ifndef __gc_hal_kernel_hardware_func_flop_reset_h_
#define __gc_hal_kernel_hardware_func_flop_reset_h_
#ifdef __cplusplus
extern "C" {
#endif
#include "gc_hal.h"
#include "gc_hal_kernel.h"
#include "gc_hal_kernel_hardware.h"

gceSTATUS
gckHARDWARE_ResetFlopWithPPU(
    IN gckHARDWARE Hardware,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND_PTR Command
    );

gceSTATUS
gckHARDWARE_ResetFlopWithNN(
    IN gckHARDWARE Hardware,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND_PTR Command
    );

gceSTATUS
gckHARDWARE_ResetFlopWithTP(
    IN gckHARDWARE Hardware,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND_PTR Command
    );
#ifdef __cplusplus
}
#endif
#endif


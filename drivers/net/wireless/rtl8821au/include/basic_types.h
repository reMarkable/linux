/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#ifndef __BASIC_TYPES_H__
#define __BASIC_TYPES_H__


#define SUCCESS	0
#define FAIL	(-1)

	/* ULLI : the w*nd*ws hell */

	#include <linux/types.h>

//        #define UCHAR u8

	typedef 	__kernel_size_t	SIZE_T;
	typedef	__kernel_ssize_t	SSIZE_T;
	#define FIELD_OFFSET(s,field)	((SSIZE_T)&((s*)(0))->field)


#define MEM_ALIGNMENT_OFFSET	(sizeof (SIZE_T))
#define MEM_ALIGNMENT_PADDING	(sizeof(SIZE_T) - 1)

#define SIZE_PTR SIZE_T
#define SSIZE_PTR SSIZE_T

// Get the N-bytes aligment offset from the current length
#define N_BYTE_ALIGMENT(__Value, __Aligment) ((__Aligment == 1) ? (__Value) : (((__Value + __Aligment - 1) / __Aligment) * __Aligment))

#endif //__BASIC_TYPES_H__


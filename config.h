/*
 * config.h
 *
 *

 *   Copyright (C) 2008-2022 Wanye Knowles ZL2BKC. 2023 Simon Eatough ZL2BRG
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */



#ifndef CONFIG_H_
#define CONFIG_H_


#define VERSION "4.2A"

#if 1		// ADF5355, ZLPLL 14G
#include "adf5355.h"
#define _ADF5355		1
#define PLLCHIP			"ADF5355"
#define PLL(a)			ADF5355_##a
#define LOCKINPUT		MUXOUTPIN
#define PWM_ENABLE		0

#else		// ADF4351, ZLPLL V4
#include "ADF435x.h"
#define _ADF4351		1
#define PLLCHIP			"ADF4351"
#define PLL(a)			ADF4351_##a
#define LOCKINPUT		LOCKIN
#define PWM_ENABLE		1
#endif

#define SWEEP		0
#define CHANNELS	16
#define CW_BEACON	1
#define LOCK_TEST	1
#define THUMBWHEEL	0		// Broken

#if CHANNELS > 16
#error "CHANNELS must be <= 16"
#endif



#endif /* CONFIG_H_ */

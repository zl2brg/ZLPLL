/*
 * encoder.h
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



#ifndef DIAL_H_
#define DIAL_H_

#include <stdlib.h>
#include <stdint.h>


#define DIR_NONE	0x0
#define DIR_CW		0x10		// Clockwise step.
#define DIR_CCW		0x20		// Anti-clockwise step.
#define DIR_PRESS	0x40

#define DIR_CW2	(0x10|0x40)		// rotate with button pushed
#define DIR_CCW2 (0x20|0x40)

#define DIR_PRESS2	0x80

void dial_init();
uint8_t dial_read();

#endif /* DIAL_H_ */

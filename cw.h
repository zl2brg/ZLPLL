/*
 * IncFile1.h
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





#ifndef CW_H_
#define CW_H_

#define	 MSGTYPE_EEPROM		0
#define  MSGTYPE_PGMMEM		1
#define  MSGTYPE_RAM		2

void cw_start(uint8_t type, uint8_t *msg, uint8_t delay);
void cw_stop(uint8_t rfon);

#endif /* CW_H_ */

/*
 *   mgm.c
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

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdio.h>

#include "config.h"
#include "zlpll.h"
#include "mgm.h"

uint8_t		MGM_Message[MGM_MAXMSGLEN];
uint8_t		MGM_MsgLen;

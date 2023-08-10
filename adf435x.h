/*
 * ADF435x.h
 *
 *   Copyright (C) 2017-2022 Wanye Knowles ZL2BKC. 2023 Simon Eatough ZL2BRG
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



#ifndef ADF435X_H_
#define ADF435X_H_

#include <stdlib.h>
#include <stdint.h>

#define ADF4351_MIN_FREQ	(2200.0/64)

void ADF4351_set_rflevel(int8_t enable, int8_t level);
void ADF4351_set_freq(float freq);
void ADF4351_set_muxout(uint8_t v);
void ADF4351_powerdown();
void ADF4351_fsk(uint16_t step, int16_t delta);

#endif /* ADF435X_H_ */


/*
 * ADF5355.h
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


#ifndef ADF5355_H_
#define ADF5355_H_
#include "zlpll.h"
#include <stdlib.h>
#include <stdint.h>

/*
 * 8  bit mod2 kconst =2^48/pfd 2^24(f1) + 2^8(f2) + 2^8 (mod2 = 258)
 * 12 bit mod2 konst = 2^52/pfd 2^24(f1) + 2^13(f2) + 2^13 (mod2 = 4096)
 * 13 bit mod2 konst = 2^53/pfd 2^24(f1) + 2^13(f2) + 2^16 (mod2 = 8192)
*/


//#define KCONST     1125899907
//#define KCONST_LOW 549755

//#define FRACTIONAL_SCALE 2^53
#define ADF5355_MODULUS1	16777216

#define MAX_VCO_FREQ 680000000
#define MAX_REF_FREQ 100000000


extern  uint64_t  __udivdi3( uint64_t , uint64_t  );
extern  uint64_t  __muldi3( uint64_t , uint64_t );


void ADF5355_set_rflevel(int8_t enable, int8_t level);
void ADF5355_set_freq(uint32_t freq);
void ADF5355_set_muxout(uint8_t v);
void ADF5355_powerdown();
void ADF5355_fsk(uint16_t step, int16_t delta);




typedef union{
    uint64_t R;
    struct PACKED fs  {
	uint32_t  padding1:8;
    uint32_t  fract2:8;
    uint32_t  fract1:24;
	uint32_t  padding:24;

}fs;
}PACKED_RESULT_40;

/*
typedef union{
    uint64_t R;
    struct  PACKED fs_52 {
	unsigned  padding :16;
    uint16_t  fract2:16;
    uint32_t  fract1:32;
}fs_52;
}PACKED_RESULT_52;


typedef union{
    uint64_t R;
    struct  PACKED fs_53 {
    uint16_t  fract2:8;
    uint32_t  fract1:24;
    uint32_t padding :32;

}fs_53;
}PACKED_RESULT_53;
*/

#endif /* ADF5355_H_ */

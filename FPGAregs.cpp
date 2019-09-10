/*****************************************************************************
 *
 * RF CREATIONS Copyright (C) 2015
 *
 * All Rights Reserved. Unpublished rights reserved under the
 * copyright laws of the United Kingdom
 *
 * The software contained on this media is proprietary to and embodies
 * the confidential technology of RF CREATIONS LTD.
 *
 * Possession, use, duplication or dissemination of the software and
 * media is authorized only pursuant to a valid written license from
 * RF CREATIONS LTD.
 *
 * PROJECT:     Gromit
 *
 * FILE:        FPGAregs.c
 *
 * DESCRIPTION: Mapping of FPGA registers into memory
 *
 *****************************************************************************/

#include	"FPGAregs.h"
#include <morephsearch.h>

#include "registers.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

//_____________________________________________________________________________
//
//	void		FPGAVersion( void )
//
//	Prints the FPGA version
//_____________________________________________________________________________

void	FPGAVersion( void )
{
    uint32_t	x;

    x = gromit_regs.build_version;
    printf( "FPGA version      : %03d.%03d.%03d.%03d\r\n" , (x >> 24) , (x >> 16) & 255 , (x >> 8) & 255 , x & 255 );

    x = gromit_regs.build_date;
    printf( "FPGA date         : %04d-%02d-%02d\r\n" , x & 0xFFFF , (x >> 16) & 255 , (x >> 24) );

    x = gromit_regs.build_time;
    printf( "FPGA time         : %02d:%02d:%02d\r\n" , (x >> 16) , (x >> 8) & 255 , x & 255 );

}

//_____________________________________________________________________________
//
//	void		FPGAtime( uint32_t *msb , uint32_t *lsb )
//
//	Reads FPGA time
//_____________________________________________________________________________

void	FPGAtime( uint32_t *msb , uint32_t *lsb )
{
    uint32_t	last_msb;
    uint32_t	count;


    count = 0;
    *msb = hw.fpga_time_msb;
    last_msb = *msb + 1;
    while( *msb != last_msb ) {
        last_msb = *msb;
        *lsb = hw.fpga_time_lsb;
        *msb = hw.fpga_time_msb;
        count++;
        if( count == 10000 ) printf( "FPGAtime : Failed to read FPGA time\r\n" );
    }
}

//_____________________________________________________________________________

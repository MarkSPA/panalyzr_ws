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
 * FILE:        FPGAregs.h
 *
 * DESCRIPTION: Memory mapped FPGA registers
 *
 *****************************************************************************/

//_____________________________________________________________________________

#ifndef	FPGAregsH

#define	FPGAregsH

#include    <cstdint>

void			FPGAVersion( void );
void			FPGAtime( uint32_t *msb , uint32_t *lsb );

#endif

//_____________________________________________________________________________


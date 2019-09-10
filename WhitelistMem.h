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
 * FILE:        WhitelistMem.h
 *
 * DESCRIPTION: Memory mapped whitelist
 *
 *****************************************************************************/

//_____________________________________________________________________________

#ifndef WhitelistMemH

#define WhitelistMemH
#include "utils.h"
class WhitelistTimeout : public ExceptionFormat {
    using ExceptionFormat::ExceptionFormat;
};

void WhitelistMap(void);
uint32_t WhitelistGet(uint32_t* WhitelistEntries);
void WhitelistAdd(uint32_t aa, uint32_t seed, uint32_t flags);
void WhitelistSub(uint32_t aa);
void WhitelistTrig(uint32_t aa, bool trig);
void WhitelistClear(void);
void WhitelistUpdateConn(bool conn);
void WhitelistUpdatePeriodic(bool per);
void WhitelistUpdateICl(bool icl);
void WhitelistUpdateArm(bool arm);
void WhitelistUpdateSearch12(bool search);
void WhitelistUpdateSearchLR(bool search);
void WhitelistUpdateICO(bool ico);
uint8_t WhitelistFlags(uint32_t addr);
uint32_t WhitelistCRC(uint32_t addr);

extern bool SortAge;

#endif

//_____________________________________________________________________________

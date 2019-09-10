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
 * FILE:        USBdata.h
 *
 * DESCRIPTION: Include file for USB data end-point
 *
 *****************************************************************************/

//_____________________________________________________________________________

#ifndef USBdataH
#define USBdataH

#include "lapis.h"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <vector>

void USBdataThread();
void InitTime(void);

using system_clock = std::chrono::system_clock;
using namespace std::chrono_literals;

extern system_clock::time_point twl;
extern system_clock::time_point tenv;
extern system_clock::time_point tbuf;
extern system_clock::duration twl_int;
extern system_clock::duration tenv_int;
extern system_clock::duration tbuf_int;

extern uint64_t time_offset;

extern uint16_t low_water;
extern uint16_t hig_water;

extern uint32_t NumPktsPending;

extern bool host_up;

void usb_data_callback(std::vector<uint8_t> v);

extern std::atomic_bool data_stop;
extern lapis_data_callback_t callback;
lapis_error_t handle_errors();

#define NumEu (70)

extern  uint32_t EuAA[NumEu];
extern  uint64_t EuTT[NumEu];
extern  uint32_t EuCRC[NumEu];
extern  uint32_t EuBAD[NumEu];
extern  uint32_t EuTYP[NumEu];
extern  uint64_t tkeep;

extern bool SendBle;
extern bool SendFM;
extern bool SendWLup;
extern bool SendSpec;
extern bool SendGain;
extern bool SendDIO;
extern bool SendU0;
extern bool SendU1;
extern bool SendU2;
extern bool SendU3;
extern bool SendWhitelist;
extern bool SendEnv;
extern bool SendDebug;
extern bool SendDebugBlockRam;
extern bool SendSpace;

extern bool AutoPruneConnRqst;
extern bool AutoPruneAge;
extern bool AutoPruneCRC;
extern uint64_t TimeToLive;
extern uint32_t NumBadCRC;

#endif

//_____________________________________________________________________________

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
 * FILE:        USBdata.c
 *
 * DESCRIPTION: Routines for handling the USB data end-point
 *
 *****************************************************************************/

//_____________________________________________________________________________
//
// Includes
//_____________________________________________________________________________

#include "USBdata.h"
#include "FPGAregs.h"
#include "WhitelistMem.h"
#include "lapis.h"
#include "registers.h"
#include <chrono>
#include <iostream>
#include <thread>

using system_clock = std::chrono::system_clock;
using namespace std::chrono_literals;

system_clock::time_point twl;
system_clock::time_point tenv;
system_clock::time_point tbuf;
system_clock::duration twl_int = 1s;
system_clock::duration tenv_int = 5s;
system_clock::duration tbuf_int = 5s;
uint64_t time_offset;

uint16_t low_water = 20;
uint16_t hig_water = 80;

uint32_t NumPktsPending = 0;

bool SendBle = false;
bool SendFM = false;
bool SendWLup = false;
bool SendSpec = true;
bool SendGain = false;
bool SendDIO = false;
bool SendU0 = false;
bool SendU1 = false;
bool SendU2 = false;
bool SendU3 = false;
bool SendWhitelist = false;
bool SendEnv = false;
bool SendDebug = false;
bool SendDebugBlockRam = false;
bool SendSpace = false;

bool AutoPruneConnRqst = true;
bool AutoPruneAge = true;
bool AutoPruneCRC = true;
uint64_t TimeToLive = 600ull * 400000ull;
uint32_t NumBadCRC = 100;

//_____________________________________________________________________________
//
// Prototypes
//_____________________________________________________________________________

static void ConnectionRequest(uint16_t idx);
static int PktSort(const void* a, const void* b);

void EuthanasiaUpdate(uint32_t aa, uint64_t pkt_time, uint64_t now_time, uint32_t pktTyp,
                      uint32_t crc, uint32_t len, uint32_t chan, uint32_t phy, int8_t rssi);
void EuthanasiaKill(uint64_t tnow);

//_____________________________________________________________________________
//
// Static variables
//_____________________________________________________________________________

uint32_t EuAA[NumEu] = {0};
uint64_t EuTT[NumEu];
uint32_t EuCRC[NumEu];
uint32_t EuBAD[NumEu];
uint32_t EuTYP[NumEu];
uint64_t tkeep;

static int usb_data_fd;
static uint32_t nCRobs = 0;
static uint8_t CRlst[5120];

typedef struct BuffSort_struct {
    uint64_t delta;
    uint16_t typ;
    uint16_t idx;
} BuffSort;

typedef enum { empty, full, continuation } occupancy_type;

typedef struct Buff1kB_struct {
    uint64_t timestamp;
    uint16_t len;
    uint16_t typ;
    occupancy_type occupancy;
} Buff1kB;

union time_union {
    uint32_t msb_lsb[2];
    uint64_t full;
};

union Pkt_union {
    uint16_t len_typ[2];
    uint32_t time_msb;
    uint32_t hdr;
};

typedef struct Pkt_struct {
    union Pkt_union invalid;
    union Pkt_union valid;
    uint32_t time_lsb;
    uint32_t x[253];
} Pkt;

static Buff1kB buff[65536];
static BuffSort bsort[65536];
static volatile Pkt* pkt;
static uint32_t WhitelistEntries[512];
static uint32_t nWL;

static char wrap_buffer[9 * 1024];

static uint32_t BuffStats[4] = {BUFFSTATSpkt << 24 | 3, 0, 0, 0};

inline void usleep(int us) { std::this_thread::sleep_for(us * 1us); }

const size_t buffer_size = 64 * 1024 * 1024;
static uint8_t buffer_memory[buffer_size] = {0};
static size_t wrIndex = 4;
static uint16_t fpga_wr = 0;
static uint16_t arm_rd = 65535;
static size_t consume = 0;
static bool ditch = false;
static uint16_t nBuff = 0;

void usb_data_callback(std::vector<uint8_t> v) {
    size_t l;
    size_t offset;
    size_t k;
    size_t rem;
    size_t spare;
    uint32_t* hdr;

    if (v.size() == 0)
        return;

    std::vector<uint8_t> msg(4);
    if (callback)
        callback(msg.data(), (uint32_t)msg.size(), LAPIS_NO_ERROR);

    offset = 0;   // Pointer to start of next packet in new data
    l = v.size(); // Amount of new data still to be processed

    while (l) {

        if (consume) {

            k = (l < consume) ? l : consume; // Amount of data to be copied from USB

            if (!ditch) { // If space then copy data from USB

                rem = buffer_size - wrIndex; // Remaining space in buffer before it wraps

                if (k > rem)
                    memcpy(buffer_memory, v.data() + offset + rem, k - rem);
                else
                    rem = k;

                memcpy(buffer_memory + wrIndex, v.data() + offset, rem);

                consume -= k;
                offset += k;
                l -= k;

                wrIndex += k;
                if (wrIndex >= buffer_size)
                    wrIndex -= buffer_size;
                if (consume == 0)
                    fpga_wr += nBuff;

            } else {
                consume -= k;
                offset += k;
                l -= k;
            }
        }

        if (!l)
            break; // All data from USB consumed

        hdr = (uint32_t*)(v.data() + offset); // Pointer to packet header

        consume = 4 * (*hdr & 0x0000FFFF) + 4; // Amount of data to be taken from USB

        wrIndex = 1024 * ((uint32_t)fpga_wr) +
                  4; // Data is written starting at 2nd word to allow for timestamp insertion

        nBuff = ((uint16_t)((consume + 4 + 1023) /
                            1024)); // Number of buffers required to hold new data

        if (fpga_wr < arm_rd) // Number of free buffers
            spare = arm_rd - fpga_wr - 1;
        else
            spare = 65535 - (fpga_wr - arm_rd);

        ditch = nBuff >= spare; // Discard new data if not space to store it
    }

    return;
}

//_____________________________________________________________________________
//
//	int	PktSort( void *a , void *b )
//
//	Sorts pending packets into time order, oldest packets first
//_____________________________________________________________________________

static int PktSort(const void* a, const void* b) {
    BuffSort* aa;
    BuffSort* bb;
    int atyp;
    int btyp;

    aa = (BuffSort*)a;
    bb = (BuffSort*)b;

    if (aa->delta == bb->delta) {
        atyp = aa->typ;
        btyp = bb->typ;
        return (atyp - btyp);
    }

    if (aa->delta > bb->delta)
        return (-1);

    return (1);
}

//_____________________________________________________________________________
//
//	void	InitTime( void )
//
//	Synchronises FPGA time with Linux time
//_____________________________________________________________________________

void InitTime(void) {
    uint64_t tnow;
    uint64_t fpga_time;
    uint32_t fpga_lsb;
    uint32_t fpga_msb;

    // Initialise update times

    twl = std::chrono::system_clock::now();

    tenv = std::chrono::system_clock::now();

    tbuf = std::chrono::system_clock::now();

    // Get current FPGA time and calculate time offset

    FPGAtime(&fpga_msb, &fpga_lsb);

    tnow = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now())
               .time_since_epoch()
               .count();

    fpga_time = fpga_msb;
    fpga_time <<= 28;
    fpga_time += fpga_lsb;

    time_offset = tnow * 4;

    time_offset -= fpga_time;
}

//_____________________________________________________________________________
//
//	bool	HostOpen( void )
//
//	Test for open host connection
//_____________________________________________________________________________

static bool DataWrite(uint8_t* x, int n) {
    if (callback) {
        std::vector<uint8_t> msg(n + 4);
        uint8_t framing[4] = {0xAA, 0x55, 0xAA, 0x55};
        std::copy(x, x + n, msg.data());
        std::copy(framing, framing + 4, msg.data() + n);
        callback(msg.data(), (uint32_t)msg.size(), LAPIS_NO_ERROR);
        return true;
    }
    return false;
}

//_____________________________________________________________________________
//
//	void	USBdataThread( void )
//
//	Thread for USB data end point
//_____________________________________________________________________________

extern uint32_t blkmem[16384];
void USBdataThread() try {
    uint32_t k;
    uint16_t idx = 0;
    uint16_t idx_last = 0;
    uint16_t idx_tst;
    uint16_t idx_buf;
    uint32_t jdx;
    uint32_t jdx_end;
    uint16_t nbuff;
    uint64_t delta;
    uint16_t fpga_idx;
    uint32_t fpga_lsb;
    uint32_t fpga_msb;
    union time_union fpga_time;
    uint64_t msb_norm;
    uint64_t msb_wrap;
    union time_union pkt_time;
    uint16_t typ;
    uint16_t num;
    uint32_t lsb;
    uint16_t incr;
    uint32_t Send;
    system_clock::time_point tnow;
    system_clock::time_point tnext;
    bool WorkDone;
    bool write_ok;
    uint32_t space_now;
    uint32_t space_old = 0;
    bool space_change;
    bool no_time;
    bool space_low;
    bool space_hig;
    int space_last = 0;
    uint32_t jdx_delta;
    uint32_t kdx;
    uint16_t idx_j;
    uint16_t idx_k;
    uint32_t xj;
    uint32_t xk;
    int32_t rssi_j;
    int32_t rssi_k;
    bool is_tst;
    bool went_bad = false;
    uint64_t tlast = 0;
    bool throttling = false;

    // Pointer intialisation

    pkt = (Pkt*)buffer_memory;

    // Initialise time

    InitTime();

    usb_data_fd = -1;

    arm_rd = 65535;

    // Clear out all buffers

    for (k = 0; k < 65536; k++) {
        buff[k].timestamp = 0;
        buff[k].occupancy = empty;
    }

    NumPktsPending = 0;

    // Get current FPGA write location

    fpga_idx = (uint16_t)fpga_wr;

    // Set current sw read location just before FPGA write location

    idx_last = arm_rd;
    idx = idx_last + 1;

    space_now = 100;
    space_old = 0;

    // Packet processing loop

    WorkDone = true;

    while (!data_stop) {

        if (went_bad) {
            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!! NOT_WRITTEN\r\n");
            printf("%d %d\n", idx, idx_last);
            while (1) {
            }
        }

        if (!WorkDone)
            usleep(10000);

        if (!callback) { // Sleep until we have somewhere to put the data
            usleep(100000);
            continue;
        }

        if (gromit->dead()) {
            callback(nullptr, 0, LAPIS_ERROR_NO_DEVICE);
            break;
        }

        WorkDone = false;

        try {
            write_ok = true;

            // Get location of new FPGA write pointer
            // This must be done before we get the FPGA time to ensure packets are older than
            // current FPGA time

            fpga_idx = (uint16_t)fpga_wr;

            // Get current FPGA timestamp
            // This must be done after reading fgpa_wr so that all packets are in the past

            FPGAtime(&fpga_msb, &fpga_lsb);

            msb_norm = (((uint64_t)fpga_msb) << 28);
            msb_norm += time_offset;
            msb_wrap = msb_norm - 0x10000000;

            fpga_time.full = msb_norm + fpga_lsb;

            tkeep = fpga_time.full;

            if ((fpga_time.full - tlast) > 4000000ull) {
                tlast = fpga_time.full;
                EuthanasiaKill(tlast);
            }

            if (fpga_idx >= idx_last)
                space_now = 65536 - (((uint32_t)fpga_idx) - ((uint32_t)idx_last));
            else
                space_now = ((uint32_t)idx_last) - ((uint32_t)fpga_idx);

            if ((space_now < 6554) && (!throttling)) {
                gromit_regs.throttle = 1;
                throttling = true;
            } else if ((space_now > 6554) && throttling) {
                gromit_regs.throttle = 0;
                throttling = true;
            }

            space_now *= 100;
            space_now += 32768;
            space_now /= 65536;

            went_bad = false;

            // Update packet records

            while (idx != fpga_idx) {

                typ = pkt[idx].valid.len_typ[1] >> 8;

                if (typ == NOT_WRITTEN) {
                    went_bad = true;
                    break;
                }

                WorkDone = true;

                num = pkt[idx].valid.len_typ[0] + 1; // add one to allow for insertion of time msb
                buff[idx].len = num;
                buff[idx].typ = typ;

                switch (typ) {

                case BLEpkt: // Need to swap timestamp with BLE packet header

                    // Swap access address and CRC seed

                    lsb = pkt[idx].x[1];
                    pkt[idx].x[1] = pkt[idx].x[2];
                    pkt[idx].x[2] = lsb;

                    lsb = pkt[idx].x[0] & 0x0FFFFFFF;
                    pkt[idx].x[0] =
                        pkt[idx].time_lsb - (((uint32_t)25) << 16); // Botch up length field
                    pkt[idx].time_lsb = lsb;

                    // Botch up access address if ICL stream
                    // The packet currently has the base address
                    // The stream number can be extracted from the CRC seed

                    if ((pkt[idx].x[1] >> 24) == 2) {
                        uint32_t aa_div = pkt[idx].x[1] & 0xFF;
                        aa_div = (35 * aa_div + 42) & 127;
                        uint32_t b1 = (aa_div >> 1) & 1;
                        uint32_t b2 = (aa_div >> 2) & 1;
                        uint32_t b3 = (aa_div >> 3) & 1;
                        uint32_t b4 = (aa_div >> 4) & 1;
                        uint32_t b5 = (aa_div >> 5) & 1;
                        uint32_t b6 = (aa_div >> 6) & 1;
                        uint32_t aa = pkt[idx].x[2];
                        uint32_t a15 = (aa >> 15) & 1;
                        if (b1)
                            aa ^= 0xFC000000;
                        aa &= 0x02C90000;
                        aa |= (b1 << 25) | ((1 - b1) << 23) | ((1 - a15) << 22) | (a15 << 19) |
                              ((1 - a15) << 16);
                        aa ^= (b6 << 24) | (b5 << 21) | (b4 << 20) | (b3 << 18) | (b2 << 17);
                        pkt[idx].x[2] = aa;
                    }

                    Send = SendBle;

                    ConnectionRequest(idx);
                    break;

                case WHITELISTpkt:
                    lsb = pkt[idx].time_lsb;
                    Send = SendWLup;
                    break;

                case SPECTRUMpkt:
                    lsb = pkt[idx].time_lsb;
                    Send = SendSpec;
                    break;

                case GAINpkt:
                    lsb = pkt[idx].time_lsb;
                    Send = SendGain;
                    break;

                case DIGITALpkt:
                    printf("DIO %08X\r\n", pkt[idx].x[0]);
                    lsb = pkt[idx].time_lsb;
                    Send = SendDIO;
                    break;

                case UART0pkt:
                    printf("UART\r\n");
                    lsb = pkt[idx].time_lsb;
                    Send = SendU0;
                    break;

                case UART1pkt:
                    printf("UART\r\n");
                    lsb = pkt[idx].time_lsb;
                    Send = SendU1;
                    break;

                case UART2pkt:
                    printf("UART\r\n");
                    lsb = pkt[idx].time_lsb;
                    Send = SendU2;
                    break;

                case UART3pkt:
                    printf("UART\r\n");
                    lsb = pkt[idx].time_lsb;
                    Send = SendU3;
                    break;

                default:
                    printf("pkts sent = %08X\n", uint32_t(gromit_regs.build_feat));
                    printf("UNKNOWN : %6d %2d %08X %08X %08X %08X %08X %08X %08X %08X %08X\r\n",
                           idx, typ, pkt[idx].valid.hdr, pkt[idx].time_lsb, pkt[idx].x[0],
                           pkt[idx].x[1], pkt[idx].x[2], pkt[idx].x[3], pkt[idx].x[4],
                           pkt[idx].x[5], pkt[idx].x[6]);
                    printf("Pkt type = %d (h'%04X)\r\n", typ, typ);
                    printf("Pkt len  = %d (h'%04X)\r\n", num, num);
                    for (k = 0; k < 10; k++)
                        printf("%08X ", pkt[idx].x[k]);
                    printf("\r\n");
                    printf("USBdataThread : Unknown packet type @ index %d\r\n", idx);
                    callback(nullptr, 0, LAPIS_ERROR_UNKNOWN_PACKET);
                    lsb = pkt[idx].time_lsb;
                    Send = false;
                    while (1) {
                    }
                }

                // Timestamps in packets must be before current FPGA time

                if (lsb > fpga_lsb)
                    pkt_time.full = ((uint64_t)lsb) + msb_wrap;
                else
                    pkt_time.full = ((uint64_t)lsb) + msb_norm;

                buff[idx].timestamp = pkt_time.full;

                // Copy packet header forward to make space for time msb
                // Need to add one to the length because we have added the time msb

                pkt[idx].invalid.len_typ[0] =
                    num; // This has been incremented to allow for time msb insertion
                pkt[idx].invalid.len_typ[1] = pkt[idx].valid.len_typ[1];

                // Fill in time msb and update time lsb

                pkt[idx].valid.time_msb = pkt_time.msb_lsb[1];
                pkt[idx].time_lsb = pkt_time.msb_lsb[0];

                if (Send) {
                    buff[idx].occupancy = full;
                    WorkDone = true;
                    NumPktsPending++;
                } else {
                    buff[idx].occupancy = empty;
                }

                if (typ == BLEpkt)
                    EuthanasiaUpdate(
                        pkt[idx].x[2], pkt_time.full, fpga_time.full, pkt[idx].x[1] >> 24,
                        pkt[idx].x[num - 3] & 0xFF000000, pkt[idx].x[0] >> 16, pkt[idx].x[0] & 63,
                        (pkt[idx].x[0] >> 6) & 3, (int8_t)((uint8_t)(pkt[idx].x[0] >> 8) & 255));

                // Step to next packet
                // Number of words in packet is ( num + 1 )
                // since the word count is not included in the length
                // (num now includes the extra word at the start)

                incr = 1 + (num / 256);
                idx++;

                if (Send) {
                    // Flag any continuation packets
                    for (k = 1; k < incr; k++)
                        buff[idx++].occupancy = continuation;
                } else {
                    for (k = 1; k < incr; k++)
                        buff[idx++].occupancy = empty;
                }
            }

            // Attempt whitelist and environmental data before buffers fill

            tnow = std::chrono::system_clock::now();

            // If these were copied into the main buffer then they would come out in time order
            // could be tricky to allocate the space in the main buffer

            // Check for whitelist dump

            no_time = twl_int.count() == 0;

            if ((!no_time) && (tnow > twl)) {
                tnext = twl + twl_int;
                if (tnow > tnext)
                    tnext = tnow + twl_int;
                nWL = WhitelistGet(&(WhitelistEntries[4]));

                if (SendWhitelist && write_ok) {

                    WhitelistEntries[3] = nWL;
                    WhitelistEntries[2] = fpga_time.msb_lsb[0];
                    WhitelistEntries[1] = fpga_time.msb_lsb[1];
                    WhitelistEntries[0] = (2 * nWL + 3) | WHITELISTENTRIESpkt << 24;
                    WorkDone = true;
                    write_ok = DataWrite((uint8_t*)WhitelistEntries, 4 * (2 * nWL + 4));
                    if (write_ok)
                        twl = tnext;
                }
            }

            // Check for buffer space dump

            if (SendSpace && write_ok) {

                space_low = ((space_old >= low_water) && (space_now < low_water));
                if (space_last == -1)
                    space_low = false;

                space_hig = ((space_old <= hig_water) && (space_now > hig_water));
                if (space_last == 1)
                    space_hig = false;

                space_change = (space_low || space_hig);

                no_time = tbuf_int.count() == 0;

                if (((!no_time) && tnow > tbuf) || space_change) {
                    tnext = tbuf + tbuf_int;
                    if (tnow > tnext)
                        tnext = tnow + tbuf_int;
                    WorkDone = true;
                    BuffStats[1] = fpga_time.msb_lsb[1];
                    BuffStats[2] = fpga_time.msb_lsb[0];
                    BuffStats[3] = space_now;
                    write_ok = DataWrite((uint8_t*)&(BuffStats), 16);
                    if (write_ok) {
                        space_old = space_now;
                        space_last = 0;
                        if (space_low)
                            space_last = -1;
                        if (space_hig)
                            space_last = 1;
                        tbuf = tnext;
                    }
                } else {
                    space_old = space_now;
                }

            } else {
                if (write_ok)
                    space_old = space_now;
            }

            // No point sorting packets if we can't write to the USB

            if (!write_ok)
                continue;

            // Now sort and empty packet buffers

            // By using unsigned arithmetic packets with bad time stamps just appear far in the past
            // and so do not block the buffer

            idx_tst = idx_last;
            jdx = 0;
            while (idx_tst != idx) {
                if (buff[idx_tst].occupancy == full) {
                    delta = fpga_time.full - buff[idx_tst].timestamp;
                    if (delta > 400000) { // 100ms
                        bsort[jdx].delta = delta;
                        bsort[jdx].typ = buff[idx_tst].typ;
                        bsort[jdx].idx = idx_tst;
                        jdx++;
                    } else {
                    }
                }
                idx_tst++;
            }
            jdx_end = jdx;

            if (jdx_end) {

                WorkDone = true;

                // Sort packets so that oldest are first

                if (jdx_end > 1)
                    qsort(&(bsort[0]), jdx_end, sizeof(BuffSort), PktSort);
                // Filter spurious BLE responses
                jdx_delta = 0;

                // Loop over packets which are at least 100ms old
                for (jdx = 0; jdx < jdx_end; jdx++) {
                    // Save age of this packet -
                    delta = bsort[jdx].delta;
                    if (delta > 500000)
                        jdx_delta = jdx + 1; // Older than 125ms
                    // Only interested in BLE packets
                    if (bsort[jdx].typ != BLEpkt)
                        continue;
                    // Loop over more recent packets
                    for (kdx = jdx + 1; kdx < jdx_end; kdx++) {
                        // Only interested in BLE packets; filtered BLE packets are now SPURIOUS
                        if (bsort[kdx].typ != BLEpkt)
                            continue;
                        // Difference in age must be less than 1.5us
                        if ((delta - bsort[kdx].delta) > 6)
                            break;
                        idx_j = bsort[jdx].idx;
                        idx_k = bsort[kdx].idx;
                        // Check same or inverted access address
                        xj = pkt[idx_j].x[2];
                        xk = pkt[idx_k].x[2];
                        if ((xj != xk) && (xj != (~xk)))
                            continue;
                        is_tst = (xj == 0x71764129) && (xk == 0x71764129);
                        // Check same modulation scheme
                        xj = pkt[idx_j].x[0];
                        xk = pkt[idx_k].x[0];
                        if ((xj & 0x000000C0) != (xk & 0x000000C0))
                            continue;
                        // Check different channels
                        if ((xj & 0x0000003F) == (xk & 0x0000003F)) {
                            // Same channel, so check whether duplicate ICO first packet
                            if ((pkt[idx_j].x[1] >> 24) != 3)
                                continue;
                            if ((pkt[idx_k].x[1] >> 24) != 3)
                                continue;
                            // Arbitrarily discard one of these
                            buff[idx_j].typ = SPURIOUS;
                            buff[idx_j].occupancy = empty;
                            bsort[jdx].typ = SPURIOUS;
                            break; // j packet removed, skip to next j
                        }
                        // Check rssi difference of at least 35dB
                        rssi_j = ((xj >> 8) & 255);
                        if (rssi_j >= 128)
                            rssi_j -= 256;
                        rssi_k = ((xk >> 8) & 255);
                        if (rssi_k >= 128)
                            rssi_k -= 256;
                        // Cannot use CRC check on test packets since there is no whitening so CRC
                        // is channel independent - it can be used if the spurious packet is
                        // inverted
                        if ((rssi_j + 25) < rssi_k) { // Reject
                            // Only reject if CRC is bad or both are test pkts
                            if ((!(pkt[idx_j].x[buff[idx_j].len - 3] & 0x80000000)) || is_tst) {
                                buff[idx_j].typ = SPURIOUS;
                                buff[idx_j].occupancy = empty;
                                bsort[jdx].typ = SPURIOUS;
                                break; // j packet removed, skip to next j
                            }
                        } else if (rssi_j > (rssi_k + 25)) { // Reject k
                            // Only reject if CRC is bad or both are test pkts
                            if ((!(pkt[idx_k].x[buff[idx_k].len - 3] & 0x80000000)) || is_tst) {
                                buff[idx_k].typ = SPURIOUS;
                                buff[idx_k].occupancy = empty;
                                bsort[kdx].typ = SPURIOUS;
                                continue; // k packet removed, skip to next k
                            }
                        }
                    }
                }

                // Only send packets which are at least 125ms old so that we can be reasonably sure
                // that they have been filtered

                jdx_end = jdx_delta;

                // Limit this to a certain number of packets
                // Prevents stalling in this loop when things start backing up
                // That would prevent env / gain messages getting through

                if (jdx_end > 1024)
                    jdx_end = 1024;

                for (jdx = 0; jdx < jdx_end; jdx++) {
                    idx_tst = bsort[jdx].idx;
                    num = buff[idx_tst].len;
                    typ = bsort[jdx].typ;

                    // Might be better to have an array indexed by typ

                    switch (typ) {

                    case BLEpkt:
                        Send = SendBle;
                        break;

                    case FMpkt:
                        Send = SendFM;
                        break;

                    case WHITELISTpkt:
                        Send = SendWLup;
                        break;

                    case SPECTRUMpkt:
                        Send = SendSpec;
                        break;

                    case GAINpkt:
                        Send = SendGain;
                        break;

                    case DIGITALpkt:
                        Send = SendDIO;
                        break;

                    case UART0pkt:
                        Send = SendU0;
                        break;

                    case UART1pkt:
                        Send = SendU1;
                        break;

                    case UART2pkt:
                        Send = SendU2;
                        break;

                    case UART3pkt:
                        Send = SendU3;
                        break;

                    case SPURIOUS:
                        Send = false;
                        break;

                    default:
                        Send = false;
                        printf("USBdataThread : Sending unknown packet type @ index %d\r\n",
                               idx_tst);
                        callback(nullptr, 0, LAPIS_ERROR_UNKNOWN_PACKET);
                    }

                    if (Send) {

                        nbuff = num / 256;

                        if (nbuff >= 9) {
                            printf("USBdataThread : nbuff = %d, num = %d, idx = %d\r\n", nbuff, num,
                                   idx_tst);
                            callback(nullptr, 0, LAPIS_ERROR_PACKET_OVERFLOW);
                            nbuff = 8;
                        }

                        idx_buf = idx_tst;
                        idx_buf += nbuff;

                        if (idx_buf < idx_tst) {
                            idx_buf = idx_tst;
                            for (k = 0; k <= nbuff; k++) {
                                memcpy(&(wrap_buffer[1024 * k]), (char*)&(pkt[idx_buf]), 1024);
                                idx_buf++;
                            }
                            write_ok = DataWrite((uint8_t*)wrap_buffer, 4 * (num + 1));
                        } else {
                            write_ok = DataWrite((uint8_t*)&(pkt[idx_tst]), 4 * (num + 1));
                        }

                        if (write_ok) {
                            buff[idx_tst].occupancy = empty;
                            NumPktsPending--;
                        } else {
                            break;
                        }

                    } else {
                        buff[idx_tst].occupancy = empty;
                        NumPktsPending--;
                    }
                }
            }

            // Need to update even if nothing sorted since could have got LE packets to update
            // WL which are not then sent on to host

            while (idx_last != idx) {
                if (buff[idx_last].occupancy == full)
                    break;
                pkt[idx_last].valid.hdr = 0xFFFFFFFF;
                idx_last++;
            }
            if (idx_last == idx)
                idx_last = idx - 1;

            // Update sw pointer

            arm_rd = idx_last;
        } catch (...) {
            if (callback)
                callback(nullptr, 0, handle_errors());
        }
    }
} catch (...) {
    if (callback)
        callback(nullptr, 0, handle_errors());
}

//_____________________________________________________________________________
//
//	void	ConnectionRequest( uint16_t idx )
//
//	Handles BLE connection request processing
//_____________________________________________________________________________

void ConnectionRequest(uint16_t idx) {

    uint16_t hdr;
    uint16_t chan;
    uint16_t code;
    uint8_t CRtst[18];
    uint32_t k;
    uint8_t* x;
    int res;
    uint32_t aa;
    uint32_t aanew;

    if (!AutoPruneConnRqst) {
        nCRobs = 0;
        return;
    }

    if (pkt[idx].x[2] != 0x8E89BED6)
        return; // Check advertising address

    hdr = (uint16_t)(pkt[idx].x[3] & 0x0000FFFF);

    if ((hdr &= 0xFF3F) != 0x2205)
        return; // Check PDU type, reserved bits and packet length

    chan = (uint16_t)(pkt[idx].x[0] & 0x0000003F);
    code = (uint16_t)((pkt[idx].x[0] >> 6) & 3);

    if ((chan != 0) && (chan != 12) && (chan != 39)) { // Check for sec adv chan
        if ((code & 1) == 0)
            return; // Only coded phy on sec adv chans
    } else {
        if (code != 0)
            return; // Only 1Mbps uncoded on pri adv chans
    }

    memcpy(CRtst, (char*)&(pkt[idx].x[3]), 18); // Copy header, init addr, adv addr & access addr
    CRtst[0] &= 0xC0;                           // Mask TxAdd and RxAdd fields in header
    CRtst[1] = 0;

    memcpy(&aanew, CRtst + 14, 4); // New access address

    // Test against previous entries

    x = CRlst;
    for (k = 0; k < nCRobs; k++) {

        res = memcmp(CRtst, x, 14); // Test init and adv addresses
        x += 20;
        if (res)
            continue;

        memcpy(&aa, (x - 6), 4); // Recover access address used in previous connection

        if (aa != aanew) { // Check that access address has changed

            uint32_t seed = 0;
            memcpy(&seed, CRtst + 18, 3); // New CRC seed

            // Remove any CIS associated with old connection

            for (int kk = 0; kk < NumEu; kk++) {
                if ((EuAA[kk] != 0) && (EuTYP[kk] == 3)) {
                    if (EuCRC[kk] == seed) {
                        WhitelistSub(EuAA[kk]);
                        EuAA[kk] = 0;
                    }
                }
            }

            WhitelistSub(aa); // Remove previous access address
        }

        memmove((x - 20), x, (nCRobs - k - 1) * 20); // Shuffle down other entries

        nCRobs--; // Update number of entries
    }

    if (nCRobs == 256) {
        memmove(CRlst, (CRlst + 20), 255 * 20); // If list is full need to keep old items at start
        nCRobs = 255;
    }

    memcpy((CRlst + 20 * nCRobs), CRtst, 18);

    nCRobs++;
}

//_____________________________________________________________________________
//
//	void	EuthanasiaUpdate( uint32_t aa , uint64_t pkt_time , uint64_t now_time ,
//                            uint32_t pktTyp , uint32_t crc , uint32_t len , uint32_t chan.
//                            uint32_t phy , int8_t rssi )
//
//	Updates euthanasia list for newly received packet
//_____________________________________________________________________________

void EuthanasiaUpdate(uint32_t aa, uint64_t pkt_time, uint64_t now_time, uint32_t pktTyp,
                      uint32_t crc, uint32_t len, uint32_t chan, uint32_t phy, int8_t rssi) {

    int i;
    for (i = 0; i < NumEu; i++) {
        if (EuAA[i] == aa) {
            EuTT[i] = pkt_time;
            if ((pktTyp == 3) || (pktTyp == 4)) {
                if (crc && len)
                    EuCRC[i]++;
                else
                    EuBAD[i]++;
                if (EuCRC[i])
                    return;
                if ((EuBAD[i] > NumBadCRC) && AutoPruneCRC) {
                    WhitelistSub(aa);
                    EuAA[i] = 0;
                }
            } else {
                if (crc)
                    EuCRC[i]++;
            }
            return;
        }
    }

    if ((pktTyp == 3) || (pktTyp == 4)) {
        if ((len == 0) || (crc == 0))
            return; // For 1/2 search & ICO don't re-enter, the 1st packet should have non-zero
                    // length & correct crc
    }

    int k = 0;
    uint64_t t = pkt_time;
    for (i = 0; i < NumEu; i++) {
        if (EuAA[i] == 0) {
            EuAA[i] = aa;
            EuTT[i] = pkt_time;
            EuCRC[i] = 0; // Set both to zero since first ISO will always be valid
            EuBAD[i] = 0;
            EuTYP[i] = pktTyp;
            return;
        } else if (EuTT[i] < t) {
            k = i;
            t = EuTT[i];
        }
    }
    EuAA[k] = aa;
    EuTT[k] = pkt_time;
    EuCRC[k] = 0; // Set both to zero since first ISO will always be valid
    EuBAD[k] = 0;
    EuTYP[k] = pktTyp;
}

//_____________________________________________________________________________
//
//	void	EuthanasiaKill( uint64_t tnow )
//
//	Deletes old entries
//_____________________________________________________________________________

void EuthanasiaKill(uint64_t tnow) {
    for (int k = 0; k < NumEu; k++) {
        if (EuAA[k] != 0) {
            // TBD should this be extended to all manually entered?
            if (EuTYP[k] != 7) {
                if ((tnow - EuTT[k]) > TimeToLive) {
                    for (int kk = 0; kk < NumEu; kk++) {
                        if ((EuAA[kk] != 0) && (EuTYP[kk] == 3)) {
                            if (EuCRC[kk] == EuCRC[k]) {
                                WhitelistSub(EuAA[kk]);
                                EuAA[kk] = 0;
                            }
                        }
                    }
                    WhitelistSub(EuAA[k]);
                    EuAA[k] = 0;
                }
            }
        }
    }
}

//_____________________________________________________________________________

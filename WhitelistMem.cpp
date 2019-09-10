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
 * FILE:        WhitelistMem.c
 *
 * DESCRIPTION: Maps whitelist into user space
 *
 *****************************************************************************/

//_____________________________________________________________________________
//
// Includes
//_____________________________________________________________________________

#include	"FPGAregs.h"	
#include	"WhitelistMem.h"
#include    "gromitinterface.h"
#include    "registers.h"
#include    "USBdata.h"

#include    <mutex>
#include    <thread>

using namespace std::chrono_literals;

std::mutex wl_mutex;
bool SortAge = true;

static uint32_t WhitelistBase(uint32_t addr) {
    int n = 0;
    gromit_regs.whitelist = addr;
    while( (n < 1000) & (gromit_regs.whitelist != addr) ) {n++;}
    if( n == 1000 ) printf( "Whitelist read failed %08X\n" , addr);
    return gromit_regs.whitelist_data;
}
static uint32_t WhitelistBaseFlags(uint32_t addr) {
    int n = 0;
    addr += 64;
    gromit_regs.whitelist = addr;
    while( (n < 1000) & (gromit_regs.whitelist != addr) ) {n++;}
    if( n == 1000 ) printf( "Whitelist read failed %08X\n",addr);
    return gromit_regs.whitelist_data;
}
static uint32_t WhitelistBaseAge(uint32_t addr) {
    int n = 0;
    addr += 128;
    gromit_regs.whitelist = addr;
    while( (n < 1000) & (gromit_regs.whitelist != addr) ) {n++;}
    if( n == 1000 ) printf( "Whitelist read failed %08X\n",addr);
    return gromit_regs.whitelist_data;
}

//_____________________________________________________________________________
//
// Prototypes
//_____________________________________________________________________________

static	void	WhitelistCmd( uint32_t cmd , uint32_t aa , uint32_t seed , uint32_t flags );
static	int	WhitelistSort( const void *a , const void *b );

//_____________________________________________________________________________
//
//	int	WhitelistSort( const void *a , const void *b )
//
//	Sorts access addresses
//
//_____________________________________________________________________________

static	int	WhitelistSort( const void *a , const void *b )
{
    uint32_t	*aa;
    uint32_t	*bb;

    aa = (uint32_t *) a;
    bb = (uint32_t *) b;

    if( SortAge ) {
        if( aa[2] < bb[2] ) return (-1);
        if( aa[2] > bb[2] ) return (1);
    }
    else {
        if( aa[0] < bb[0] ) return (-1);
        if( aa[0] > bb[0] ) return (1);
    }

    return	(0);

}

//_____________________________________________________________________________
//
//	uint32_t	WhitelistGet( uint32_t *WhitelistEntries , uint8_t *WhitelistBits )
//
//	Gets ordered list of whitelist entries
//
//_____________________________________________________________________________

uint32_t	WhitelistGet( uint32_t *WhitelistEntries )
{
    uint32_t	n;
    uint32_t	k;
    uint32_t	aa;
    uint32_t	vl;

    wl_mutex.lock();

    n = 0;
    for( k = 0 ; k < 60 ; k++ ) {
        aa = WhitelistBaseFlags(k);
        vl = aa & 0x1E000000;
        if( vl ) {
            WhitelistEntries[n] = WhitelistBase(k);
            n++;
            WhitelistEntries[n] = ( aa >> 24 ) | ( aa << 8 );  // Rearrange so flags are at bottom
            n++;
            WhitelistEntries[n] = WhitelistBaseAge(k);
            n++;
        }
    }

    wl_mutex.unlock();

    n = n / 3;

    qsort( WhitelistEntries , n , 12 , WhitelistSort );

    uint32_t m = 0;
    for( k = 0 ; k < 3*n ; k += 3 ) {
        WhitelistEntries[m] = WhitelistEntries[k];
        m++;
        WhitelistEntries[m] = WhitelistEntries[k+1];
        m++;

        int typ = (WhitelistEntries[k+1] >> 5) & 7;
        bool found = false;
        for( int j = 0 ; j < NumEu ; j++ ) {
            if( EuAA[j] == WhitelistEntries[k] ) {
                found = true;
                break;
            }
        }

        if( ! found ) {
            uint64_t t = 0xFFFFFFFFFFFFFFFFull;
            int empty = -1;
            int oldest = -1;
            for( int j = 0 ; j < NumEu ; j++ ) {
                if( EuAA[j] == 0 ) {
                    empty = j;
                    break;
                }
                if( (EuTT[j] < t) && (EuTYP[j] != 7) ) {
                    t = EuTT[j];
                    oldest = j;
                }
            }
            if( empty < 0 ) empty = oldest;
            if( empty >= 0 ) {
                EuAA[empty] = WhitelistEntries[k];
                EuTT[empty] = tkeep;
                EuCRC[empty] = 0;
                EuBAD[empty] = 0;
                EuTYP[empty] = typ;
            }
        }
    }

    return	(n);
}

//_____________________________________________________________________________
//
//	uint8_t	WhitelistFlags( uint32_t addr )
//
//	Returns the flags associated with a whitelist entry
//
//_____________________________________________________________________________

uint8_t	WhitelistFlags( uint32_t addr )
{
    int		k;
    uint32_t	aa;
    uint32_t	vl;
    uint8_t		flags;
    bool		found;

    found = false;

    wl_mutex.lock();

    for( k = 0 ; k < 60 ; k++ ) {
        aa = WhitelistBase(k);
        if( aa == addr ) {
            aa = WhitelistBaseFlags(k);
            vl = aa & 0x1E000000;
            if( vl ) {
                flags = (aa >> 24);
                found = true;
                break;
            }
        }
    }

    wl_mutex.unlock();

    if( found ) return (flags);

    return	(0x80);
}

//_____________________________________________________________________________
//
//	uint32_t	WhitelistCRC( uint32_t addr )
//
//	Returns the CRC seed associated with a whitelist entry
//
//_____________________________________________________________________________

uint32_t	WhitelistCRC( uint32_t addr )
{
    int		k;
    uint32_t	aa;
    uint32_t	vl;
    uint32_t	seed;
    bool		found;

    found = false;

    wl_mutex.lock();

    for( k = 0 ; k < 60 ; k++ ) {
        aa = WhitelistBase(k);
        if( aa == addr ) {
            aa = WhitelistBaseFlags(k);
            vl = aa & 0x1E000000;
            if( vl ) {
                seed = aa & 0xFFFFFF;
                found = true;
                break;
            }
        }
    }

    wl_mutex.unlock();

    if( found ) return (seed);

    return	(0xFFFFFFFF);
}

//_____________________________________________________________________________
//
//	void		WhitelistCmd( uint32_t cmd , uint32_t aa , uint32_t seed , unit8_t flags )
//
//	Executes a whitelist command
//
//_____________________________________________________________________________

static	void	WhitelistCmd( uint32_t cmd , uint32_t aa , uint32_t seed , uint32_t flags )
{
    uint32_t	k;
    uint32_t	done;

    k = 0;
    while( 1 ) {

        done = (gromit_regs.app_status & 4);

        if( done ) break;
        printf( "Waiting for previous whitelist operation to complete ...\r\n" );
        std::this_thread::sleep_for(10ms);
        k++;
        if( k == 100 ) {
            throw WhitelistTimeout("Timed out waiting for previous whitelist operation.");
            break;
        }
    }

    gromit_regs.wl_aa   = aa;
    gromit_regs.wl_crc_seed = (seed & 0x00FFFFFF) | (flags << 24);

    bitfield<GROMIT_CTRL> val = gromit_regs.ctrl;
    val &= ~(GROMIT_CTRL::WHITELIST_OPCODE | GROMIT_CTRL::WHITELIST_CTRL);
    val |= cmd << GROMIT_CTRL::WHITELIST_OPCODE;
    gromit_regs.ctrl = val;
    gromit_regs.ctrl = val | GROMIT_CTRL::WHITELIST_CTRL;
    gromit_regs.ctrl = val;

    k = 0;
    done = 0;
    while( 1 ) {

        done = (gromit_regs.app_status & 4);

        if( done ) break;
        printf( "Waiting for whitelist operation to complete ...\r\n" );
        std::this_thread::sleep_for(10ms);
        k++;
        if( k == 100 ) {
            throw WhitelistTimeout("Timed out waiting for whitelist operation to complete.");
            break;
        }
    }

}

//_____________________________________________________________________________
//
//	void		WhitelistAdd( uint32_t aa , uint32_t seed , uint32_t flags )
//
//	Add access address to whitelist
//
//_____________________________________________________________________________

void	WhitelistAdd( uint32_t aa , uint32_t seed , uint32_t flags )
{

    WhitelistCmd( 1 , aa , seed , flags );

    //	printf( "Access address %08X added to whitelist\r\n" , aa );

}

//_____________________________________________________________________________
//
//	void		WhitelistSub( uint32_t aa )
//
//	Remove access address to whitelist
//
//_____________________________________________________________________________

void	WhitelistSub( uint32_t aa )
{

    WhitelistCmd( 2 , aa , 0 , 0 );

    //	printf( "Access address %08X removed from whitelist\r\n" , aa );

}

//_____________________________________________________________________________
//
//	void		WhitelistTrig( uint32_t aa , bool trig )
//
//	Set trigger output
//
//_____________________________________________________________________________

void	WhitelistTrig( uint32_t aa , bool trig )
{

    if( trig ) {
        WhitelistCmd( 6 , aa , 0 , 0 );
        //		printf( "Access address %08X now has digital output trigger\r\n" , aa );
    }
    else {
        WhitelistCmd( 5 , aa , 0 , 0 );
        //		printf( "Access address %08X no longer has digital output trigger\r\n" , aa );
    }

}

//_____________________________________________________________________________
//
//	void		WhitelistClear( void )
//
//	Clears all entries in the whitelist
//
//_____________________________________________________________________________

void	WhitelistClear( void )
{
    int		k;
    uint32_t	aa;
    uint32_t	vl;

    wl_mutex.lock();

    for( k = 0 ; k < 60 ; k++ ) {
        aa = WhitelistBaseFlags(k);
        vl = aa & 0x1E000000;
        if( vl ) {
            aa = WhitelistBase(k);
            WhitelistSub( aa );
        }
    }

    wl_mutex.unlock();
}

//_____________________________________________________________________________
//
//	void		WhitelistUpdateConn( bool conn )
//
//	Allow connection requests to update whitelist
//
//_____________________________________________________________________________

void	WhitelistUpdateConn( bool conn )
{

    if( conn )
        gromit_regs.ctrl |= GROMIT_CTRL::WL_UPDATE_CON_RQT;
    else
        gromit_regs.ctrl &= ~GROMIT_CTRL::WL_UPDATE_CON_RQT;
}

//_____________________________________________________________________________
//
//	void		WhitelistUpdatePeriodic( bool conn )
//
//	Allow AUX_ADV_IND & AUX_SCAN_RESP to update whitelist
//
//_____________________________________________________________________________

void	WhitelistUpdatePeriodic( bool per )
{

    if( per )
        gromit_regs.ctrl |= GROMIT_CTRL::WL_UPDATE_PERIODIC_ADV;
    else
        gromit_regs.ctrl &= ~GROMIT_CTRL::WL_UPDATE_PERIODIC_ADV;
}

//_____________________________________________________________________________
//
//	void		WhitelistUpdateICl( bool icl )
//
//	Allow ISO connectionless baseaddresses to be added to the whitelist
//
//_____________________________________________________________________________

void	WhitelistUpdateICl( bool icl )
{
    if( icl )
        gromit_regs.ctrl |= GROMIT_CTRL::WL_UPDATE_ICL;
    else
        gromit_regs.ctrl &= ~GROMIT_CTRL::WL_UPDATE_ICL;
}

//_____________________________________________________________________________
//
//	void		WhitelistUpdateArm( bool arm )
//
//	Allow ARM to update whitelist
//
//_____________________________________________________________________________

void	WhitelistUpdateArm( bool arm )
{

    if( arm )
        gromit_regs.ctrl |= GROMIT_CTRL::WL_UPDATE_ARM;
    else
        gromit_regs.ctrl &= ~GROMIT_CTRL::WL_UPDATE_ARM;
}

//_____________________________________________________________________________
//
//	void		WhitelistUpdateSearch12( bool search )
//
//	Allow 1Mbps & 2Mbps search engines to update whitelist
//
//_____________________________________________________________________________

void	WhitelistUpdateSearch12( bool search )
{

    if( search )
        gromit_regs.ctrl |= GROMIT_CTRL::WL_UPDATE_SEARCH_12;
    else
        gromit_regs.ctrl &= ~GROMIT_CTRL::WL_UPDATE_SEARCH_12;
}

//_____________________________________________________________________________
//
//	void		WhitelistUpdateSearchLR( bool search )
//
//	Allow long range detections to update whitelist
//
//_____________________________________________________________________________

void	WhitelistUpdateSearchLR( bool search )
{

    if( search )
        gromit_regs.ctrl |= GROMIT_CTRL::WL_UPDATE_SEARCH_LR;
    else
        gromit_regs.ctrl &= ~GROMIT_CTRL::WL_UPDATE_SEARCH_LR;
}

//_____________________________________________________________________________
//
//	void		WhitelistUpdateICO( bool ico )
//
//	Allow ICO connectionless addresses to be added to the whitelist
//
//_____________________________________________________________________________

void	WhitelistUpdateICO( bool ico )
{
    if( ico )
        gromit_regs.ctrl |= GROMIT_CTRL::WL_UPDATE_ICO;
    else
        gromit_regs.ctrl &= ~GROMIT_CTRL::WL_UPDATE_ICO;
}
//_____________________________________________________________________________

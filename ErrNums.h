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
 * PROJECT:     Moreph
 *
 * FILE:        ErrNums.h
 *
 * DESCRIPTION: Defintions of error numbers
 *
 *****************************************************************************/

//_____________________________________________________________________________

#ifndef	ERRNUMS_H

#define	ERRNUMS_H

#define	e_ADCWait				(0)
#define	e_AttenWait				(1)
#define	e_ClkGenWait			(2)
#define	e_ClkGenLock			(3)
#define	e_mtd_get_partition_data (4)
#define	e_mtd_erase_partition	(5)
#define	e_mtd_write_partition	(6)
#define	e_mtd_crc_partition		(7)
#define	e_mtd_update_partition	(8)
#define	e_mtd_read_partition	(9)
#define	e_mtd_read_boot_block	(10)
#define	e_mtd_update_boot_block	(11)
#define	e_FPGAVersion			(12)
#define	e_FPGAtime				(13)
#define	e_LicenceFriendlyName	(14)
#define	e_LicenceLoad			(15)
#define	e_SynthWait				(16)
#define	e_SynthLock				(17)
#define	e_USBconThread			(18)
#define	e_USBctrlWrite			(19)
#define	e_DoRun					(20)
#define	e_DoClrLicenceKey		(21)
#define	e_DoFeatures			(22)
#define	e_DoTime				(23)
#define	e_USBctrlRead			(24)
#define	e_USBctrlOpen			(25)

#define e_Synth2870Init			(26)
#define	e_Synth2870Lock			(27)
#define	e_Synth2870RF			(28)
#define	e_Synth2870PD			(29)
#define	e_Synth2870Tune			(30)
#define	e_Synth2870Pwr			(31)
#define	e_Synth2870Atten		(32)
#define	e_Synth2870AttenWait	(33)
#define	e_Synth2870Notch		(34)
#define	e_Synth2870Wait			(35)

#define	e_DoFileMoveToRam		(60)
#define e_Mixer					(61)
#define e_DoPwrDn				(62)
#define e_DoReset				(63)
#define e_DoExit				(64)
#define e_LEDThread				(65)
#define e_PowerDownSafe			(66)
#define	e_USBctrlStop			(73)
#define	e_USBctrlRestart		(74)
#define	e_SCPIctrlWrite			(75)
#define	e_SCPIctrlRead			(76)
#define	e_SCPIctrlOpen			(77)
#define	e_SCPIctrlStop			(78)
#define	e_SCPIctrlRestart		(79)
#define	e_SCPIctrlParse			(80)
#define	e_DoSystErrNext			(81)
#define	e_DoSystTime			(82)
#define	e_DoSystDate			(83)
#define	e_DoProgStat			(84)
#define	e_SCPIctrlBlockData		(85)
#define	e_SCPIctrlToken			(86)
#define	e_SCPIctrlGPIB			(86)

#define	e_CtrlRead				(87)
#define	e_NativeCtrlWrite		(88)
#define	e_NativeCtrlRead		(89)
#define	e_NativeCtrlOpen		(90)
#define	e_NativeCtrlStop		(91)
#define	e_NativeCtrlRestart		(92)
#define	e_NativeDataWrite		(93)
#define	e_NativeDataListen		(94)

#define	e_xADC_stats			(100)
#define	e_EnvMon				(101)
#define	e_xADC_ext				(104)

#define	e_FPGAPrintRegs			(102)

#define	e_NativeBlockRamWrite	(103)

#define	e_ClkGen10MHz			(105)

#define	e_DACWait				(200)
#define	e_TxAttenWait			(201)
#define	e_DACReset				(202)

#define	e_UIOclear				(203)
#define	e_UIOwait				(204)
#define	e_UIOopen				(205)
#define	e_UIOclose				(206)

#define	e_MsgQueRead			(207)
#define	e_MsgQueWrite			(208)
#define	e_MsgQuePut				(209)
#define	e_MsgQueCount			(210)

#define	e_SearchEngineSet		(26)
#define	e_SearchEngineSetThresholds	(27)
#define	e_SearchEngineGetThresholds	(28)
#define	e_SearchEngineThread		(29)
#define	e_DoWhitelistAlgor		(30)
#define	e_DoAutoFM			(31)
#define	e_DoEnvPeriod			(32)
#define	e_DoWLPeriod			(33)
#define	e_DoSpecPeriod			(34)
#define	e_DoGainPeriod			(35)
#define	e_DoUart			(36)
#define	e_DoDioEdges			(37)
#define	e_DoDioChan			(38)
#define	e_DoDioTimestamp		(39)
#define	e_DoCalibration			(40)
#define	e_DoGain			(41)
#define	e_DoDewhitening			(42)
#define	e_DoSearchParams		(43)
#define	e_DoShortPkts			(44)
#define	e_DoBitErrors			(45)
#define	e_DoWhitelistUpdate		(46)
#define	e_DoWhitelistAddition		(47)
#define	e_DoDataCollectionMask		(48)
#define	e_DoBlockRam			(49)
#define	e_InitTime			(50)
#define	e_USBBlockRamWrite		(51)
#define	e_USBdataWrite			(52)
#define	e_USBdataOpen			(53)
#define	e_USBdataThread			(54)
#define	e_WhitelistPrint		(55)
#define	e_WhitelistGet			(56)
#define	e_WhitelistFlags		(57)
#define	e_WhitelistCRC			(58)
#define	e_WhitelistCmd			(59)

#define	e_DoBuffPeriod			(95)
#define	e_DoTimestamping		(96)
#define	e_DoDIOrw			(97)
#define	e_DoThresholds			(98)
#define	e_DoDigTrig			(99)

#include <cstdint>
#include <cstdlib>
#include <fmt/format.h>
extern bool IntErr;
inline void	InternalError( uint32_t e_routine , uint32_t e_point ) {
    fmt::print_colored(fmt::MAGENTA, "ERROR: {} {}", e_routine, e_point);
    IntErr = true;
    exit(-(int)e_routine);
}

inline void qerror(const char* msg) {
    fmt::print_colored(fmt::MAGENTA, "QERROR: {}", msg);
}

#endif

//_____________________________________________________________________________


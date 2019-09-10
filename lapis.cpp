#include "lapis.h"
#include "FPGAregs.h"
#include "USBdata.h"
#include "WhitelistMem.h"
#include "enums.h"
#include "libusbcpp.h"
#include "registers.h"
#include <atomic>
#include <chrono>
#include <filepackloader.h>
#include <future>
#include <iostream>
#include <morephsearch.h>
#include <thread>
#include <stdio.h>
using namespace std::chrono_literals;

std::unique_ptr<GromitInterface> gromit;
std::atomic_bool data_stop(false);
std::unique_ptr<std::thread> data_thread;
lapis_data_callback_t callback = nullptr;
std::unique_ptr<MorephSearch> ms = nullptr;

static std::vector<lapis_search_result_t> search_results;
static std::vector<MorephTransport> search_results_transports;
static std::string error_msg;

lapis_error_t handle_errors() {
    error_msg.clear();
    try {
        throw;
    } catch (WhitelistTimeout& e) {
        error_msg = e.what();
        return LAPIS_ERROR_WHITELIST_FAILURE;
    } catch (libusb::exception& e) {
        error_msg = e.what();
        return LAPIS_ERROR_USB;
    } catch (ExceptionFormat& e) {
        error_msg = e.what();
        return LAPIS_ERROR_EXCEPTION;
    } catch (std::exception& e) {
        error_msg = e.what();
        return LAPIS_ERROR_SYSTEM_EXCEPTION;
    } catch (...) {
        error_msg = "Unknown exception";
    }
    return LAPIS_ERROR_UNKNOWN;
}

#define HANDLE_ERROR                                                                               \
    catch (...) {                                                                                  \
        return handle_errors();                                                                    \
    }

const char* lapis_error_msg() { return error_msg.c_str(); }

lapis_error_t lapis_search(lapis_search_result_t** results, int* N) try {
    lapis_free_search_results();
    if (!ms) {
        ms = std::make_unique<MorephSearch>([]() {
            try {
                std::rethrow_exception(std::current_exception());
            } catch (std::exception& e) {
                callback(nullptr, 0, handle_errors());
            }
        });
    }
    ms->set_callback([](const MorephDevice& d) {
        if (d.type == RFCEnums::USB) {
            auto m = d.getTransport();
            GromitInterface g(m);
            auto f = g.getFeatures();
            auto serial = g.getSerialNumber();
            if (f.linux.substr(0, 3) == "FX3") {
                search_results_transports.push_back(m);
                search_results.push_back(
                    {reinterpret_cast<lapis_handle_t*>(&search_results_transports.back()), serial});
            }
        }
    });
    ms->search_once();
    if (results)
        *results = search_results.data();
    if (N)
        *N = (int)search_results.size();
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_free_search_results() try {
    search_results.clear();
    search_results_transports.clear();
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

#ifdef _WIN32
static std::string get_library_path() {
    HMODULE m;
    std::string name(1000, 0);
    GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, (LPCTSTR)&get_library_path, &m);
    name.resize(GetModuleFileNameA(m, &name[0], (DWORD)name.size()));
    return name;
}
#else
#ifdef LAPIS_STATIC
#include <unistd.h>
static std::string get_library_path() {
    std::string name(1000, 0);
    auto r = readlink("/proc/self/exe", &name[0], name.size());
    ifError(r <= 0, "Cannot find apppack");
    name.resize(r);
    return name;
}
#else
#include <dlfcn.h>
static std::string get_library_path() {
    Dl_info info;
    if ((dladdr(reinterpret_cast<void*>(&get_library_path), &info) && info.dli_fname)) {
        return std::string(info.dli_fname);
    }
    throw ExceptionFormat("Cannot find shared library name");
}
#endif
#endif

lapis_error_t lapis_disconnect() try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    lapis_set_data_callback(nullptr);
    try {
        gromit->exitApp();
    } catch (...) {
        // Ignore errors here, it probably means the device is unplugged.
    };
    gromit.reset();
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_connect(lapis_handle_t* handle) try {
    if (gromit)
        return LAPIS_ERROR_NO_DEVICE;

    FilePackLoader ldr{get_library_path()};

    auto& mo = *reinterpret_cast<MorephTransport*>(handle);
    gromit = std::make_unique<GromitInterface>(mo);

    mo.setDataCallback(usb_data_callback, 1);
    mo.setDebugCallback([](std::string s) { fmt::print_colored(fmt::YELLOW, s); });

    fmt::print_colored(fmt::CYAN, "Load App\n");
    gromit->download(ldr.getFile("gromit.rfcapp"));
    gromit->executeFileInRam();

    fmt::print_colored(fmt::CYAN, "Load FPGA\n");
    {
        std::promise<void> pr;
        mo.writeData(ldr.getFile("MorephLP_Gromit.bit.bin"), [&pr]() { pr.set_value(); });
        pr.get_future().get();
        gromit->load_fpga(ldr.getFile("MorephLP_Gromit.bit.bin.sig"));
        fmt::print_colored(fmt::CYAN, "FPGA Loaded\n");
    }

    // Print FPGA version

    FPGAVersion();

    // Terminate tx path, enable lna, put lna into circuit

    hw.ctrl |= HW_CTRL::tx_amp_sw;

    // Maximum gain, AGC on

    lapis_set_gain(0, 1, 1);

    // Set calibration factors

    lapis_set_calibration(284, 58);

    // Set thresholds

    lapis_set_detection_threshold(-100, -125, -90);
    uint8_t BitErr[8] = {1, 1, 6, 4, 0, 0, 0, 0};
    lapis_set_correlator_bit_errors(BitErr);

    // Reporting intervals

    lapis_set_spec_period(4095);
    lapis_set_gain_period(10);
    lapis_set_wl_period(100, 1);
    lapis_set_buffer_period(10, 20, 80);

    // Set digital functions
    // Configure as logic analyser on all pins
    // External IO voltage

    lapis_set_dio(0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0);
    lapis_set_uart(115200, 8, 0, 1, 0);
    lapis_set_uart(115200, 8, 0, 1, 1);
    lapis_set_uart(115200, 8, 0, 1, 2);
    lapis_set_uart(115200, 8, 0, 1, 3);

    // Whitelist algorithm

    lapis_set_whitelist_algorithm(0x7, 100, 50);

    // Enable whitelist updates

    lapis_set_whitelist_mask(0x7F);

    // Set capture mask

    lapis_set_capture_mask(0x7FD);
    //    lapis_set_capture_mask( 0x000 );

    // Make sure advertising access address is present

    lapis_whitelist_add(0x8E89BED6, 0x00555555, 0xFE);

    // CheckADCPhase();

    gromit_regs.ctrl |= GROMIT_CTRL::ENABLE_1M;
    std::this_thread::sleep_for(500ms);
    gromit_regs.ctrl |= GROMIT_CTRL::ENABLE_2M;
    std::this_thread::sleep_for(100ms);

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_data_callback(lapis_data_callback_t cb) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    callback = cb;
    if (data_thread) {
        data_stop = true;
        data_thread->join();
        data_thread.reset();
    }
    if (cb) {
        // Start USB data thread
        data_stop = false;
        data_thread = std::make_unique<std::thread>(USBdataThread);
    }

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_friendly_name(char* name, unsigned maxlen) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (!name)
        return LAPIS_ERROR_MISSING_ARGUMENT;
    auto n = gromit->getFriendlyName();
    strncpy(name, n.c_str(), n.length() + 1);
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_manufacturing_string(char* name, unsigned maxlen) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (!name)
        return LAPIS_ERROR_MISSING_ARGUMENT;
    auto n = gromit->getManufacturingString();
    strncpy(name, n.c_str(), n.length() + 1);
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t morpehlp_serial_number(uint32_t* serial) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (!serial)
        return LAPIS_ERROR_MISSING_ARGUMENT;
    *serial = gromit->getSerialNumber();
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_detection_threshold(int8_t rssi_uc, int8_t rssi_c, int8_t search) try {
    uint32_t uc;
    uint32_t c;
    uint32_t s;

    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if (rssi_uc < 0)
        uc = ((int32_t)rssi_uc) + 256;
    else
        uc = rssi_uc;

    if (rssi_c < 0)
        c = ((int32_t)rssi_c) + 256;
    else
        c = rssi_c;

    if (search < 0)
        s = ((int32_t)search) + 256;
    else
        s = search;

    gromit_regs.RSSIthresh = (uc & 0xFF) | ((c & 0xFF) << 8) | ((s & 0xFF) << 16);

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_detection_threshold(int8_t* rssi_uc, int8_t* rssi_c, int8_t* search) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if ((!rssi_uc) || (!rssi_c) || (!search))
        return LAPIS_ERROR_MISSING_ARGUMENT;
    uint32_t val = gromit_regs.RSSIthresh;
    *rssi_uc = (int8_t)((uint8_t)(val & 0xFF));
    *rssi_c = (int8_t)((uint8_t)((val >> 8) & 0xFF));
    *search = (int8_t)((uint8_t)((val >> 16) & 0xFF));
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_whitelist_algorithm(uint16_t mask, uint16_t ttl, uint16_t nBad) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if (mask & 1)
        AutoPruneConnRqst = true;
    else
        AutoPruneConnRqst = false;

    if (mask & 2)
        AutoPruneAge = true;
    else
        AutoPruneAge = false;

    if (mask & 4)
        AutoPruneCRC = true;
    else
        AutoPruneCRC = false;

    TimeToLive = 400000ull * (uint64_t)ttl;

    NumBadCRC = nBad;

    bitfield<GROMIT_CTRL> val = gromit_regs.ctrl;

    if (mask)
        val |= GROMIT_CTRL::CAPTURE_PKT;
    else if (!SendBle)
        val &= ~GROMIT_CTRL::CAPTURE_PKT;

    gromit_regs.ctrl = val;
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_whitelist_algorithm(uint16_t* mask, uint16_t* ttl, uint16_t* nBad) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if ((!mask) || (!ttl))
        return LAPIS_ERROR_MISSING_ARGUMENT;

    *mask = (AutoPruneConnRqst ? 1 : 0);
    *mask |= (AutoPruneAge ? 2 : 0);
    *mask |= (AutoPruneCRC ? 4 : 0);

    *ttl = (uint16_t)(TimeToLive / 400000ull);
    *nBad = (uint16_t)NumBadCRC;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_buffer_period(uint16_t period, uint16_t low_water_,
                                      uint16_t high_water) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if (low_water_ >= hig_water)
        return LAPIS_ERROR_INVALID_ARGUMENT;
    if (low_water_ > 100)
        return LAPIS_ERROR_INVALID_ARGUMENT;
    if (high_water > 100)
        return LAPIS_ERROR_INVALID_ARGUMENT;

    if (period > 100) {
        tbuf_int = 10s;
    } else {
        tbuf_int = std::chrono::milliseconds(((uint32_t)period) * 100);
    }

    tbuf = std::chrono::system_clock::now(); // Reset reporting when updating interval

    low_water = low_water_;
    hig_water = high_water;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_buffer_period(uint16_t* period, uint16_t* low_water_,
                                      uint16_t* high_water) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if ((!period) || (!low_water) || (!high_water))
        return LAPIS_ERROR_MISSING_ARGUMENT;
    *period =
        (uint16_t)(std::chrono::duration_cast<std::chrono::milliseconds>(tbuf_int).count() / 100);
    *low_water_ = low_water;
    *high_water = hig_water;
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_wl_period(uint16_t period, uint8_t age) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if (period == 0) {
        period = 10;
        SendWhitelist = false;
    } else {
        SendWhitelist = true;
    }

    if (period > 100)
        twl_int = 10s;
    else
        twl_int = std::chrono::milliseconds(((uint32_t)period) * 100);

    twl = std::chrono::system_clock::now(); // Reset reporting when updating interval

    SortAge = (age != 0);

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_wl_period(uint16_t* period, uint8_t* age) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if ((!period) || (!age))
        return LAPIS_ERROR_MISSING_ARGUMENT;
    *period =
        (uint16_t)(std::chrono::duration_cast<std::chrono::milliseconds>(twl_int).count() / 100);
    *age = SortAge ? 1 : 0;
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_spec_period(uint16_t period) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (period >= 16364)
        gromit_regs.spec_period = 0x00003FFF;
    else if (period == 0)
        gromit_regs.spec_period = 0x00000001;
    else
        gromit_regs.spec_period = period;
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_spec_period(uint16_t* period) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (!period)
        return LAPIS_ERROR_MISSING_ARGUMENT;
    *period = (uint16_t)(gromit_regs.spec_period & 0x00003FFF);
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_gain_period(uint16_t period) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (period >= 26)
        gromit_regs.gain_period = 0x0FFFFFFF;
    else if (period == 0)
        gromit_regs.gain_period = 10000000;
    else
        gromit_regs.gain_period = ((uint32_t)period) * 10000000;
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_gain_period(uint16_t* period) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (!period)
        return LAPIS_ERROR_MISSING_ARGUMENT;
    *period = (uint16_t)((gromit_regs.gain_period) / 10000000);
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_features(uint32_t* app, uint32_t* fpga_version, uint32_t* fpga_date,
                             uint32_t* fpga_time, uint32_t* fpga_features, char* linux,
                             unsigned linux_len) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if ((!app) || (!fpga_version) || (!fpga_date) || (!fpga_time) || (!fpga_features) || (!linux))
        return LAPIS_ERROR_MISSING_ARGUMENT;
    auto f = gromit->getFeatures();
    *app = f.app;
    *fpga_version = f.fpga_ver;
    *fpga_date = f.fpga_date;
    *fpga_time = f.fpga_time;
    *fpga_features = f.fpga_feat;

    strncat(linux, f.linux.c_str(), linux_len);
    strncat(linux, " : ", linux_len - strlen(linux));
    strncat(linux, f.moreph.c_str(), linux_len - strlen(linux));
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_calibration(uint16_t offset, uint16_t switch_atten) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    gromit_regs.db_cal = (uint32_t(switch_atten) << 16 | offset) & 0x01FF01FF;
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_calibration(uint16_t* offset, uint16_t* switch_atten) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if ((!offset) || (!switch_atten))
        return LAPIS_ERROR_MISSING_ARGUMENT;
    uint32_t val = gromit_regs.db_cal;
    *offset = val;
    *switch_atten = val >> 16;
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_gain(uint8_t atten, uint8_t agc, uint8_t lna) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    int k;
    bitfield<HW_CTRL> val = hw.ctrl;

    hw.ctrl = val & ~HW_CTRL::agc_enable; // Turn off AGC

    for (k = 0; k < 1000; k++) { // Wait for attenuator control
        if (!(hw.status & HW_STATUS::atten_busy))
            break;
    }

    if (agc) {
        hw.atten_rx = 0;
        hw.ctrl = val | HW_CTRL::agc_enable;
    } else {
        hw.atten_rx = atten & 0x003F;
    }

    if (lna) {
        hw.ctrl &= ~(HW_CTRL::mon_sw);
        hw.ctrl |= (HW_CTRL::tx_sw | HW_CTRL::lna_en);
    } else {
        hw.ctrl |= HW_CTRL::mon_sw;
        hw.ctrl &= ~(HW_CTRL::tx_sw | HW_CTRL::lna_en);
    }

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_gain(uint8_t* atten, uint8_t* agc, uint8_t* lna) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if ((!atten) || (!agc) || (!lna))
        return LAPIS_ERROR_MISSING_ARGUMENT;
    bitfield<HW_CTRL> val = hw.ctrl;
    *atten = hw.atten_rx;
    *agc = (val & HW_CTRL::agc_enable) ? 1 : 0;
    *lna = (val & HW_CTRL::lna_en) ? 1 : 0;
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_correlator_bit_errors(uint8_t* Nerr) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (!Nerr)
        return LAPIS_ERROR_MISSING_ARGUMENT;
    uint32_t val;
    uint8_t x[8];
    memcpy(x, Nerr, 8);

    if (x[0] > 3)
        x[0] = 2;
    if (x[1] > 3)
        x[1] = 2;
    if (x[2] > 7)
        x[2] = 6;
    if (x[3] > 7)
        x[3] = 6;
    if (x[4] > 3)
        x[4] = 2;
    if (x[5] > 3)
        x[5] = 2;
    if (x[6] > 7)
        x[6] = 6;
    if (x[7] > 7)
        x[7] = 6;

    val = x[0] & 3;
    val |= (x[1] & 3) << 4;
    val |= (x[2] & 7) << 8;
    val |= (x[3] & 7) << 12;
    val |= (x[4] & 3) << 16;
    val |= (x[5] & 3) << 20;
    val |= (x[6] & 7) << 24;
    val |= (x[7] & 7) << 28;

    gromit_regs.DemodThresh = val;
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_correlator_bit_errors(uint8_t* Nerr) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (!Nerr)
        return LAPIS_ERROR_MISSING_ARGUMENT;

    uint32_t val = gromit_regs.DemodThresh;

    Nerr[0] = (val & 3);
    Nerr[1] = (val >> 4) & 3;
    Nerr[2] = (val >> 8) & 7;
    Nerr[3] = (val >> 12) & 7;
    Nerr[4] = (val >> 16) & 3;
    Nerr[5] = (val >> 20) & 3;
    Nerr[6] = (val >> 24) & 7;
    Nerr[7] = (val >> 28) & 7;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_whitelist_mask(uint8_t mask) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    bitfield<GROMIT_CTRL> val = gromit_regs.ctrl;
    val &=
        ~(GROMIT_CTRL::WL_UPDATE_CON_RQT | GROMIT_CTRL::WL_UPDATE_ICL | GROMIT_CTRL::WL_UPDATE_ARM |
          GROMIT_CTRL::WL_UPDATE_PERIODIC_ADV | GROMIT_CTRL::WL_UPDATE_SEARCH_12 |
          GROMIT_CTRL::WL_UPDATE_SEARCH_LR | GROMIT_CTRL::WL_UPDATE_ICO);

    if (mask & 0x0001)
        val |= GROMIT_CTRL::WL_UPDATE_CON_RQT; // Connection requests
    if (mask & 0x0002)
        val |= GROMIT_CTRL::WL_UPDATE_ICL; // ICL
    if (mask & 0x0004)
        val |= GROMIT_CTRL::WL_UPDATE_ARM; // ARM / host
    if (mask & 0x0008)
        val |= GROMIT_CTRL::WL_UPDATE_PERIODIC_ADV; // AUX_ADV_IND & AUX_SCAN_RESP
    if (mask & 0x0010)
        val |= GROMIT_CTRL::WL_UPDATE_SEARCH_12; // 1Mbps & 2Mbps search engines
    if (mask & 0x0020)
        val |= GROMIT_CTRL::WL_UPDATE_SEARCH_LR; // long range search engines
    if (mask & 0x0040)
        val |= GROMIT_CTRL::WL_UPDATE_ICO; // ICO

    gromit_regs.ctrl = val;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_whitelist_mask(uint8_t* mask_out) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (!mask_out)
        return LAPIS_ERROR_MISSING_ARGUMENT;

    uint8_t mask = 0;

    bitfield<GROMIT_CTRL> val = gromit_regs.ctrl;

    if (val & GROMIT_CTRL::WL_UPDATE_CON_RQT)
        mask |= 0x0001;
    if (val & GROMIT_CTRL::WL_UPDATE_ICL)
        mask |= 0x0002;
    if (val & GROMIT_CTRL::WL_UPDATE_ARM)
        mask |= 0x0004;
    if (val & GROMIT_CTRL::WL_UPDATE_PERIODIC_ADV)
        mask |= 0x0008;
    if (val & GROMIT_CTRL::WL_UPDATE_SEARCH_12)
        mask |= 0x0010;
    if (val & GROMIT_CTRL::WL_UPDATE_SEARCH_LR)
        mask |= 0x0020;
    if (val & GROMIT_CTRL::WL_UPDATE_ICO)
        mask |= 0x0040;

    *mask_out = mask;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_whitelist_add(uint32_t access_address, uint32_t crc_seed, uint8_t flags) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    bool update = (gromit_regs.ctrl & GROMIT_CTRL::WL_UPDATE_ARM);

    if (!update)
        return LAPIS_ERROR_WHITELIST_BUSY;

    WhitelistAdd(access_address, crc_seed, flags);

    if (WhitelistFlags(access_address) == 0x80)
        return LAPIS_ERROR_WHITELIST_FAILURE;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_whitelist_delete(uint32_t access_address) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    bool update = (gromit_regs.ctrl & GROMIT_CTRL::WL_UPDATE_ARM);

    if (!update)
        return LAPIS_ERROR_WHITELIST_BUSY;

    WhitelistSub(access_address);

    if (WhitelistFlags(access_address) != 0x80)
        return LAPIS_ERROR_WHITELIST_FAILURE;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_whitelist_crc(uint32_t access_address, uint32_t* crc) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (!crc)
        return LAPIS_ERROR_MISSING_ARGUMENT;
    *crc = WhitelistCRC(access_address);
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_whitelist_flags(uint32_t access_address, uint8_t* flags) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    if (!flags)
        return LAPIS_ERROR_MISSING_ARGUMENT;
    *flags = WhitelistFlags(access_address);
    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_whitelist_trig(uint32_t aa, uint8_t on) try {

    WhitelistTrig(aa, (bool)on);

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_whitelist_trig(uint32_t aa, uint8_t* on) try {

    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if (!on)
        return LAPIS_ERROR_MISSING_ARGUMENT;

    uint8_t flags = WhitelistFlags(aa);

    if (flags == 0x80)
        *on = 0;
    else if (flags & 0x01)
        *on = 1;
    else
        *on = 0;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_chan_mask(uint64_t mask) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    gromit_regs.trigger_mask0 = (uint32_t)(mask & 0xFFFFFFFF);
    gromit_regs.trigger_mask1 = (uint32_t)(mask >> 32);

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_chan_mask(uint64_t* mask) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if (!mask)
        return LAPIS_ERROR_MISSING_ARGUMENT;

    *mask = gromit_regs.trigger_mask1;
    *mask <<= 32;
    *mask |= gromit_regs.trigger_mask0;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_capture_mask(uint32_t mask) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    bitfield<GROMIT_CTRL> val = gromit_regs.ctrl;

    val &= ~(GROMIT_CTRL::CAPTURE_PKT | GROMIT_CTRL::CAPTURE_SPECTRUM |
             GROMIT_CTRL::CAPTURE_WHITELIST | GROMIT_CTRL::CAPTURE_GAIN | GROMIT_CTRL::CAPTURE_DIG |
             GROMIT_CTRL::CAPTURE_UART0 | GROMIT_CTRL::CAPTURE_UART1 | GROMIT_CTRL::CAPTURE_UART2 |
             GROMIT_CTRL::CAPTURE_UART3);

    SendBle = (mask & 0x0001) != 0;
    if (SendBle | AutoPruneConnRqst | AutoPruneAge)
        val |= GROMIT_CTRL::CAPTURE_PKT; // Packets
    SendSpec = (mask & 0x0004) != 0;
    if (SendSpec)
        val |= GROMIT_CTRL::CAPTURE_SPECTRUM; // Spectrum
    SendWLup = (mask & 0x0008) != 0;
    if (SendWLup)
        val |= GROMIT_CTRL::CAPTURE_WHITELIST; // Whitelist updates
    SendDIO = (mask & 0x0010) != 0;
    if (SendDIO)
        val |= GROMIT_CTRL::CAPTURE_DIG; // Digital input
    SendU0 = (mask & 0x0020) != 0;
    if (SendU0)
        val |= GROMIT_CTRL::CAPTURE_UART0; // UART #0
    SendU1 = (mask & 0x0040) != 0;
    if (SendU1)
        val |= GROMIT_CTRL::CAPTURE_UART1; // UART #1
    SendU2 = (mask & 0x0080) != 0;
    if (SendU2)
        val |= GROMIT_CTRL::CAPTURE_UART2; // UART #2
    SendU3 = (mask & 0x0100) != 0;
    if (SendU3)
        val |= GROMIT_CTRL::CAPTURE_UART3; // UART #3
    SendGain = (mask & 0x0400) != 0;
    if (SendGain)
        val |= GROMIT_CTRL::CAPTURE_GAIN; // Gain control

    gromit_regs.ctrl = val;

    SendWhitelist = (mask & 0x0200) != 0;
    SendEnv = (mask & 0x0800) != 0;
    SendDebug = (mask & 0x1000) != 0;
    SendSpace = (mask & 0x2000) != 0;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_capture_mask(uint32_t* mask_out) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;
    uint32_t mask = 0;

    if (!mask_out)
        return LAPIS_ERROR_MISSING_ARGUMENT;

    bitfield<GROMIT_CTRL> val = gromit_regs.ctrl;

    if (SendBle)
        mask |= 0x0001;
    if (val & GROMIT_CTRL::CAPTURE_SPECTRUM)
        mask |= 0x0004;
    if (val & GROMIT_CTRL::CAPTURE_WHITELIST)
        mask |= 0x0008;
    if (val & GROMIT_CTRL::CAPTURE_DIG)
        mask |= 0x0010;
    if (val & GROMIT_CTRL::CAPTURE_UART0)
        mask |= 0x0020;
    if (val & GROMIT_CTRL::CAPTURE_UART1)
        mask |= 0x0040;
    if (val & GROMIT_CTRL::CAPTURE_UART2)
        mask |= 0x0080;
    if (val & GROMIT_CTRL::CAPTURE_UART3)
        mask |= 0x0100;
    if (val & GROMIT_CTRL::CAPTURE_GAIN)
        mask |= 0x0400;

    if (SendWhitelist)
        mask |= 0x0200;
    if (SendEnv)
        mask |= 0x0800;
    if (SendDebug)
        mask |= 0x1000;
    if (SendSpace)
        mask |= 0x2000;

    *mask_out = mask;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_dio(uint16_t mask, uint16_t state, uint16_t up, uint16_t dn,
                            uint8_t cfg) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    hw.dio_ctrl = (((uint32_t)mask) << 16) | state;

    gromit_regs.up_dn = (((uint32_t)dn) << 16) | up;

    uint32_t val = hw.ctrl;

    val &= ~0x00002000;
    if (cfg & 4)
        val |= 0x00002000;

    val &= 0xFFFCFFFF;
    val |= (cfg & 3) << 16;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_dio(uint16_t* mask, uint16_t* state, uint16_t* up, uint16_t* dn,
                            uint8_t* cfg) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if ((!mask) || (!state) || (!up) || (!dn) || (!cfg))
        return LAPIS_ERROR_MISSING_ARGUMENT;

    *mask = hw.dio_ctrl >> 16;
    *state = hw.dio_ctrl & 0xFFFF;

    *up = gromit_regs.up_dn & 0xFFFF;
    *dn = gromit_regs.up_dn >> 16;

    uint32_t val = hw.ctrl;

    *cfg = (val >> 16) & 3;
    if (val & 0x00002000)
        *cfg |= 4;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_uart(uint32_t baud, uint8_t databits, uint8_t parity, uint8_t stopbits,
                             uint8_t nUART) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    uint32_t val;

    baud = (100000000 + (baud / 2)) / baud;
    if (databits > 8)
        databits = 8;
    if (parity == 3)
        parity = 0;
    if (stopbits == 3)
        stopbits = 2;

    val = baud | (databits << 16) | (stopbits << 24) | (parity << 28);

    switch (nUART) {

    case 0:
        gromit_regs.uart0 = val;
        break;

    case 1:
        gromit_regs.uart1 = val;
        break;

    case 2:
        gromit_regs.uart2 = val;
        break;

    case 3:
        gromit_regs.uart3 = val;
        break;
    }

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_uart(uint32_t* baud, uint8_t* databits, uint8_t* parity, uint8_t* stopbits,
                             uint8_t nUART) try {
    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if ((!baud) || (!databits) || (!parity) || (!stopbits))
        return LAPIS_ERROR_MISSING_ARGUMENT;

    uint32_t val;
    switch (nUART) {

    case 0:
        val = gromit_regs.uart0;
        break;

    case 1:
        val = gromit_regs.uart1;
        break;

    case 2:
        val = gromit_regs.uart2;
        break;

    case 3:
        val = gromit_regs.uart3;
        break;
    }

    *baud = val & 0xFFFF;
    *baud = (100000000 + (*baud / 2)) / *baud;
    *databits = (val >> 16) & 15;
    *stopbits = (val >> 24) & 3;
    *parity = (val >> 28) & 3;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_timestamping(uint32_t Hz) try {
    uint32_t Ft;
    uint32_t Fa;
    uint32_t A;
    uint32_t An;
    uint32_t B;
    uint32_t Bn;
    uint32_t Best;
    uint32_t Err;
    uint32_t val = 2ul | (1ul << 6) | (3ul << 12) | (1ul << 18);
    uint32_t Denom;

    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if (Hz) {

        Ft = 4000000ul;
        Best = Ft;
        A = Ft / Hz;
        B = A + 1;
        for (An = 1; An < 31; An++) {
            Bn = (An * (Hz - A * Ft));
            Denom = (B * Ft - Hz);
            Bn += (Denom / 2);
            Bn /= Denom;
            if (Bn > 31)
                Bn = 31;
            Fa = ((An + Bn) * Hz); // This is the biggest number in the loop
            Denom = (An * A + Bn * B);
            Fa += (Denom / 2);
            Fa /= Denom;
            if (Ft > Fa)
                Err = Ft - Fa;
            else
                Err = Fa - Ft;
            if (Err < Best) { // Take the 1st one which is zero to minimise jitter
                Best = Err;
                if (Bn == 0)
                    val = A | (An << 6) | (A << 12) | (An << 6) | 0x01000000;
                else
                    val = A | (An << 6) | (B << 12) | (Bn << 6) | 0x01000000;
            }
        }
        val |= 0x01000000; // External timestamp clk
    } else {
        val = 2ul | (1ul << 6) | (3ul << 12) | (1ul << 18);
    }

    hw.clk_ctrl = val | (hw.clk_ctrl & 0xFE000000);

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_timestamping(uint32_t* Hz) try {
    uint32_t Fa;
    uint32_t A;
    uint32_t An;
    uint32_t B;
    uint32_t Bn;
    uint32_t val;
    uint32_t Denom;

    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if (!Hz)
        return LAPIS_ERROR_MISSING_ARGUMENT;

    val = hw.clk_ctrl;

    A = val & 0x3F;
    An = (val >> 6) & 0x3F;
    B = (val >> 12) & 0x3F;
    Bn = (val >> 18) & 0x3F;

    if (val & 0x01000000)
        Fa = *Hz;
    else
        Fa = 10000000;

    Fa = (An + Bn) * Fa;
    Denom = (An * A + Bn * B);
    Fa += (Denom / 2);
    Fa /= Denom;

    *Hz = Fa;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_set_ext_clock(uint8_t flags) try {

    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if ((flags & 1) == 0) {
        hw.ctrl &= (~HW_CTRL::ext_clk_en);
    } else {
        hw.ctrl |= HW_CTRL::ext_clk_en;
        uint32_t val = hw.clk_ctrl;
        if ((flags & 2) == 0)
            val |= 0x04000000;
        if ((flags & 4) == 0)
            val |= 0x08000000;
    }

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

lapis_error_t lapis_get_ext_clock(uint8_t* flags) try {

    if (!gromit)
        return LAPIS_ERROR_NO_DEVICE;

    if (!flags)
        return LAPIS_ERROR_MISSING_ARGUMENT;

    *flags = 0;
    if (hw.ctrl & HW_CTRL::ext_clk_en)
        *flags |= 1;
    uint32_t val = hw.clk_ctrl;
    if (!(val & 0x04000000))
        *flags |= 2;
    if (!(val & 0x08000000))
        *flags |= 4;

    return LAPIS_NO_ERROR;
}
HANDLE_ERROR

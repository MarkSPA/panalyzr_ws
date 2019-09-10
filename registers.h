#ifndef REGISTERS_H
#define REGISTERS_H
#include "gromitinterface.h"

extern std::unique_ptr<GromitInterface> gromit;

enum class HW_CTRL {
    driver_rst,
    driver_en,
    adc_driver_rst,
    agc_enable,
    synth_en,
    lna_en,
    rx_amp_en,
    rx_en,
    rx_bb_en,
    tx_sw,
    tx_amp_sw,
    tx_amp_en,
    tx_en,
    mon_sw,
    vout_en,
    dac_test_en,
    dio_dir0 = field(16, 2),
    spare0 = 18,
    spare1 = 19,
    spare2 = 20,
    spare3 = 21,
    spare4 = 22,
    spare5 = 23,
    ext_clk_en = 24,
    spare6 = 25,
    synth_le_rx = 26,
    synth_le_tx = 27
};
DECLARE_BITFIELD(HW_CTRL, uint32_t);

enum class HW_STATUS {
    adc_busy,
    dac_busy,
    synth_busy,
    no_adc_clk,
    pll_locked,
    clkgen_status,
    synth_ld,
    vout_fault,
    sd_detect,
    dig_prog_full,
    dig_full,
    dig_empty,
    atten_busy
};
DECLARE_BITFIELD(HW_STATUS, uint32_t);

enum class GROMIT_CTRL {
    ENABLE_1M = 0,
    ENABLE_2M = 1,

    WL_UPDATE_CON_RQT = 4,
    WL_UPDATE_ICL = 5,
    WL_UPDATE_ARM = 6,
    WL_UPDATE_PERIODIC_ADV = 7,
    WL_UPDATE_SEARCH_12 = 8,
    WL_UPDATE_SEARCH_LR = 9,
    WL_UPDATE_ICO = 10,

    CAPTURE_PKT = 12,
    CAPTURE_SPECTRUM = 13,
    CAPTURE_WHITELIST = 14,
    CAPTURE_GAIN = 15,
    CAPTURE_DIG = 16,
    CAPTURE_UART0 = 17,
    CAPTURE_UART1 = 18,
    CAPTURE_UART2 = 19,
    CAPTURE_UART3 = 20,

    WHITELIST_OPCODE = field(28, 3),
    WHITELIST_CTRL = 31
};
DECLARE_BITFIELD(GROMIT_CTRL, uint32_t);


template<class T = uint32_t>
class Register {
    Register(const Register&) = delete;
public:
    using type = T;
    uint32_t addr() {
        return (uintptr_t)this/sizeof(Register);
    }
    T operator=(T val) {
        gromit->reg_write(addr(), uint32_t(val));
        return val;
    }
    T operator|=(T val) {
        val |= T(gromit->reg_read(addr()));
        gromit->reg_write(addr(), uint32_t(val));
        return val;
    }
    T operator&=(T val) {
        val &= T(gromit->reg_read(addr()));
        gromit->reg_write(addr(), uint32_t(val));
        return val;
    }
    operator T() {
        return T(gromit->reg_read(addr()));
    }
    template<typename U = T, typename V = std::void_t<typename U::Int>>
    operator typename U::Int() {
        return typename U::Int(T(gromit->reg_read(addr())));
    }
};

static struct hw_regs_t {
    Register<bitfield<HW_CTRL>> ctrl;
    Register<> adc;
    Register<> dac;
    Register<> synth;
    Register<> atten_rx;
    Register<> atten_tx;
    Register<> phi_shift;
    Register<> dio_ctrl;
    Register<> clk_ctrl;
    Register<> dac_delay;
    Register<bitfield<HW_STATUS>> status;
    Register<> phi_done;
    Register<> fpga_time_lsb;
    Register<> fpga_time_msb;
    Register<> temperature;
}& hw = *(hw_regs_t*)0;

static struct gromit_regs_t {
    Register<> dbg;
    Register<bitfield<GROMIT_CTRL>> ctrl;
    Register<> wl_aa;
    Register<> throttle;
    Register<> DemodThresh;
    Register<> trigger_mask0;
    Register<> trigger_mask1;
    Register<> db_cal;
    Register<> spec_period;
    Register<> wl_crc_seed;
    Register<> RSSIthresh;
    Register<> capture_addr;
    Register<> capture;
    Register<> whitelist;
    Register<> gain_period;
    Register<> up_dn;
    Register<> uart0;
    Register<> uart1;
    Register<> uart2;
    Register<> uart3;
    Register<> app_status;
    Register<> whitelist_data;
    Register<> young;
    Register<> build_feat;
    Register<> build_version;
    Register<> build_date;
    Register<> build_time;
    Register<> agc_hist;
}& gromit_regs = *(gromit_regs_t*)0x100;

#endif // REGISTERS_H

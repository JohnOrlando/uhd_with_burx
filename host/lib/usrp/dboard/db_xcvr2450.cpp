//
// Copyright 2010-2011 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

// TX IO Pins
#define HB_PA_OFF_TXIO      (1 << 15)    // 5GHz PA, 1 = off, 0 = on
#define LB_PA_OFF_TXIO      (1 << 14)    // 2.4GHz PA, 1 = off, 0 = on
#define ANTSEL_TX1_RX2_TXIO (1 << 13)    // 1 = Ant 1 to TX, Ant 2 to RX
#define ANTSEL_TX2_RX1_TXIO (1 << 12)    // 1 = Ant 2 to TX, Ant 1 to RX
#define TX_EN_TXIO          (1 << 11)    // 1 = TX on, 0 = TX off
#define AD9515DIV_TXIO      (1 << 4)     // 1 = Div  by 3, 0 = Div by 2

#define TXIO_MASK (HB_PA_OFF_TXIO | LB_PA_OFF_TXIO | ANTSEL_TX1_RX2_TXIO | ANTSEL_TX2_RX1_TXIO | TX_EN_TXIO | AD9515DIV_TXIO)

// TX IO Functions
#define HB_PA_TXIO               LB_PA_OFF_TXIO
#define LB_PA_TXIO               HB_PA_OFF_TXIO
#define TX_ENB_TXIO              TX_EN_TXIO
#define TX_DIS_TXIO              0
#define AD9515DIV_3_TXIO         AD9515DIV_TXIO
#define AD9515DIV_2_TXIO         0

// RX IO Pins
#define LOCKDET_RXIO (1 << 15)           // This is an INPUT!!!
#define POWER_RXIO   (1 << 14)           // 1 = power on, 0 = shutdown
#define RX_EN_RXIO   (1 << 13)           // 1 = RX on, 0 = RX off
#define RX_HP_RXIO   (1 << 12)           // 0 = Fc set by rx_hpf, 1 = 600 KHz

#define RXIO_MASK (POWER_RXIO | RX_EN_RXIO | RX_HP_RXIO)

// RX IO Functions
#define POWER_UP_RXIO            POWER_RXIO
#define POWER_DOWN_RXIO          0
#define RX_ENB_RXIO              RX_EN_RXIO
#define RX_DIS_RXIO              0

#include "max2829_regs.hpp"
#include <uhd/utils/log.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/utils/assert_has.hpp>
#include <uhd/utils/algorithm.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/usrp/dboard_base.hpp>
#include <uhd/usrp/dboard_manager.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/math/special_functions/round.hpp>
#include <utility>

using namespace uhd;
using namespace uhd::usrp;
using namespace boost::assign;

/***********************************************************************
 * The XCVR 2450 constants
 **********************************************************************/
static const freq_range_t xcvr_freq_range = list_of
    (range_t(2.4e9, 2.5e9))
    (range_t(4.9e9, 6.0e9))
;

static const prop_names_t xcvr_antennas = list_of("J1")("J2");

static const uhd::dict<std::string, gain_range_t> xcvr_tx_gain_ranges = map_list_of
    ("VGA", gain_range_t(0, 30, 0.5))
    ("BB", gain_range_t(0, 5, 1.5))
;
static const uhd::dict<std::string, gain_range_t> xcvr_rx_gain_ranges = map_list_of
    ("LNA", gain_range_t(list_of
        (range_t(0))
        (range_t(15))
        (range_t(30.5))
    ))
    ("VGA", gain_range_t(0, 62, 2.0))
;

/***********************************************************************
 * The XCVR 2450 dboard class
 **********************************************************************/
class xcvr2450 : public xcvr_dboard_base{
public:
    xcvr2450(ctor_args_t args);
    ~xcvr2450(void);

    void rx_get(const wax::obj &key, wax::obj &val);
    void rx_set(const wax::obj &key, const wax::obj &val);

    void tx_get(const wax::obj &key, wax::obj &val);
    void tx_set(const wax::obj &key, const wax::obj &val);

private:
    double _lo_freq;
    double _rx_bandwidth, _tx_bandwidth;
    uhd::dict<std::string, double> _tx_gains, _rx_gains;
    std::string _tx_ant, _rx_ant;
    int _ad9515div;
    max2829_regs_t _max2829_regs;

    void set_lo_freq(double target_freq);
    void set_lo_freq_core(double target_freq);
    void set_tx_ant(const std::string &ant);
    void set_rx_ant(const std::string &ant);
    void set_tx_gain(double gain, const std::string &name);
    void set_rx_gain(double gain, const std::string &name);
    void set_rx_bandwidth(double bandwidth);
    void set_tx_bandwidth(double bandwidth);

    void update_atr(void);
    void spi_reset(void);
    void send_reg(boost::uint8_t addr){
        boost::uint32_t value = _max2829_regs.get_reg(addr);
        UHD_LOGV(often) << boost::format(
            "XCVR2450: send reg 0x%02x, value 0x%05x"
        ) % int(addr) % value << std::endl;
        this->get_iface()->write_spi(
            dboard_iface::UNIT_RX,
            spi_config_t::EDGE_RISE,
            value, 24
        );
    }

    static bool is_highband(double freq){return freq > 3e9;}

    /*!
     * Is the LO locked?
     * \return true for locked
     */
    bool get_locked(void){
        return (this->get_iface()->read_gpio(dboard_iface::UNIT_RX) & LOCKDET_RXIO) != 0;
    }

    /*!
     * Read the RSSI from the aux adc
     * \return the rssi in dB
     */
    double get_rssi(void){
        //*FIXME* RSSI depends on LNA Gain Setting (datasheet pg 16 top middle chart)
        double max_power = 0.0;
        switch(_max2829_regs.rx_lna_gain){
        case 0:
        case 1: max_power = 0;    break;
        case 2: max_power = -15;   break;
        case 3: max_power = -30.5; break;
        }

        //constants for the rssi calculation
        static const double min_v = 0.5, max_v = 2.5;
        static const double rssi_dyn_range = 60;
        //calculate the rssi from the voltage
        double voltage = this->get_iface()->read_aux_adc(dboard_iface::UNIT_RX, dboard_iface::AUX_ADC_B);
        return max_power - rssi_dyn_range*(voltage - min_v)/(max_v - min_v);
    }
};

/***********************************************************************
 * Register the XCVR 2450 dboard
 **********************************************************************/
static dboard_base::sptr make_xcvr2450(dboard_base::ctor_args_t args){
    return dboard_base::sptr(new xcvr2450(args));
}

UHD_STATIC_BLOCK(reg_xcvr2450_dboard){
    //register the factory function for the rx and tx dbids
    dboard_manager::register_dboard(0x0061, 0x0060, &make_xcvr2450, "XCVR2450");
}

/***********************************************************************
 * Structors
 **********************************************************************/
xcvr2450::xcvr2450(ctor_args_t args) : xcvr_dboard_base(args){
    //enable only the clocks we need
    this->get_iface()->set_clock_enabled(dboard_iface::UNIT_TX, true);

    //set the gpio directions and atr controls (identically)
    this->get_iface()->set_pin_ctrl(dboard_iface::UNIT_TX, TXIO_MASK);
    this->get_iface()->set_pin_ctrl(dboard_iface::UNIT_RX, RXIO_MASK);
    this->get_iface()->set_gpio_ddr(dboard_iface::UNIT_TX, TXIO_MASK);
    this->get_iface()->set_gpio_ddr(dboard_iface::UNIT_RX, RXIO_MASK);

    spi_reset(); //prepare the spi

    _rx_bandwidth = 9.5e6;
    _tx_bandwidth = 12.0e6;

    //setup the misc max2829 registers
    _max2829_regs.mimo_select         = max2829_regs_t::MIMO_SELECT_MIMO;
    _max2829_regs.band_sel_mimo       = max2829_regs_t::BAND_SEL_MIMO_MIMO;
    _max2829_regs.pll_cp_select       = max2829_regs_t::PLL_CP_SELECT_4MA;
    _max2829_regs.rssi_high_bw        = max2829_regs_t::RSSI_HIGH_BW_6MHZ;
    _max2829_regs.tx_lpf_coarse_adj   = max2829_regs_t::TX_LPF_COARSE_ADJ_12MHZ;
    _max2829_regs.rx_lpf_coarse_adj   = max2829_regs_t::RX_LPF_COARSE_ADJ_9_5MHZ;
    _max2829_regs.rx_lpf_fine_adj     = max2829_regs_t::RX_LPF_FINE_ADJ_100;
    _max2829_regs.rx_vga_gain_spi     = max2829_regs_t::RX_VGA_GAIN_SPI_SPI;
    _max2829_regs.rssi_output_range   = max2829_regs_t::RSSI_OUTPUT_RANGE_HIGH;
    _max2829_regs.rssi_op_mode        = max2829_regs_t::RSSI_OP_MODE_ENABLED;
    _max2829_regs.rssi_pin_fcn        = max2829_regs_t::RSSI_PIN_FCN_RSSI;
    _max2829_regs.rx_highpass         = max2829_regs_t::RX_HIGHPASS_100HZ;
    _max2829_regs.tx_vga_gain_spi     = max2829_regs_t::TX_VGA_GAIN_SPI_SPI;
    _max2829_regs.pa_driver_linearity = max2829_regs_t::PA_DRIVER_LINEARITY_78;
    _max2829_regs.tx_vga_linearity    = max2829_regs_t::TX_VGA_LINEARITY_78;
    _max2829_regs.tx_upconv_linearity = max2829_regs_t::TX_UPCONV_LINEARITY_78;

    //send initial register settings
    for(boost::uint8_t reg = 0x2; reg <= 0xC; reg++){
        this->send_reg(reg);
    }

    //set defaults for LO, gains, antennas
    set_lo_freq(2.45e9);
    set_rx_ant(xcvr_antennas.at(0));
    set_tx_ant(xcvr_antennas.at(1));
    BOOST_FOREACH(const std::string &name, xcvr_tx_gain_ranges.keys()){
        set_tx_gain(xcvr_tx_gain_ranges[name].start(), name);
    }
    BOOST_FOREACH(const std::string &name, xcvr_rx_gain_ranges.keys()){
        set_rx_gain(xcvr_rx_gain_ranges[name].start(), name);
    }
}

xcvr2450::~xcvr2450(void){
    UHD_SAFE_CALL(spi_reset();)
}

void xcvr2450::spi_reset(void){
    //spi reset mode: global enable = off, tx and rx enable = on
    this->get_iface()->set_atr_reg(dboard_iface::UNIT_TX, dboard_iface::ATR_REG_IDLE, TX_ENB_TXIO);
    this->get_iface()->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_IDLE, RX_ENB_RXIO | POWER_DOWN_RXIO);
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));

    //take it back out of spi reset mode and wait a bit
    this->get_iface()->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_IDLE, RX_DIS_RXIO | POWER_UP_RXIO);
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
}

void xcvr2450::update_atr(void){
    //calculate tx atr pins
    int band_sel   = (xcvr2450::is_highband(_lo_freq))? HB_PA_TXIO : LB_PA_TXIO;
    int tx_ant_sel = (_tx_ant == "J1")? ANTSEL_TX1_RX2_TXIO : ANTSEL_TX2_RX1_TXIO;
    int rx_ant_sel = (_rx_ant == "J2")? ANTSEL_TX1_RX2_TXIO : ANTSEL_TX2_RX1_TXIO;
    int xx_ant_sel = tx_ant_sel; //Prefer the tx antenna selection for full duplex,
    //due to the issue that USRP1 will take the value of full duplex for its TXATR.
    int ad9515div  = (_ad9515div == 3)? AD9515DIV_3_TXIO : AD9515DIV_2_TXIO;

    //set the tx registers
    this->get_iface()->set_atr_reg(dboard_iface::UNIT_TX, dboard_iface::ATR_REG_IDLE,        band_sel | ad9515div | TX_DIS_TXIO);
    this->get_iface()->set_atr_reg(dboard_iface::UNIT_TX, dboard_iface::ATR_REG_RX_ONLY,     band_sel | ad9515div | TX_DIS_TXIO | rx_ant_sel);
    this->get_iface()->set_atr_reg(dboard_iface::UNIT_TX, dboard_iface::ATR_REG_TX_ONLY,     band_sel | ad9515div | TX_ENB_TXIO | tx_ant_sel);
    this->get_iface()->set_atr_reg(dboard_iface::UNIT_TX, dboard_iface::ATR_REG_FULL_DUPLEX, band_sel | ad9515div | TX_ENB_TXIO | xx_ant_sel);

    //set the rx registers
    this->get_iface()->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_IDLE,        POWER_UP_RXIO | RX_DIS_RXIO);
    this->get_iface()->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_RX_ONLY,     POWER_UP_RXIO | RX_ENB_RXIO);
    this->get_iface()->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_TX_ONLY,     POWER_UP_RXIO | RX_DIS_RXIO);
    this->get_iface()->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_FULL_DUPLEX, POWER_UP_RXIO | RX_DIS_RXIO);
}

/***********************************************************************
 * Tuning
 **********************************************************************/
void xcvr2450::set_lo_freq(double target_freq){
    //tune the LO and sleep a bit for lock
    //if not locked, try some carrier offsets
    for (double offset = 0.0; offset <= 3e6; offset+=1e6){
        this->set_lo_freq_core(target_freq + offset);
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        if (this->get_locked()) return;
    }
}

void xcvr2450::set_lo_freq_core(double target_freq){

    //clip the input to the range
    target_freq = xcvr_freq_range.clip(target_freq);

    //variables used in the calculation below
    double scaler = xcvr2450::is_highband(target_freq)? (4.0/5.0) : (4.0/3.0);
    double ref_freq = this->get_iface()->get_clock_rate(dboard_iface::UNIT_TX);
    int R, intdiv, fracdiv;

    //loop through values until we get a match
    for(_ad9515div = 2; _ad9515div <= 3; _ad9515div++){
        for(R = 1; R <= 7; R++){
            double N = (target_freq*scaler*R*_ad9515div)/ref_freq;
            intdiv = int(std::floor(N));
            fracdiv = boost::math::iround((N - intdiv)*double(1 << 16));
            //actual minimum is 128, but most chips seems to require higher to lock
            if (intdiv < 131 or intdiv > 255) continue;
            //constraints met: exit loop
            goto done_loop;
        }
    } done_loop:

    //calculate the actual freq from the values above
    double N = double(intdiv) + double(fracdiv)/double(1 << 16);
    _lo_freq = (N*ref_freq)/(scaler*R*_ad9515div);

    UHD_LOGV(often)
        << boost::format("XCVR2450 tune:\n")
        << boost::format("    R=%d, N=%f, ad9515=%d, scaler=%f\n") % R % N % _ad9515div % scaler
        << boost::format("    Ref    Freq=%fMHz\n") % (ref_freq/1e6)
        << boost::format("    Target Freq=%fMHz\n") % (target_freq/1e6)
        << boost::format("    Actual Freq=%fMHz\n") % (_lo_freq/1e6)
        << std::endl;

    //high-high band or low-high band?
    if(_lo_freq > (5.35e9 + 5.47e9)/2.0){
        UHD_LOGV(often) << "XCVR2450 tune: Using  high-high band" << std::endl;
        _max2829_regs.band_select_802_11a = max2829_regs_t::BAND_SELECT_802_11A_5_47GHZ_TO_5_875GHZ;
    }else{
        UHD_LOGV(often) << "XCVR2450 tune: Using  low-high band" << std::endl;
        _max2829_regs.band_select_802_11a = max2829_regs_t::BAND_SELECT_802_11A_4_9GHZ_TO_5_35GHZ;
    }

    //new band select settings and ad9515 divider
    this->update_atr();

    //load new counters into registers
    _max2829_regs.int_div_ratio_word = intdiv;
    _max2829_regs.frac_div_ratio_lsb = fracdiv & 0x3;
    _max2829_regs.frac_div_ratio_msb = fracdiv >> 2;
    this->send_reg(0x3); //integer
    this->send_reg(0x4); //fractional

    //load the reference divider and band select into registers
    //toggle the bandswitch from off to automatic (which really means start)
    _max2829_regs.ref_divider = R;
    _max2829_regs.band_select = (xcvr2450::is_highband(_lo_freq))?
                                max2829_regs_t::BAND_SELECT_5GHZ   :
                                max2829_regs_t::BAND_SELECT_2_4GHZ ;
    _max2829_regs.vco_bandswitch = max2829_regs_t::VCO_BANDSWITCH_DISABLE;
    this->send_reg(0x5);
    _max2829_regs.vco_bandswitch = max2829_regs_t::VCO_BANDSWITCH_AUTOMATIC;;
    this->send_reg(0x5);
}

/***********************************************************************
 * Antenna Handling
 **********************************************************************/
void xcvr2450::set_tx_ant(const std::string &ant){
    assert_has(xcvr_antennas, ant, "xcvr antenna name");
   _tx_ant = ant;
    this->update_atr(); //sets the atr to the new antenna setting
}

void xcvr2450::set_rx_ant(const std::string &ant){
    assert_has(xcvr_antennas, ant, "xcvr antenna name");
    _rx_ant = ant;
    this->update_atr(); //sets the atr to the new antenna setting
}

/***********************************************************************
 * Gain Handling
 **********************************************************************/
/*!
 * Convert a requested gain for the tx vga into the integer register value.
 * The gain passed into the function will be set to the actual value.
 * \param gain the requested gain in dB
 * \return 6 bit the register value
 */
static int gain_to_tx_vga_reg(double &gain){
    //calculate the register value
    int reg = uhd::clip(boost::math::iround(gain*60/30.0) + 3, 0, 63);

    //calculate the actual gain value
    if (reg < 4)       gain = 0;
    else if (reg < 48) gain = double(reg/2 - 1);
    else               gain = double(reg/2.0 - 1.5);

    //return register value
    return reg;
}

/*!
 * Convert a requested gain for the tx bb into the integer register value.
 * The gain passed into the function will be set to the actual value.
 * \param gain the requested gain in dB
 * \return gain enum value
 */
static max2829_regs_t::tx_baseband_gain_t gain_to_tx_bb_reg(double &gain){
    int reg = uhd::clip(boost::math::iround(gain*3/5.0), 0, 3);
    switch(reg){
    case 0:
        gain = 0;
        return max2829_regs_t::TX_BASEBAND_GAIN_0DB;
    case 1:
        gain = 2;
        return max2829_regs_t::TX_BASEBAND_GAIN_2DB;
    case 2:
        gain = 3.5;
        return max2829_regs_t::TX_BASEBAND_GAIN_3_5DB;
    case 3:
        gain = 5;
        return max2829_regs_t::TX_BASEBAND_GAIN_5DB;
    }
    UHD_THROW_INVALID_CODE_PATH();
}

/*!
 * Convert a requested gain for the rx vga into the integer register value.
 * The gain passed into the function will be set to the actual value.
 * \param gain the requested gain in dB
 * \return 5 bit the register value
 */
static int gain_to_rx_vga_reg(double &gain){
    int reg = uhd::clip(boost::math::iround(gain/2.0), 0, 31);
    gain = double(reg*2);
    return reg;
}

/*!
 * Convert a requested gain for the rx lna into the integer register value.
 * The gain passed into the function will be set to the actual value.
 * \param gain the requested gain in dB
 * \return 2 bit the register value
 */
static int gain_to_rx_lna_reg(double &gain){
    int reg = uhd::clip(boost::math::iround(gain*2/30.5) + 1, 0, 3);
    switch(reg){
    case 0:
    case 1: gain = 0;    break;
    case 2: gain = 15;   break;
    case 3: gain = 30.5; break;
    }
    return reg;
}

void xcvr2450::set_tx_gain(double gain, const std::string &name){
    assert_has(xcvr_tx_gain_ranges.keys(), name, "xcvr tx gain name");
    if (name == "VGA"){
        _max2829_regs.tx_vga_gain = gain_to_tx_vga_reg(gain);
        send_reg(0xC);
    }
    else if(name == "BB"){
        _max2829_regs.tx_baseband_gain = gain_to_tx_bb_reg(gain);
        send_reg(0x9);
    }
    else UHD_THROW_INVALID_CODE_PATH();
    _tx_gains[name] = gain;
}

void xcvr2450::set_rx_gain(double gain, const std::string &name){
    assert_has(xcvr_rx_gain_ranges.keys(), name, "xcvr rx gain name");
    if (name == "VGA"){
        _max2829_regs.rx_vga_gain = gain_to_rx_vga_reg(gain);
        send_reg(0xB);
    }
    else if(name == "LNA"){
        _max2829_regs.rx_lna_gain = gain_to_rx_lna_reg(gain);
        send_reg(0xB);
    }
    else UHD_THROW_INVALID_CODE_PATH();
    _rx_gains[name] = gain;
}


/***********************************************************************
 * Bandwidth Handling
 **********************************************************************/
static max2829_regs_t::tx_lpf_coarse_adj_t bandwidth_to_tx_lpf_coarse_reg(double &bandwidth){
    int reg = uhd::clip(boost::math::iround((bandwidth-6.0e6)/6.0e6), 1, 3);

    switch(reg){
    case 1: // bandwidth < 15MHz
        bandwidth = 12e6;
        return max2829_regs_t::TX_LPF_COARSE_ADJ_12MHZ;
    case 2: // 15MHz < bandwidth < 21MHz
        bandwidth = 18e6;
        return max2829_regs_t::TX_LPF_COARSE_ADJ_18MHZ;
    case 3: // bandwidth > 21MHz
        bandwidth = 24e6;
        return max2829_regs_t::TX_LPF_COARSE_ADJ_24MHZ;
    }
    UHD_THROW_INVALID_CODE_PATH();
}

static max2829_regs_t::rx_lpf_fine_adj_t bandwidth_to_rx_lpf_fine_reg(double &bandwidth, double requested_bandwidth){
    int reg = uhd::clip(boost::math::iround((requested_bandwidth/bandwidth)/0.05), 18, 22);

    switch(reg){
    case 18: // requested_bandwidth < 92.5%
        bandwidth = 0.9 * bandwidth;
        return max2829_regs_t::RX_LPF_FINE_ADJ_90;
    case 19: // 92.5% < requested_bandwidth < 97.5%
        bandwidth = 0.95 * bandwidth;
        return max2829_regs_t::RX_LPF_FINE_ADJ_95;
    case 20: // 97.5% < requested_bandwidth < 102.5%
        bandwidth = 1.0 * bandwidth;
        return max2829_regs_t::RX_LPF_FINE_ADJ_100;
    case 21: // 102.5% < requested_bandwidth < 107.5%
        bandwidth = 1.05 * bandwidth;
        return max2829_regs_t::RX_LPF_FINE_ADJ_105;
    case 22: // 107.5% < requested_bandwidth
        bandwidth = 1.1 * bandwidth;
        return max2829_regs_t::RX_LPF_FINE_ADJ_110;
    }
    UHD_THROW_INVALID_CODE_PATH();
}

static max2829_regs_t::rx_lpf_coarse_adj_t bandwidth_to_rx_lpf_coarse_reg(double &bandwidth){
    int reg = uhd::clip(boost::math::iround((bandwidth-7.0e6)/1.0e6), 0, 11);

    switch(reg){
    case 0: // bandwidth < 7.5MHz
    case 1: // 7.5MHz < bandwidth < 8.5MHz
        bandwidth = 7.5e6;
        return max2829_regs_t::RX_LPF_COARSE_ADJ_7_5MHZ;
    case 2: // 8.5MHz < bandwidth < 9.5MHz
    case 3: // 9.5MHz < bandwidth < 10.5MHz
    case 4: // 10.5MHz < bandwidth < 11.5MHz
        bandwidth = 9.5e6;
        return max2829_regs_t::RX_LPF_COARSE_ADJ_9_5MHZ;
    case 5: // 11.5MHz < bandwidth < 12.5MHz
    case 6: // 12.5MHz < bandwidth < 13.5MHz
    case 7: // 13.5MHz < bandwidth < 14.5MHz
    case 8: // 14.5MHz < bandwidth < 15.5MHz
        bandwidth = 14e6;
        return max2829_regs_t::RX_LPF_COARSE_ADJ_14MHZ;
    case 9: // 15.5MHz < bandwidth < 16.5MHz
    case 10: // 16.5MHz < bandwidth < 17.5MHz
    case 11: // 17.5MHz < bandwidth
        bandwidth = 18e6;
        return max2829_regs_t::RX_LPF_COARSE_ADJ_18MHZ;
    }
    UHD_THROW_INVALID_CODE_PATH();
}

void xcvr2450::set_rx_bandwidth(double bandwidth){
    double requested_bandwidth = bandwidth;

    //compute coarse low pass cutoff frequency setting
    _max2829_regs.rx_lpf_coarse_adj = bandwidth_to_rx_lpf_coarse_reg(bandwidth);

    //compute fine low pass cutoff frequency setting
    _max2829_regs.rx_lpf_fine_adj = bandwidth_to_rx_lpf_fine_reg(bandwidth, requested_bandwidth);

    //shadow bandwidth setting
    _rx_bandwidth = bandwidth;

    //update register
    send_reg(0x7);

    UHD_LOGV(often) << boost::format(
        "XCVR2450 RX Bandwidth (lp_fc): %f Hz, coarse reg: %d, fine reg: %d"
    ) % _rx_bandwidth % (int(_max2829_regs.rx_lpf_coarse_adj)) % (int(_max2829_regs.rx_lpf_fine_adj)) << std::endl;
}

void xcvr2450::set_tx_bandwidth(double bandwidth){
    //compute coarse low pass cutoff frequency setting
    _max2829_regs.tx_lpf_coarse_adj = bandwidth_to_tx_lpf_coarse_reg(bandwidth);

    //shadow bandwidth setting
    _tx_bandwidth = bandwidth;

    //update register
    send_reg(0x7);

    UHD_LOGV(often) << boost::format(
        "XCVR2450 TX Bandwidth (lp_fc): %f Hz, coarse reg: %d"
    ) % _tx_bandwidth % (int(_max2829_regs.tx_lpf_coarse_adj)) << std::endl;
}


/***********************************************************************
 * RX Get and Set
 **********************************************************************/
void xcvr2450::rx_get(const wax::obj &key_, wax::obj &val){
    named_prop_t key = named_prop_t::extract(key_);

    //handle the get request conditioned on the key
    switch(key.as<subdev_prop_t>()){
    case SUBDEV_PROP_NAME:
        val = get_rx_id().to_pp_string();
        return;

    case SUBDEV_PROP_OTHERS:
        val = prop_names_t(); //empty
        return;

    case SUBDEV_PROP_GAIN:
        assert_has(_rx_gains.keys(), key.name, "xcvr rx gain name");
        val = _rx_gains[key.name];
        return;

    case SUBDEV_PROP_GAIN_RANGE:
        assert_has(xcvr_rx_gain_ranges.keys(), key.name, "xcvr rx gain name");
        val = xcvr_rx_gain_ranges[key.name];
        return;

    case SUBDEV_PROP_GAIN_NAMES:
        val = prop_names_t(xcvr_rx_gain_ranges.keys());
        return;

    case SUBDEV_PROP_FREQ:
        val = _lo_freq;
        return;

    case SUBDEV_PROP_FREQ_RANGE:
        val = xcvr_freq_range;
        return;

    case SUBDEV_PROP_ANTENNA:
        val = _rx_ant;
        return;

    case SUBDEV_PROP_ANTENNA_NAMES:
        val = xcvr_antennas;
        return;

    case SUBDEV_PROP_CONNECTION:
        val = SUBDEV_CONN_COMPLEX_IQ;
        return;

    case SUBDEV_PROP_ENABLED:
        val = true; //always enabled
        return;

    case SUBDEV_PROP_USE_LO_OFFSET:
        val = false;
        return;

    case SUBDEV_PROP_SENSOR:
        if (key.name == "lo_locked")
            val = sensor_value_t("LO", this->get_locked(), "locked", "unlocked");
        else if (key.name == "rssi")
            val = sensor_value_t("RSSI", this->get_rssi(), "dBm");
        else
            UHD_THROW_INVALID_CODE_PATH();
        return;

    case SUBDEV_PROP_SENSOR_NAMES:{
            prop_names_t names = list_of("lo_locked")("rssi");
            val = names;
        }
        return;

    case SUBDEV_PROP_BANDWIDTH:
        val = 2*_rx_bandwidth; //_tx_bandwidth is low-pass, we want complex double-sided
        return;

    default: UHD_THROW_PROP_GET_ERROR();
    }
}

void xcvr2450::rx_set(const wax::obj &key_, const wax::obj &val){
    named_prop_t key = named_prop_t::extract(key_);

    //handle the get request conditioned on the key
    switch(key.as<subdev_prop_t>()){

    case SUBDEV_PROP_FREQ:
        this->set_lo_freq(val.as<double>());
        return;

    case SUBDEV_PROP_GAIN:
        this->set_rx_gain(val.as<double>(), key.name);
        return;

    case SUBDEV_PROP_ANTENNA:
        this->set_rx_ant(val.as<std::string>());
        return;

    case SUBDEV_PROP_BANDWIDTH:
        this->set_rx_bandwidth(val.as<double>()/2.0); //complex double-sided, we want low-pass
        return;

    case SUBDEV_PROP_ENABLED:
        return; //always enabled

    default: UHD_THROW_PROP_SET_ERROR();
    }
}

/***********************************************************************
 * TX Get and Set
 **********************************************************************/
void xcvr2450::tx_get(const wax::obj &key_, wax::obj &val){
    named_prop_t key = named_prop_t::extract(key_);

    //handle the get request conditioned on the key
    switch(key.as<subdev_prop_t>()){
    case SUBDEV_PROP_NAME:
        val = get_tx_id().to_pp_string();
        return;

    case SUBDEV_PROP_OTHERS:
        val = prop_names_t(); //empty
        return;

    case SUBDEV_PROP_GAIN:
        assert_has(_tx_gains.keys(), key.name, "xcvr tx gain name");
        val = _tx_gains[key.name];
        return;

    case SUBDEV_PROP_GAIN_RANGE:
        assert_has(xcvr_tx_gain_ranges.keys(), key.name, "xcvr tx gain name");
        val = xcvr_tx_gain_ranges[key.name];
        return;

    case SUBDEV_PROP_GAIN_NAMES:
        val = prop_names_t(xcvr_tx_gain_ranges.keys());
        return;

    case SUBDEV_PROP_FREQ:
        val = _lo_freq;
        return;

    case SUBDEV_PROP_FREQ_RANGE:
        val = xcvr_freq_range;
        return;

    case SUBDEV_PROP_ANTENNA:
        val = _tx_ant;
        return;

    case SUBDEV_PROP_ANTENNA_NAMES:
        val = xcvr_antennas;
        return;

    case SUBDEV_PROP_CONNECTION:
        val = SUBDEV_CONN_COMPLEX_QI;
        return;

    case SUBDEV_PROP_ENABLED:
        val = true; //always enabled
        return;

    case SUBDEV_PROP_USE_LO_OFFSET:
        val = false;
        return;

    case SUBDEV_PROP_SENSOR:
        UHD_ASSERT_THROW(key.name == "lo_locked");
        val = sensor_value_t("LO", this->get_locked(), "locked", "unlocked");
        return;

    case SUBDEV_PROP_SENSOR_NAMES:
        val = prop_names_t(1, "lo_locked");
        return;

    case SUBDEV_PROP_BANDWIDTH:
        val = 2*_tx_bandwidth; //_tx_bandwidth is low-pass, we want complex double-sided
        return;

    default: UHD_THROW_PROP_GET_ERROR();
    }
}

void xcvr2450::tx_set(const wax::obj &key_, const wax::obj &val){
    named_prop_t key = named_prop_t::extract(key_);

    //handle the get request conditioned on the key
    switch(key.as<subdev_prop_t>()){

    case SUBDEV_PROP_FREQ:
        set_lo_freq(val.as<double>());
        return;

    case SUBDEV_PROP_GAIN:
        this->set_tx_gain(val.as<double>(), key.name);
        return;

    case SUBDEV_PROP_BANDWIDTH:
        this->set_tx_bandwidth(val.as<double>()/2.0); //complex double-sided, we want low-pass
        return;

    case SUBDEV_PROP_ANTENNA:
        this->set_tx_ant(val.as<std::string>());
        return;

    case SUBDEV_PROP_ENABLED:
        return; //always enabled

    default: UHD_THROW_PROP_SET_ERROR();
    }
}

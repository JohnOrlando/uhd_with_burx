//
// Copyright 2011 Ettus Research LLC
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

#include "wb_iface.hpp"
#include <uhd/types/serial.hpp>
#include "b100_regs.hpp"
#include "clock_ctrl.hpp"
#include "codec_ctrl.hpp"
#include <uhd/usrp/dboard_iface.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/exception.hpp>
#include <boost/assign/list_of.hpp>


using namespace uhd;
using namespace uhd::usrp;
using namespace boost::assign;

class b100_dboard_iface : public dboard_iface{
public:

    b100_dboard_iface(
        wb_iface::sptr wb_iface,
        i2c_iface::sptr i2c_iface,
        spi_iface::sptr spi_iface,
        b100_clock_ctrl::sptr clock,
        b100_codec_ctrl::sptr codec
    ){
        _wb_iface = wb_iface;
        _i2c_iface = i2c_iface;
        _spi_iface = spi_iface;
        _clock = clock;
        _codec = codec;

        //init the clock rate shadows
        this->set_clock_rate(UNIT_RX, _clock->get_fpga_clock_rate());
        this->set_clock_rate(UNIT_TX, _clock->get_fpga_clock_rate());

        _wb_iface->poke16(B100_REG_GPIO_RX_DBG, 0);
        _wb_iface->poke16(B100_REG_GPIO_TX_DBG, 0);
    }

    ~b100_dboard_iface(void){
        /* NOP */
    }

    special_props_t get_special_props(void){
        special_props_t props;
        props.soft_clock_divider = false;
        props.mangle_i2c_addrs = false;
        return props;
    }

    void write_aux_dac(unit_t, aux_dac_t, double);
    double read_aux_adc(unit_t, aux_adc_t);

    void _set_pin_ctrl(unit_t, boost::uint16_t);
    void _set_atr_reg(unit_t, atr_reg_t, boost::uint16_t);
    void _set_gpio_ddr(unit_t, boost::uint16_t);
    void _set_gpio_out(unit_t, boost::uint16_t);
    void set_gpio_debug(unit_t, int);
    boost::uint16_t read_gpio(unit_t);

    void write_i2c(boost::uint8_t, const byte_vector_t &);
    byte_vector_t read_i2c(boost::uint8_t, size_t);

    void write_spi(
        unit_t unit,
        const spi_config_t &config,
        boost::uint32_t data,
        size_t num_bits
    );

    boost::uint32_t read_write_spi(
        unit_t unit,
        const spi_config_t &config,
        boost::uint32_t data,
        size_t num_bits
    );

    void set_clock_rate(unit_t, double);
    std::vector<double> get_clock_rates(unit_t);
    double get_clock_rate(unit_t);
    void set_clock_enabled(unit_t, bool);
    double get_codec_rate(unit_t);

private:
    wb_iface::sptr _wb_iface;
    i2c_iface::sptr _i2c_iface;
    spi_iface::sptr _spi_iface;
    b100_clock_ctrl::sptr _clock;
    b100_codec_ctrl::sptr _codec;
    uhd::dict<unit_t, double> _clock_rates;
};

/***********************************************************************
 * Make Function
 **********************************************************************/
dboard_iface::sptr make_b100_dboard_iface(
    wb_iface::sptr wb_iface,
    i2c_iface::sptr i2c_iface,
    spi_iface::sptr spi_iface,
    b100_clock_ctrl::sptr clock,
    b100_codec_ctrl::sptr codec
){
    return dboard_iface::sptr(new b100_dboard_iface(wb_iface, i2c_iface, spi_iface, clock, codec));
}

/***********************************************************************
 * Clock Rates
 **********************************************************************/
void b100_dboard_iface::set_clock_rate(unit_t unit, double rate){
    _clock_rates[unit] = rate;
    switch(unit){
    case UNIT_RX: return _clock->set_rx_dboard_clock_rate(rate);
    case UNIT_TX: return _clock->set_tx_dboard_clock_rate(rate);
    }
}

std::vector<double> b100_dboard_iface::get_clock_rates(unit_t unit){
    switch(unit){
    case UNIT_RX: return _clock->get_rx_dboard_clock_rates();
    case UNIT_TX: return _clock->get_tx_dboard_clock_rates();
    default: UHD_THROW_INVALID_CODE_PATH();
    }
}

double b100_dboard_iface::get_clock_rate(unit_t unit){
    return _clock_rates[unit];
}

void b100_dboard_iface::set_clock_enabled(unit_t unit, bool enb){
    switch(unit){
    case UNIT_RX: return _clock->enable_rx_dboard_clock(enb);
    case UNIT_TX: return _clock->enable_tx_dboard_clock(enb);
    }
}

double b100_dboard_iface::get_codec_rate(unit_t){
    return _clock->get_fpga_clock_rate();
}

/***********************************************************************
 * GPIO
 **********************************************************************/
void b100_dboard_iface::_set_pin_ctrl(unit_t unit, boost::uint16_t value){
    UHD_ASSERT_THROW(GPIO_SEL_ATR == 1); //make this assumption
    switch(unit){
    case UNIT_RX: _wb_iface->poke16(B100_REG_GPIO_RX_SEL, value); return;
    case UNIT_TX: _wb_iface->poke16(B100_REG_GPIO_TX_SEL, value); return;
    }
}

void b100_dboard_iface::_set_gpio_ddr(unit_t unit, boost::uint16_t value){
    switch(unit){
    case UNIT_RX: _wb_iface->poke16(B100_REG_GPIO_RX_DDR, value); return;
    case UNIT_TX: _wb_iface->poke16(B100_REG_GPIO_TX_DDR, value); return;
    }
}

void b100_dboard_iface::_set_gpio_out(unit_t unit, boost::uint16_t value){
    switch(unit){
    case UNIT_RX: _wb_iface->poke16(B100_REG_GPIO_RX_IO, value); return;
    case UNIT_TX: _wb_iface->poke16(B100_REG_GPIO_TX_IO, value); return;
    }
}

boost::uint16_t b100_dboard_iface::read_gpio(unit_t unit){
    switch(unit){
    case UNIT_RX: return _wb_iface->peek16(B100_REG_GPIO_RX_IO);
    case UNIT_TX: return _wb_iface->peek16(B100_REG_GPIO_TX_IO);
    default: UHD_THROW_INVALID_CODE_PATH();
    }
}

void b100_dboard_iface::_set_atr_reg(unit_t unit, atr_reg_t atr, boost::uint16_t value){
    //define mapping of unit to atr regs to register address
    static const uhd::dict<
        unit_t, uhd::dict<atr_reg_t, boost::uint32_t>
    > unit_to_atr_to_addr = map_list_of
        (UNIT_RX, map_list_of
            (ATR_REG_IDLE,        B100_REG_ATR_IDLE_RXSIDE)
            (ATR_REG_TX_ONLY,     B100_REG_ATR_INTX_RXSIDE)
            (ATR_REG_RX_ONLY,     B100_REG_ATR_INRX_RXSIDE)
            (ATR_REG_FULL_DUPLEX, B100_REG_ATR_FULL_RXSIDE)
        )
        (UNIT_TX, map_list_of
            (ATR_REG_IDLE,        B100_REG_ATR_IDLE_TXSIDE)
            (ATR_REG_TX_ONLY,     B100_REG_ATR_INTX_TXSIDE)
            (ATR_REG_RX_ONLY,     B100_REG_ATR_INRX_TXSIDE)
            (ATR_REG_FULL_DUPLEX, B100_REG_ATR_FULL_TXSIDE)
        )
    ;
    _wb_iface->poke16(unit_to_atr_to_addr[unit][atr], value);
}

void b100_dboard_iface::set_gpio_debug(unit_t unit, int which){
    //set this unit to all outputs
    this->set_gpio_ddr(unit, 0xffff);

    //calculate the debug selections
    boost::uint32_t dbg_sels = 0x0;
    int sel = (which == 0)? GPIO_SEL_DEBUG_0 : GPIO_SEL_DEBUG_1;
    for(size_t i = 0; i < 16; i++) dbg_sels |= sel << i;

    //set the debug on and which debug selection
    switch(unit){
    case UNIT_RX:
        _wb_iface->poke16(B100_REG_GPIO_RX_DBG, 0xffff);
        _wb_iface->poke16(B100_REG_GPIO_RX_SEL, dbg_sels);
        return;

    case UNIT_TX:
        _wb_iface->poke16(B100_REG_GPIO_TX_DBG, 0xffff);
        _wb_iface->poke16(B100_REG_GPIO_TX_SEL, dbg_sels);
        return;
    }
}

/***********************************************************************
 * SPI
 **********************************************************************/
/*!
 * Static function to convert a unit type to a spi slave device number.
 * \param unit the dboard interface unit type enum
 * \return the slave device number
 */
static boost::uint32_t unit_to_otw_spi_dev(dboard_iface::unit_t unit){
    switch(unit){
    case dboard_iface::UNIT_TX: return B100_SPI_SS_TX_DB;
    case dboard_iface::UNIT_RX: return B100_SPI_SS_RX_DB;
    }
    throw std::invalid_argument("unknown unit type");
}

void b100_dboard_iface::write_spi(
    unit_t unit,
    const spi_config_t &config,
    boost::uint32_t data,
    size_t num_bits
){
    _spi_iface->transact_spi(unit_to_otw_spi_dev(unit), config, data, num_bits, false /*no rb*/);
}

boost::uint32_t b100_dboard_iface::read_write_spi(
    unit_t unit,
    const spi_config_t &config,
    boost::uint32_t data,
    size_t num_bits
){
    return _spi_iface->transact_spi(unit_to_otw_spi_dev(unit), config, data, num_bits, true /*rb*/);
}

/***********************************************************************
 * I2C
 **********************************************************************/
void b100_dboard_iface::write_i2c(boost::uint8_t addr, const byte_vector_t &bytes){
    return _i2c_iface->write_i2c(addr, bytes);
}

byte_vector_t b100_dboard_iface::read_i2c(boost::uint8_t addr, size_t num_bytes){
    return _i2c_iface->read_i2c(addr, num_bytes);
}

/***********************************************************************
 * Aux DAX/ADC
 **********************************************************************/
void b100_dboard_iface::write_aux_dac(dboard_iface::unit_t, aux_dac_t which, double value){
    //same aux dacs for each unit
    static const uhd::dict<aux_dac_t, b100_codec_ctrl::aux_dac_t> which_to_aux_dac = map_list_of
        (AUX_DAC_A, b100_codec_ctrl::AUX_DAC_A)
        (AUX_DAC_B, b100_codec_ctrl::AUX_DAC_B)
        (AUX_DAC_C, b100_codec_ctrl::AUX_DAC_C)
        (AUX_DAC_D, b100_codec_ctrl::AUX_DAC_D)
    ;
    _codec->write_aux_dac(which_to_aux_dac[which], value);
}

double b100_dboard_iface::read_aux_adc(dboard_iface::unit_t unit, aux_adc_t which){
    static const uhd::dict<
        unit_t, uhd::dict<aux_adc_t, b100_codec_ctrl::aux_adc_t>
    > unit_to_which_to_aux_adc = map_list_of
        (UNIT_RX, map_list_of
            (AUX_ADC_A, b100_codec_ctrl::AUX_ADC_A1)
            (AUX_ADC_B, b100_codec_ctrl::AUX_ADC_B1)
        )
        (UNIT_TX, map_list_of
            (AUX_ADC_A, b100_codec_ctrl::AUX_ADC_A2)
            (AUX_ADC_B, b100_codec_ctrl::AUX_ADC_B2)
        )
    ;
    return _codec->read_aux_adc(unit_to_which_to_aux_adc[unit][which]);
}

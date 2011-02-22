//
// Copyright 2010-2011 Epiq Solutions
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

// No RX IO Pins Used

// RX IO Functions

#include <uhd/utils/static.hpp>
#include <uhd/utils/assert.hpp>
#include <uhd/utils/algorithm.hpp>
#include <uhd/utils/warning.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/usrp/subdev_props.hpp>
#include <uhd/usrp/dboard_base.hpp>
#include <uhd/usrp/dboard_manager.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <utility>
#include <cmath>

using namespace uhd;
using namespace uhd::usrp;
using namespace boost::assign;

/* The following defines specify the address map provided by the
   Bitshark card. These registers are all accessed over I2C. */
#define RF_CENTER_FREQ_REG 0x00
#define RF_CHAN_FILTER_BW_REG 0x01
#define RF_GAIN_REG 0x02
#define BB_GAIN_REG 0x03
#define ADF4350_REG 0x10
#define SKY73202_REG 0x11
#define CLOCK_SCHEME_REG 0x20

#define NUM_BYTES_IN_I2C_CMD 9
/* The following table lists the registers provided by the BURX board that
   are accessible over I2C:
   --------------------------------------------------------
   |RegAddr: 0x00-RF Center Freq register |
       |4-bytes 0x00|
       |4-byte unsigned RF center freq (in KHz)|
   |RegAddr: 0x01-RF channel filter bandwidth register |
       |4-bytes 0x00|
       |4-byte unsigned RF channel filter bw (in KHz)|
   |RegAddr: 0x02-RF gain register |
       |7-bytes 0x00|
       |1-byte signed RF gain (in dB)|
   |RegAddr: 0x03-Baseband gain register (not currently supported) |
       |4-bytes 0x00|
       |4-byte signed baseband filter gain (in dB)|
   |RegAddr: 0x10-ADF4350 register |
       |4-bytes 0x00|
       |4-byte ADF4350 register value (actual ADF4350 reg addr embedded 
        within 4-byte value)|
   |RegAddr: 0x11-SKY73202 register |
       |5-bytes 0x00|
       |1-byte reg 0 of SKY73202 |
       |1-byte reg 1 of SKY73202 |
       |1-byte reg 2 of SKY73202 |
   |RegAddr: 0x20-Clock Scheme |
       |3-bytes 0x00|
       |1-byte indicating clocking scheme:
        -0x00 -> BURX local TCXO off, BURX accepts ref clock from
	         USRP (freq of USRP's ref clock specified in bytes 2-5)
	-0x01 -> BURX local TCXO on, BURX uses its local TCXO as its ref
	         clock, TCXO signal output for use by USRP |
       |4-byte USRP ref clock freq in hz (only needed if byte 1 set to 0x00) |
       
  ---------------------------------------------------------------------------
   
   As an example, lets say the client wants to set an RF center freq of
   1000 MHz.  In KHz, this translates to 1000000 (resolution is only down to
   steps of 1 KHz), which is 0x000F4240 in hex.  So the complete 9-byte I2C 
   sequence that the client should send is as follows:
   byte 0: 0x00-register 0x00 is the target of the write operation
   bytes 1-4: 0x00 (padding)
   byte 5: 0x40 (LSB of the 1000000 KHz value, in hex)
   byte 6: 0x42
   byte 7: 0x0F
   byte 8: 0x00 (MSB of the 1000000 KHz value, in hex)
   
*/

/***********************************************************************
 * The BURX constants
 **********************************************************************/
static const bool burx_debug = false;

static const freq_range_t burx_freq_range(0.3e9, 4.0e9);

static const prop_names_t burx_antennas = list_of("J5");

static const uhd::dict<std::string, gain_range_t> burx_gain_ranges = map_list_of
    ("RF_GAIN", gain_range_t(0, 42, 6))
;

/***********************************************************************
 * The BURX dboard class
 **********************************************************************/
class burx : public rx_dboard_base{
public:
    burx(ctor_args_t args);
    ~burx(void);

    void rx_get(const wax::obj &key, wax::obj &val);
    void rx_set(const wax::obj &key, const wax::obj &val);

private:
    double _lo_freq;
    double _bandwidth;
    uhd::dict<std::string, double> _gains;
    boost::uint8_t _burx_addr(void){
        return (this->get_iface()->get_special_props().mangle_i2c_addrs)? 0x45 : 0x47;
    };

    void set_lo_freq(double target_freq);
    void set_gain(double gain, const std::string &name);
    void set_bandwidth(double bandwidth);

    void send_cmd(const byte_vector_t &cmd_vector)
    {	
	if (burx_debug) std::cerr << boost::format(
	    "BURX debug: called send_cmd"
	    ) << std::endl;
	/* send the data */
	this->get_iface()->write_i2c(_burx_addr(), cmd_vector);
    }

    void set_clock_scheme(boost::uint8_t clock_scheme, boost::uint32_t ref_clk_freq)
    {
	char val[4];
	uint8_t i;

	if (burx_debug) std::cerr << boost::format(
	    "BURX debug: called set_clock_scheme with scheme %d and clk rate of %d Hz"
	    ) % int(clock_scheme) % ref_clk_freq << std::endl;

	if (clock_scheme > 1) std::cerr << boost::format(
	    "BURX error: invalid clock scheme %d"
	    ) % int(clock_scheme) << std::endl;

	memset(val,0x00,4);
	memcpy(val,&ref_clk_freq,4);
   
	/* build the cmd structure to send down to the BURX card to
	   change the LO freq */
	byte_vector_t cmd_vector(NUM_BYTES_IN_I2C_CMD);
	
	/* does byte_vector_t guarantee to be initialized to 0?  making sure...*/
	for (i=0; i<NUM_BYTES_IN_I2C_CMD; i++)
	{
	    cmd_vector[i] = 0;
	}

	memcpy(val,&ref_clk_freq,4);
	cmd_vector[0] = CLOCK_SCHEME_REG;
	cmd_vector[4] = clock_scheme;
	cmd_vector[5] = val[0];
	cmd_vector[6] = val[1];
	cmd_vector[7] = val[2];
	cmd_vector[8] = val[3];

	send_cmd(cmd_vector);
    }
};

/***********************************************************************
 * Register the BURX dboard
 **********************************************************************/
static dboard_base::sptr make_burx(dboard_base::ctor_args_t args){
    return dboard_base::sptr(new burx(args));
}

UHD_STATIC_BLOCK(reg_burx_dboard){
    //register the factory function for the rx dbid
    dboard_manager::register_dboard(0x0070, &make_burx, "BURX");
}

/***********************************************************************
 * Structors
 **********************************************************************/
burx::burx(ctor_args_t args) : rx_dboard_base(args)
{

    //enable only the clocks we need
    this->get_iface()->set_clock_enabled(dboard_iface::UNIT_RX, true);

    //set the gpio directions and atr controls (identically)
    this->get_iface()->set_gpio_ddr(dboard_iface::UNIT_RX, 0x0); // All Inputs
    
    /* set clock scheme before anything else */
    double ref_freq = this->get_iface()->get_clock_rate(dboard_iface::UNIT_RX);
    set_clock_scheme(0,(boost::uint32_t)ref_freq);
    
    //set defaults for LO, gains, and filter bandwidth
    set_lo_freq(2e9);
    set_bandwidth(28e6); // default bandwidth from datasheet
    set_gain(28.0,"RF_GAIN");
}

burx::~burx(void)
{
}

/***********************************************************************
 * Tuning
 **********************************************************************/
void burx::set_lo_freq(double target_freq)
{
    target_freq = burx_freq_range.clip(target_freq);

    uint32_t freq_in_khz = (uint32_t)(target_freq/1000.0);
    uint8_t val[4];
    uint8_t i;

    /* build tuning structure, and send the cmd */
   if (burx_debug) std::cerr << boost::format(
        "BURX debug: set_lo_freq to %f Hz"
    ) % target_freq << std::endl;

   memset(val,0x00,4);
   memcpy(val,&freq_in_khz,4);
   
   /* build the cmd structure to send down to the BURX card to
      change the LO freq */
   byte_vector_t cmd_vector(NUM_BYTES_IN_I2C_CMD);

   /* does byte_vector_t guarantee to be initialized to 0?  making sure... */
   for (i=0; i<NUM_BYTES_IN_I2C_CMD; i++)
   {
       cmd_vector[i] = 0;
   }
   cmd_vector[0] = RF_CENTER_FREQ_REG;
   cmd_vector[5] = val[0];
   cmd_vector[6] = val[1];
   cmd_vector[7] = val[2];
   cmd_vector[8] = val[3];
   
   send_cmd(cmd_vector);

   _lo_freq = target_freq;
}

/***********************************************************************
 * Gain Handling
 **********************************************************************/

void burx::set_gain(double gain, const std::string &name)
{
    uint8_t i;

    assert_has(burx_gain_ranges.keys(), name, "burx gain name");

    /* build the cmd structure to send down to the BURX card to
       change the gain */
    byte_vector_t cmd_vector(NUM_BYTES_IN_I2C_CMD);
    
    /* does byte_vector_t guarantee to be initialized to 0?  making sure... */
    for (i=0; i<NUM_BYTES_IN_I2C_CMD; i++)
    {
	cmd_vector[i] = 0;
    }

    if (name == "RF_GAIN")
    {
	gain = burx_gain_ranges["RF_GAIN"].clip(gain);
	if (burx_debug) std::cerr << boost::format(
	    "BURX debug: set_gain for RF_GAIN %d"
	    ) % gain << std::endl;

	cmd_vector[0] = RF_GAIN_REG;
	cmd_vector[5] = (boost::uint8_t)gain;
    }
    else UHD_THROW_INVALID_CODE_PATH();

    send_cmd(cmd_vector);

    _gains[name] = gain;
}

/***********************************************************************
 * Bandwidth Handling
 **********************************************************************/
void burx::set_bandwidth(double bandwidth){
    //clip the input
    bandwidth = std::clip<double>(bandwidth, 1e6, 28e6);  

    uint16_t rf_bw_in_khz = (uint16_t)(bandwidth/1000.0);
    uint8_t val[4];
    uint8_t i;

    if (burx_debug) std::cerr << boost::format(
        "BURX debug: set_bandwidth to %d KHz"
    ) % rf_bw_in_khz << std::endl;

    /* the BURX dboard expects a 32-bit value for the channel bw, but in
       reality it is only a 16-bit value, so copy over the 16-bit value
       in KHz, but populate the cmd_vector with the full 32-bits below */
    memset(val,0x00,4);
    memcpy(val,&rf_bw_in_khz,2);
    
    /* build the cmd structure to send down to the BURX card to
       change the gain */
    byte_vector_t cmd_vector(NUM_BYTES_IN_I2C_CMD);
    
    /* does byte_vector_t guarantee to be initialized to 0?  making sure... */
    for (i=0; i<NUM_BYTES_IN_I2C_CMD; i++)
    {
	cmd_vector[i] = 0;
    }
    
    cmd_vector[0] = RF_CHAN_FILTER_BW_REG;
    cmd_vector[5] = val[0];
    cmd_vector[6] = val[1];
    cmd_vector[7] = val[2];
    cmd_vector[8] = val[3];

    send_cmd(cmd_vector);

    _bandwidth = bandwidth;
}

/***********************************************************************
 * RX Get and Set
 **********************************************************************/
void burx::rx_get(const wax::obj &key_, wax::obj &val){
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
        assert_has(_gains.keys(), key.name, "burx gain name");
        val = _gains[key.name];
        return;

    case SUBDEV_PROP_GAIN_RANGE:
        assert_has(burx_gain_ranges.keys(), key.name, "burx gain name");
        val = burx_gain_ranges[key.name];
        return;

    case SUBDEV_PROP_GAIN_NAMES:
        val = prop_names_t(burx_gain_ranges.keys());
        return;

    case SUBDEV_PROP_FREQ:
        val = _lo_freq;
        return;

    case SUBDEV_PROP_FREQ_RANGE:
        val = burx_freq_range;
        return;

    case SUBDEV_PROP_ANTENNA:
        val = std::string("J3");
        return;

    case SUBDEV_PROP_ANTENNA_NAMES:
        val = burx_antennas;
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

    case SUBDEV_PROP_LO_LOCKED:
        val = true;  /* we don't have a way to report lock state of the LO,
			so always report true */
	return;

    case SUBDEV_PROP_BANDWIDTH:
        val = _bandwidth; /* _bandwidth is full channel complex, and the BURX board
			       takes care of splitting it in half for low-pass */
        return;

    default: UHD_THROW_PROP_GET_ERROR();
    }
}

void burx::rx_set(const wax::obj &key_, const wax::obj &val){
    named_prop_t key = named_prop_t::extract(key_);

    //handle the get request conditioned on the key
    switch(key.as<subdev_prop_t>()){

    case SUBDEV_PROP_FREQ:
        this->set_lo_freq(val.as<double>());
        return;

    case SUBDEV_PROP_GAIN:
        this->set_gain(val.as<double>(), key.name);
        return;

    case SUBDEV_PROP_ENABLED:
        return; //always enabled

    case SUBDEV_PROP_BANDWIDTH:
        this->set_bandwidth(val.as<double>());
        return;

    default: UHD_THROW_PROP_SET_ERROR();
    }
}


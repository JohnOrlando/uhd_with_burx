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

#include "e100_impl.hpp"
#include "e100_regs.hpp"
#include <uhd/utils/msg.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/images.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>
#include <boost/assign/list_of.hpp>
#include <fstream>

using namespace uhd;
using namespace uhd::usrp;
namespace fs = boost::filesystem;

////////////////////////////////////////////////////////////////////////
// I2C addresses
////////////////////////////////////////////////////////////////////////
#define I2C_DEV_EEPROM  0x50 // 24LC02[45]:  7-bits 1010xxx
#define	I2C_ADDR_MBOARD (I2C_DEV_EEPROM | 0x0)
#define	I2C_ADDR_TX_DB  (I2C_DEV_EEPROM | 0x4)
#define	I2C_ADDR_RX_DB  (I2C_DEV_EEPROM | 0x5)

/***********************************************************************
 * Discovery
 **********************************************************************/
static device_addrs_t e100_find(const device_addr_t &hint){
    device_addrs_t e100_addrs;

    //return an empty list of addresses when type is set to non-usrp-e
    if (hint.has_key("type") and hint["type"] != "e100") return e100_addrs;

    //device node not provided, assume its 0
    if (not hint.has_key("node")){
        device_addr_t new_addr = hint;
        new_addr["node"] = "/dev/usrp_e0";
        return e100_find(new_addr);
    }

    //use the given device node name
    if (fs::exists(hint["node"])){
        device_addr_t new_addr;
        new_addr["type"] = "e100";
        new_addr["node"] = fs::system_complete(fs::path(hint["node"])).string();
        try{
            i2c_iface::sptr i2c_iface = e100_ctrl::make_dev_i2c_iface(E100_I2C_DEV_NODE);
            const mboard_eeprom_t mb_eeprom(*i2c_iface, mboard_eeprom_t::MAP_E100);
            new_addr["name"] = mb_eeprom["name"];
            new_addr["serial"] = mb_eeprom["serial"];
        }
        catch(const std::exception &e){
            new_addr["name"] = "";
            new_addr["serial"] = "";
        }
        if (
            (not hint.has_key("name")   or hint["name"]   == new_addr["name"]) and
            (not hint.has_key("serial") or hint["serial"] == new_addr["serial"])
        ){
            e100_addrs.push_back(new_addr);
        }
    }

    return e100_addrs;
}

/***********************************************************************
 * Make
 **********************************************************************/
static size_t hash_fpga_file(const std::string &file_path){
    size_t hash = 0;
    std::ifstream file(file_path.c_str());
    if (not file.good()) throw uhd::io_error("cannot open fpga file for read: " + file_path);
    while (file.good()) boost::hash_combine(hash, file.get());
    file.close();
    return hash;
}

static device::sptr e100_make(const device_addr_t &device_addr){
    return device::sptr(new e100_impl(device_addr));
}

UHD_STATIC_BLOCK(register_e100_device){
    device::register_device(&e100_find, &e100_make);
}

/***********************************************************************
 * Structors
 **********************************************************************/
e100_impl::e100_impl(const uhd::device_addr_t &device_addr){

    //setup the main interface into fpga
    const std::string node = device_addr["node"];
    _fpga_ctrl = e100_ctrl::make(node);

    //read the eeprom so we can determine the hardware
    _dev_i2c_iface = e100_ctrl::make_dev_i2c_iface(E100_I2C_DEV_NODE);
    const mboard_eeprom_t mb_eeprom(*_dev_i2c_iface, mboard_eeprom_t::MAP_E100);

    //determine the model string for this device
    const std::string model = device_addr.get("model", mb_eeprom.get("model", ""));
    if (model.empty()) throw uhd::runtime_error("unable to determine model");

    //extract the fpga path and compute hash
    static const uhd::dict<std::string, std::string> model_to_fpga_file_name = boost::assign::map_list_of
        ("E100", "usrp_e100_fpga_v2.bin")
        ("E110", "usrp_e110_fpga.bin")
    ;
    const std::string default_fpga_file_name = model_to_fpga_file_name[model];
    const std::string e100_fpga_image = find_image_path(device_addr.get("fpga", default_fpga_file_name));
    const boost::uint32_t file_hash = boost::uint32_t(hash_fpga_file(e100_fpga_image));

    //When the hash does not match:
    // - close the device node
    // - load the fpga bin file
    // - re-open the device node
    if (_fpga_ctrl->peek32(E100_REG_RB_MISC_TEST32) != file_hash){
        _fpga_ctrl.reset();
        e100_load_fpga(e100_fpga_image);
        _fpga_ctrl = e100_ctrl::make(node);
    }

    //setup clock control here to ensure that the FPGA has a good clock before we continue
    const double master_clock_rate = device_addr.cast<double>("master_clock_rate", E100_DEFAULT_CLOCK_RATE);
    _aux_spi_iface = e100_ctrl::make_aux_spi_iface();
    _clock_ctrl = e100_clock_ctrl::make(_aux_spi_iface, master_clock_rate);

    //Perform wishbone readback tests, these tests also write the hash
    bool test_fail = false;
    UHD_MSG(status) << "Performing wishbone readback test... " << std::flush;
    for (size_t i = 0; i < 100; i++){
        _fpga_ctrl->poke32(E100_REG_SR_MISC_TEST32, file_hash);
        test_fail = _fpga_ctrl->peek32(E100_REG_RB_MISC_TEST32) != file_hash;
        if (test_fail) break; //exit loop on any failure
    }
    UHD_MSG(status) << ((test_fail)? " fail" : "pass") << std::endl;

    if (test_fail) UHD_MSG(error) << boost::format(
        "The FPGA is either clocked improperly\n"
        "or the FPGA build is not compatible.\n"
        "Subsequent errors may follow...\n"
    );

    //check that the compatibility is correct
    const boost::uint16_t fpga_compat_num = _fpga_ctrl->peek16(E100_REG_MISC_COMPAT);
    if (fpga_compat_num != E100_FPGA_COMPAT_NUM){
        throw uhd::runtime_error(str(boost::format(
            "\nPlease update the FPGA image for your device.\n"
            "See the application notes for USRP E-Series for instructions.\n"
            "Expected FPGA compatibility number 0x%x, but got 0x%x:\n"
            "The FPGA build is not compatible with the host code build."
        ) % E100_FPGA_COMPAT_NUM % fpga_compat_num));
    }

    ////////////////////////////////////////////////////////////////////
    // Create controller objects
    ////////////////////////////////////////////////////////////////////
    _fpga_i2c_ctrl = i2c_core_100::make(_fpga_ctrl, E100_REG_SLAVE(3));
    _fpga_spi_ctrl = spi_core_100::make(_fpga_ctrl, E100_REG_SLAVE(2));
    _data_transport = e100_make_mmap_zero_copy(_fpga_ctrl);

    ////////////////////////////////////////////////////////////////////
    // Initialize the properties tree
    ////////////////////////////////////////////////////////////////////
    _tree = property_tree::make();
    _tree->create<std::string>("/name").set("E-Series Device");
    const fs_path mb_path = "/mboards/0";
    _tree->create<std::string>(mb_path / "name").set(str(boost::format("%s (euewanee)") % model));

    ////////////////////////////////////////////////////////////////////
    // setup the mboard eeprom
    ////////////////////////////////////////////////////////////////////
    _tree->create<mboard_eeprom_t>(mb_path / "eeprom")
        .set(mb_eeprom)
        .subscribe(boost::bind(&e100_impl::set_mb_eeprom, this, _1));

    ////////////////////////////////////////////////////////////////////
    // create clock control objects
    ////////////////////////////////////////////////////////////////////
    //^^^ clock created up top, just reg props here... ^^^
    _tree->create<double>(mb_path / "tick_rate")
        .publish(boost::bind(&e100_clock_ctrl::get_fpga_clock_rate, _clock_ctrl))
        .subscribe(boost::bind(&e100_impl::update_tick_rate, this, _1));

    ////////////////////////////////////////////////////////////////////
    // create codec control objects
    ////////////////////////////////////////////////////////////////////
    _codec_ctrl = e100_codec_ctrl::make(_fpga_spi_ctrl);
    const fs_path rx_codec_path = mb_path / "rx_codecs/A";
    const fs_path tx_codec_path = mb_path / "tx_codecs/A";
    _tree->create<std::string>(rx_codec_path / "name").set("ad9522");
    _tree->create<meta_range_t>(rx_codec_path / "gains/pga/range").set(e100_codec_ctrl::rx_pga_gain_range);
    _tree->create<double>(rx_codec_path / "gains/pga/value")
        .coerce(boost::bind(&e100_impl::update_rx_codec_gain, this, _1));
    _tree->create<std::string>(tx_codec_path / "name").set("ad9522");
    _tree->create<meta_range_t>(tx_codec_path / "gains/pga/range").set(e100_codec_ctrl::tx_pga_gain_range);
    _tree->create<double>(tx_codec_path / "gains/pga/value")
        .subscribe(boost::bind(&e100_codec_ctrl::set_tx_pga_gain, _codec_ctrl, _1))
        .publish(boost::bind(&e100_codec_ctrl::get_tx_pga_gain, _codec_ctrl));

    ////////////////////////////////////////////////////////////////////
    // and do the misc mboard sensors
    ////////////////////////////////////////////////////////////////////
    //none for now...
    _tree->create<int>(mb_path / "sensors"); //phony property so this dir exists

    ////////////////////////////////////////////////////////////////////
    // create frontend control objects
    ////////////////////////////////////////////////////////////////////
    _rx_fe = rx_frontend_core_200::make(_fpga_ctrl, E100_REG_SR_ADDR(UE_SR_RX_FRONT));
    _tx_fe = tx_frontend_core_200::make(_fpga_ctrl, E100_REG_SR_ADDR(UE_SR_TX_FRONT));
    //TODO lots of properties to expose here for frontends
    _tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec")
        .subscribe(boost::bind(&e100_impl::update_rx_subdev_spec, this, _1));
    _tree->create<subdev_spec_t>(mb_path / "tx_subdev_spec")
        .subscribe(boost::bind(&e100_impl::update_tx_subdev_spec, this, _1));

    ////////////////////////////////////////////////////////////////////
    // create rx dsp control objects
    ////////////////////////////////////////////////////////////////////
    _rx_dsps.push_back(rx_dsp_core_200::make(
        _fpga_ctrl, E100_REG_SR_ADDR(UE_SR_RX_DSP0), E100_REG_SR_ADDR(UE_SR_RX_CTRL0), E100_RX_SID_BASE + 0
    ));
    _rx_dsps.push_back(rx_dsp_core_200::make(
        _fpga_ctrl, E100_REG_SR_ADDR(UE_SR_RX_DSP1), E100_REG_SR_ADDR(UE_SR_RX_CTRL1), E100_RX_SID_BASE + 1
    ));
    for (size_t dspno = 0; dspno < _rx_dsps.size(); dspno++){
        _rx_dsps[dspno]->set_link_rate(E100_RX_LINK_RATE_BPS);
        _tree->access<double>(mb_path / "tick_rate")
            .subscribe(boost::bind(&rx_dsp_core_200::set_tick_rate, _rx_dsps[dspno], _1));
        fs_path rx_dsp_path = mb_path / str(boost::format("rx_dsps/%u") % dspno);
        _tree->create<double>(rx_dsp_path / "rate/value")
            .coerce(boost::bind(&rx_dsp_core_200::set_host_rate, _rx_dsps[dspno], _1))
            .subscribe(boost::bind(&e100_impl::update_rx_samp_rate, this, _1));
        _tree->create<double>(rx_dsp_path / "freq/value")
            .coerce(boost::bind(&rx_dsp_core_200::set_freq, _rx_dsps[dspno], _1));
        _tree->create<meta_range_t>(rx_dsp_path / "freq/range")
            .publish(boost::bind(&rx_dsp_core_200::get_freq_range, _rx_dsps[dspno]));
        _tree->create<stream_cmd_t>(rx_dsp_path / "stream_cmd")
            .subscribe(boost::bind(&rx_dsp_core_200::issue_stream_command, _rx_dsps[dspno], _1));
    }

    ////////////////////////////////////////////////////////////////////
    // create tx dsp control objects
    ////////////////////////////////////////////////////////////////////
    _tx_dsp = tx_dsp_core_200::make(
        _fpga_ctrl, E100_REG_SR_ADDR(UE_SR_TX_DSP), E100_REG_SR_ADDR(UE_SR_TX_CTRL), E100_TX_ASYNC_SID
    );
    _tx_dsp->set_link_rate(E100_TX_LINK_RATE_BPS);
    _tree->access<double>(mb_path / "tick_rate")
        .subscribe(boost::bind(&tx_dsp_core_200::set_tick_rate, _tx_dsp, _1));
    _tree->create<double>(mb_path / "tx_dsps/0/rate/value")
        .coerce(boost::bind(&tx_dsp_core_200::set_host_rate, _tx_dsp, _1))
        .subscribe(boost::bind(&e100_impl::update_tx_samp_rate, this, _1));
    _tree->create<double>(mb_path / "tx_dsps/0/freq/value")
        .coerce(boost::bind(&tx_dsp_core_200::set_freq, _tx_dsp, _1));
    _tree->create<meta_range_t>(mb_path / "tx_dsps/0/freq/range")
        .publish(boost::bind(&tx_dsp_core_200::get_freq_range, _tx_dsp));

    ////////////////////////////////////////////////////////////////////
    // create time control objects
    ////////////////////////////////////////////////////////////////////
    time64_core_200::readback_bases_type time64_rb_bases;
    time64_rb_bases.rb_secs_now = E100_REG_RB_TIME_NOW_SECS;
    time64_rb_bases.rb_ticks_now = E100_REG_RB_TIME_NOW_TICKS;
    time64_rb_bases.rb_secs_pps = E100_REG_RB_TIME_PPS_SECS;
    time64_rb_bases.rb_ticks_pps = E100_REG_RB_TIME_PPS_TICKS;
    _time64 = time64_core_200::make(
        _fpga_ctrl, E100_REG_SR_ADDR(UE_SR_TIME64), time64_rb_bases
    );
    _tree->access<double>(mb_path / "tick_rate")
        .subscribe(boost::bind(&time64_core_200::set_tick_rate, _time64, _1));
    _tree->create<time_spec_t>(mb_path / "time/now")
        .publish(boost::bind(&time64_core_200::get_time_now, _time64))
        .subscribe(boost::bind(&time64_core_200::set_time_now, _time64, _1));
    _tree->create<time_spec_t>(mb_path / "time/pps")
        .publish(boost::bind(&time64_core_200::get_time_last_pps, _time64))
        .subscribe(boost::bind(&time64_core_200::set_time_next_pps, _time64, _1));
    //setup time source props
    _tree->create<std::string>(mb_path / "time_source/value")
        .subscribe(boost::bind(&time64_core_200::set_time_source, _time64, _1));
    _tree->create<std::vector<std::string> >(mb_path / "time_source/options")
        .publish(boost::bind(&time64_core_200::get_time_sources, _time64));
    //setup reference source props
    _tree->create<std::string>(mb_path / "clock_source/value")
        .subscribe(boost::bind(&e100_impl::update_clock_source, this, _1));
    static const std::vector<std::string> clock_sources = boost::assign::list_of("internal")("external")("auto");
    _tree->create<std::vector<std::string> >(mb_path / "clock_source/options").set(clock_sources);

    ////////////////////////////////////////////////////////////////////
    // create dboard control objects
    ////////////////////////////////////////////////////////////////////

    //read the dboard eeprom to extract the dboard ids
    dboard_eeprom_t rx_db_eeprom, tx_db_eeprom, gdb_eeprom;
    rx_db_eeprom.load(*_fpga_i2c_ctrl, I2C_ADDR_RX_DB);
    tx_db_eeprom.load(*_fpga_i2c_ctrl, I2C_ADDR_TX_DB);
    gdb_eeprom.load(*_fpga_i2c_ctrl, I2C_ADDR_TX_DB ^ 5);

    //create the properties and register subscribers
    _tree->create<dboard_eeprom_t>(mb_path / "dboards/A/rx_eeprom")
        .set(rx_db_eeprom)
        .subscribe(boost::bind(&e100_impl::set_db_eeprom, this, "rx", _1));
    _tree->create<dboard_eeprom_t>(mb_path / "dboards/A/tx_eeprom")
        .set(tx_db_eeprom)
        .subscribe(boost::bind(&e100_impl::set_db_eeprom, this, "tx", _1));
    _tree->create<dboard_eeprom_t>(mb_path / "dboards/A/gdb_eeprom")
        .set(gdb_eeprom)
        .subscribe(boost::bind(&e100_impl::set_db_eeprom, this, "gdb", _1));

    //create a new dboard interface and manager
    _dboard_iface = make_e100_dboard_iface(_fpga_ctrl, _fpga_i2c_ctrl, _fpga_spi_ctrl, _clock_ctrl, _codec_ctrl);
    _tree->create<dboard_iface::sptr>(mb_path / "dboards/A/iface").set(_dboard_iface);
    _dboard_manager = dboard_manager::make(
        rx_db_eeprom.id,
        ((gdb_eeprom.id == dboard_id_t::none())? tx_db_eeprom : gdb_eeprom).id,
        _dboard_iface
    );
    BOOST_FOREACH(const std::string &name, _dboard_manager->get_rx_subdev_names()){
        dboard_manager::populate_prop_tree_from_subdev(
            _tree->subtree(mb_path / "dboards/A/rx_frontends" / name),
            _dboard_manager->get_rx_subdev(name)
        );
    }
    BOOST_FOREACH(const std::string &name, _dboard_manager->get_tx_subdev_names()){
        dboard_manager::populate_prop_tree_from_subdev(
            _tree->subtree(mb_path / "dboards/A/tx_frontends" / name),
            _dboard_manager->get_tx_subdev(name)
        );
    }

    //initialize io handling
    this->io_init();

    ////////////////////////////////////////////////////////////////////
    // do some post-init tasks
    ////////////////////////////////////////////////////////////////////
    _tree->access<double>(mb_path / "tick_rate").update() //update and then subscribe the clock callback
        .subscribe(boost::bind(&e100_clock_ctrl::set_fpga_clock_rate, _clock_ctrl, _1));

    //and now that the tick rate is set, init the host rates to something
    BOOST_FOREACH(const std::string &name, _tree->list(mb_path / "rx_dsps")){
        _tree->access<double>(mb_path / "rx_dsps" / name / "rate" / "value").set(1e6);
    }
    BOOST_FOREACH(const std::string &name, _tree->list(mb_path / "tx_dsps")){
        _tree->access<double>(mb_path / "tx_dsps" / name / "rate" / "value").set(1e6);
    }

    _tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(subdev_spec_t("A:"+_dboard_manager->get_rx_subdev_names()[0]));
    _tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(subdev_spec_t("A:"+_dboard_manager->get_tx_subdev_names()[0]));
    _tree->access<std::string>(mb_path / "clock_source/value").set("internal");
    _tree->access<std::string>(mb_path / "time_source/value").set("none");

}

e100_impl::~e100_impl(void){
    /* NOP */
}

double e100_impl::update_rx_codec_gain(const double gain){
    //set gain on both I and Q, readback on one
    //TODO in the future, gains should have individual control
    _codec_ctrl->set_rx_pga_gain(gain, 'A');
    _codec_ctrl->set_rx_pga_gain(gain, 'B');
    return _codec_ctrl->get_rx_pga_gain('A');
}

void e100_impl::set_mb_eeprom(const uhd::usrp::mboard_eeprom_t &mb_eeprom){
    mb_eeprom.commit(*_dev_i2c_iface, mboard_eeprom_t::MAP_E100);
}

void e100_impl::set_db_eeprom(const std::string &type, const uhd::usrp::dboard_eeprom_t &db_eeprom){
    if (type == "rx") db_eeprom.store(*_fpga_i2c_ctrl, I2C_ADDR_RX_DB);
    if (type == "tx") db_eeprom.store(*_fpga_i2c_ctrl, I2C_ADDR_TX_DB);
    if (type == "gdb") db_eeprom.store(*_fpga_i2c_ctrl, I2C_ADDR_TX_DB ^ 5);
}

void e100_impl::update_clock_source(const std::string &source){
    if      (source == "auto")     _clock_ctrl->use_auto_ref();
    else if (source == "internal") _clock_ctrl->use_internal_ref();
    else if (source == "external") _clock_ctrl->use_external_ref();
    else throw uhd::runtime_error("unhandled clock configuration reference source: " + source);
}

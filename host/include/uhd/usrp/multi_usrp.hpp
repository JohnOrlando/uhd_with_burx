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

#ifndef INCLUDED_UHD_USRP_MULTI_USRP_HPP
#define INCLUDED_UHD_USRP_MULTI_USRP_HPP

#include <uhd/config.hpp>
#include <uhd/device.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/types/clock_config.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/types/tune_result.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/usrp/dboard_iface.hpp>
#include <uhd/usrp/mboard_iface.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <vector>

namespace uhd{ namespace usrp{

/*!
 * The Multi-USRP device class:
 *
 * This class facilitates ease-of-use for most use-case scenarios.
 * The wrapper provides convenience functions to tune the devices,
 * set the dboard gains, antennas, filters, and other properties.
 * This class can be used to interface with a single USRP with
 * one or more channels, or multiple USRPs in a homogeneous setup.
 * All members take an optional parameter for board number or channel number.
 * In the single device, single channel case, these parameters can be unspecified.
 *
 * When using a single device with multiple channels:
 *  - Channel mapping is determined by the subdevice specifications
 *  - All channels share a common RX sample rate
 *  - All channels share a common TX sample rate
 *
 * When using multiple devices in a configuration:
 *  - Channel mapping is determined by the device address arguments
 *  - All boards share a common RX sample rate
 *  - All boards share a common TX sample rate
 *  - All boards share a common RX subdevice specification size
 *  - All boards share a common TX subdevice specification size
 *  - All boards must have synchronized times (see the set_time_*() calls)
 *
 * Example to setup channel mapping for multiple devices:
 * <pre>
 *
 * //create a multi_usrp with two boards in the configuration
 * device_addr_t dev_addr;
 * dev_addr["addr0"] = "192.168.10.2"
 * dev_addr["addr1"] = "192.168.10.3";
 * multi_usrp::sptr dev = multi_usrp::make(dev_addr);
 *
 * //set the board on 10.2 to use the A RX subdevice (RX channel 0)
 * dev->set_rx_subdev_spec("A:A", 0);
 *
 * //set the board on 10.3 to use the B RX subdevice (RX channel 1)
 * dev->set_rx_subdev_spec("A:B", 1);
 *
 * //set both boards to use the AB TX subdevice (TX channels 0 and 1)
 * dev->set_tx_subdev_spec("A:AB", multi_usrp::ALL_MBOARDS);
 *
 * //now that all the channels are mapped, continue with configuration...
 *
 * </pre>
 */
class UHD_API multi_usrp : boost::noncopyable{
public:
    typedef boost::shared_ptr<multi_usrp> sptr;

    //! A wildcard motherboard index
    static const size_t ALL_MBOARDS = size_t(~0);

    //! A wildcard channel index
    static const size_t ALL_CHANS = size_t(~0);

    //! A wildcard gain element name
    static const std::string ALL_GAINS;

    /*!
     * Make a new multi usrp from the device address.
     * \param dev_addr the device address
     * \return a new single usrp object
     */
    static sptr make(const device_addr_t &dev_addr);

    /*!
     * Get the underlying device object.
     * This is needed to get access to the streaming API and properties.
     * \return the device object within this single usrp
     */
    virtual device::sptr get_device(void) = 0;

    /*******************************************************************
     * Mboard methods
     ******************************************************************/

    /*!
     * Set the master clock rate.
     * This controls the rate of the clock that feeds the FPGA DSP.
     * On some devices, this re-tunes the clock to the specified rate.
     * If the specified rate is not available, this method will throw.
     * On other devices, this method notifies the software of the rate,
     * but requires the the user has made the necessary hardware change.
     * \param rate the new master clock rate in Hz
     * \param mboard the motherboard index 0 to M-1
     */
    virtual void set_master_clock_rate(double rate, size_t mboard = ALL_MBOARDS) = 0;

    /*!
     * Get the master clock rate.
     * \param mboard the motherboard index 0 to M-1
     * \return the master clock rate in Hz.
     */
    virtual double get_master_clock_rate(size_t mboard = 0) = 0;

    /*!
     * Get a printable summary for this USRP configuration.
     * \return a printable string
     */
    virtual std::string get_pp_string(void) = 0;

    /*!
     * Get canonical name for this USRP motherboard.
     * \param mboard which motherboard to query
     * \return a string representing the name
     */
    virtual std::string get_mboard_name(size_t mboard = 0) = 0;

    /*!
     * Get the current time in the usrp time registers.
     * \param mboard which motherboard to query
     * \return a timespec representing current usrp time
     */
    virtual time_spec_t get_time_now(size_t mboard = 0) = 0;

    /*!
     * Get the time when the last pps pulse occured.
     * \param mboard which motherboard to query
     * \return a timespec representing the last pps
     */
    virtual time_spec_t get_time_last_pps(size_t mboard = 0) = 0;

    /*!
     * Sets the time registers on the usrp immediately.
     *
     * If only one MIMO master is present in your configuration, set_time_now is
     * safe to use because the slave's time automatically follows the master's time.
     * Otherwise, this call cannot set the time synchronously across multiple devices.
     * Please use the set_time_next_pps or set_time_unknown_pps calls with a PPS signal.
     *
     * \param time_spec the time to latch into the usrp device
     * \param mboard the motherboard index 0 to M-1
     */
    virtual void set_time_now(const time_spec_t &time_spec, size_t mboard = ALL_MBOARDS) = 0;

    /*!
     * Set the time registers on the usrp at the next pps tick.
     * The values will not be latched in until the pulse occurs.
     * It is recommended that the user sleep(1) after calling to ensure
     * that the time registers will be in a known state prior to use.
     *
     * Note: Because this call sets the time on the "next" pps,
     * the seconds in the time spec should be current seconds + 1.
     *
     * \param time_spec the time to latch into the usrp device
     */
    virtual void set_time_next_pps(const time_spec_t &time_spec) = 0;

    /*!
     * Synchronize the times across all motherboards in this configuration.
     * Use this method to sync the times when the edge of the PPS is unknown.
     *
     * Ex: Host machine is not attached to serial port of GPSDO
     * and can therefore not query the GPSDO for the PPS edge.
     *
     * This is a 2-step process, and will take at most 2 seconds to complete.
     * Upon completion, the times will be synchronized to the time provided.
     *
     * - Step1: wait for the last pps time to transition to catch the edge
     * - Step2: set the time at the next pps (synchronous for all boards)
     *
     * \param time_spec the time to latch at the next pps after catching the edge
     */
    virtual void set_time_unknown_pps(const time_spec_t &time_spec) = 0;

    /*!
     * Are the times across all motherboards in this configuration synchronized?
     * Checks that all time registers are approximately close but not exact,
     * given that the RTT may varying for a control packet transaction.
     * \return true when all motherboards time registers are in sync
     */
    virtual bool get_time_synchronized(void) = 0;

    /*!
     * Issue a stream command to the usrp device.
     * This tells the usrp to send samples into the host.
     * See the documentation for stream_cmd_t for more info.
     *
     * With multiple devices, the first stream command in a chain of commands
     * should have a time spec in the near future and stream_now = false;
     * to ensure that the packets can be aligned by their time specs.
     *
     * \param stream_cmd the stream command to issue
     * \param chan the channel index 0 to N-1
     */
    virtual void issue_stream_cmd(const stream_cmd_t &stream_cmd, size_t chan = ALL_CHANS) = 0;

    /*!
     * Set the clock configuration for the usrp device.
     * This tells the usrp how to get a 10Mhz reference and PPS clock.
     * See the documentation for clock_config_t for more info.
     * \param clock_config the clock configuration to set
     * \param mboard which motherboard to set the config
     */
    virtual void set_clock_config(const clock_config_t &clock_config, size_t mboard = ALL_MBOARDS) = 0;

    /*!
     * Get the number of USRP motherboards in this configuration.
     */
    virtual size_t get_num_mboards(void) = 0;

    /*!
     * Get a motherboard sensor value.
     * \param name the name of the sensor
     * \param mboard the motherboard index 0 to M-1
     * \return a sensor value object
     */
    virtual sensor_value_t get_mboard_sensor(const std::string &name, size_t mboard = 0) = 0;

    /*!
     * Get a list of possible motherboard sensor names.
     * \param mboard the motherboard index 0 to M-1
     * \return a vector of sensor names
     */
    virtual std::vector<std::string> get_mboard_sensor_names(size_t mboard = 0) = 0;
    
    /*!
     * Get a handle to the mboard_iface object which controls peripheral access.
     * \return a mboard_iface::sptr object
     */
    virtual mboard_iface::sptr get_mboard_iface(size_t mboard) = 0;

    /*******************************************************************
     * RX methods
     ******************************************************************/
    /*!
     * Set the RX subdevice specification:
     * The subdev spec maps a physical part of a daughter-board to a channel number.
     * Set the subdev spec before calling into any methods with a channel number.
     * The subdev spec must be the same size across all motherboards.
     * \param spec the new subdevice specification
     * \param mboard the motherboard index 0 to M-1
     */
    virtual void set_rx_subdev_spec(const uhd::usrp::subdev_spec_t &spec, size_t mboard = ALL_MBOARDS) = 0;

    /*!
     * Get the RX subdevice specification.
     * \param mboard the motherboard index 0 to M-1
     * \return the subdevice specification in use
     */
    virtual uhd::usrp::subdev_spec_t get_rx_subdev_spec(size_t mboard = 0) = 0;

    /*!
     * Get the number of RX channels in this configuration.
     * This is the number of USRPs times the number of RX channels per board,
     * where the number of RX channels per board is homogeneous among all USRPs.
     */
    virtual size_t get_rx_num_channels(void) = 0;

    /*!
     * Get the name of the RX subdevice.
     * \param chan the channel index 0 to N-1
     * \return the subdevice name
     */
    virtual std::string get_rx_subdev_name(size_t chan = 0) = 0;

    /*!
     * Set the RX sample rate.
     * \param rate the rate in Sps
     * \param chan the channel index 0 to N-1
     */
    virtual void set_rx_rate(double rate, size_t chan = ALL_CHANS) = 0;

    /*!
     * Gets the RX sample rate.
     * \param chan the channel index 0 to N-1
     * \return the rate in Sps
     */
    virtual double get_rx_rate(size_t chan = 0) = 0;

    /*!
     * Set the RX center frequency.
     * \param tune_request tune request instructions
     * \param chan the channel index 0 to N-1
     * \return a tune result object
     */
    virtual tune_result_t set_rx_freq(
        const tune_request_t &tune_request, size_t chan = 0
    ) = 0;

    /*!
     * Get the RX center frequency.
     * \param chan the channel index 0 to N-1
     * \return the frequency in Hz
     */
    virtual double get_rx_freq(size_t chan = 0) = 0;

    /*!
     * Get the RX center frequency range.
     * \param chan the channel index 0 to N-1
     * \return a frequency range object
     */
    virtual freq_range_t get_rx_freq_range(size_t chan = 0) = 0;

    /*!
     * Set the RX gain value for the specified gain element.
     * For an empty name, distribute across all gain elements.
     * \param gain the gain in dB
     * \param name the name of the gain element
     * \param chan the channel index 0 to N-1
     */
    virtual void set_rx_gain(double gain, const std::string &name, size_t chan = 0) = 0;

    //! A convenience wrapper for setting overall RX gain
    void set_rx_gain(double gain, size_t chan = 0){
        return this->set_rx_gain(gain, ALL_GAINS, chan);
    }

    /*!
     * Get the RX gain value for the specified gain element.
     * For an empty name, sum across all gain elements.
     * \param name the name of the gain element
     * \param chan the channel index 0 to N-1
     * \return the gain in dB
     */
    virtual double get_rx_gain(const std::string &name, size_t chan = 0) = 0;

    //! A convenience wrapper for getting overall RX gain
    double get_rx_gain(size_t chan = 0){
        return this->get_rx_gain(ALL_GAINS, chan);
    }

    /*!
     * Get the RX gain range for the specified gain element.
     * For an empty name, calculate the overall gain range.
     * \param name the name of the gain element
     * \param chan the channel index 0 to N-1
     * \return a gain range object
     */
    virtual gain_range_t get_rx_gain_range(const std::string &name, size_t chan = 0) = 0;

    //! A convenience wrapper for getting overall RX gain range
    gain_range_t get_rx_gain_range(size_t chan = 0){
        return this->get_rx_gain_range(ALL_GAINS, chan);
    }

    /*!
     * Get the names of the gain elements in the RX chain.
     * Gain elements are ordered from antenna to FPGA.
     * \param chan the channel index 0 to N-1
     * \return a vector of gain element names
     */
    virtual std::vector<std::string> get_rx_gain_names(size_t chan = 0) = 0;

    /*!
     * Select the RX antenna on the subdevice.
     * \param ant the antenna name
     * \param chan the channel index 0 to N-1
     */
    virtual void set_rx_antenna(const std::string &ant, size_t chan = 0) = 0;

    /*!
     * Get the selected RX antenna on the subdevice.
     * \param chan the channel index 0 to N-1
     * \return the antenna name
     */
    virtual std::string get_rx_antenna(size_t chan = 0) = 0;

    /*!
     * Get a list of possible RX antennas on the subdevice.
     * \param chan the channel index 0 to N-1
     * \return a vector of antenna names
     */
    virtual std::vector<std::string> get_rx_antennas(size_t chan = 0) = 0;

    /*!
     * Get the locked status of the LO on the subdevice.
     * \param chan the channel index 0 to N-1
     * \return true for locked
     */
    UHD_DEPRECATED bool get_rx_lo_locked(size_t chan = 0){
        return this->get_rx_sensor("lo_locked", chan).to_bool();
    }

    /*!
     * Set the RX bandwidth on the subdevice.
     * \param bandwidth the bandwidth in Hz
     * \param chan the channel index 0 to N-1
     */
    virtual void set_rx_bandwidth(double bandwidth, size_t chan = 0) = 0;

    /*!
     * Get the RX bandwidth on the subdevice.
     * \param chan the channel index 0 to N-1
     * \return the bandwidth in Hz
     */
    virtual double get_rx_bandwidth(size_t chan = 0) = 0;

    /*!
     * Read the RSSI value on the RX subdevice.
     * \param chan the channel index 0 to N-1
     * \return the rssi in dB
     * \throw exception if RSSI readback not supported
     */
    UHD_DEPRECATED double read_rssi(size_t chan = 0){
        return this->get_rx_sensor("rssi", chan).to_real();
    }

    /*!
     * Get the dboard interface object for the RX subdevice.
     * The dboard interface gives access to GPIOs, SPI, I2C, low-speed ADC and DAC.
     * Use at your own risk!
     * \param chan the channel index 0 to N-1
     * \return the dboard interface sptr
     */
    virtual dboard_iface::sptr get_rx_dboard_iface(size_t chan = 0) = 0;

    /*!
     * Get an RX subdevice sensor value.
     * \param name the name of the sensor
     * \param chan the channel index 0 to N-1
     * \return a sensor value object
     */
    virtual sensor_value_t get_rx_sensor(const std::string &name, size_t chan = 0) = 0;

    /*!
     * Get a list of possible RX subdevice sensor names.
     * \param chan the channel index 0 to N-1
     * \return a vector of sensor names
     */
    virtual std::vector<std::string> get_rx_sensor_names(size_t chan = 0) = 0;

    /*******************************************************************
     * TX methods
     ******************************************************************/
    /*!
     * Set the TX subdevice specification:
     * The subdev spec maps a physical part of a daughter-board to a channel number.
     * Set the subdev spec before calling into any methods with a channel number.
     * The subdev spec must be the same size across all motherboards.
     * \param spec the new subdevice specification
     * \param mboard the motherboard index 0 to M-1
     */
    virtual void set_tx_subdev_spec(const uhd::usrp::subdev_spec_t &spec, size_t mboard = ALL_MBOARDS) = 0;

    /*!
     * Get the TX subdevice specification.
     * \param mboard the motherboard index 0 to M-1
     * \return the subdevice specification in use
     */
    virtual uhd::usrp::subdev_spec_t get_tx_subdev_spec(size_t mboard = 0) = 0;

    /*!
     * Get the number of TX channels in this configuration.
     * This is the number of USRPs times the number of TX channels per board,
     * where the number of TX channels per board is homogeneous among all USRPs.
     */
    virtual size_t get_tx_num_channels(void) = 0;

    /*!
     * Get the name of the TX subdevice.
     * \param chan the channel index 0 to N-1
     * \return the subdevice name
     */
    virtual std::string get_tx_subdev_name(size_t chan = 0) = 0;

    /*!
     * Set the TX sample rate.
     * \param rate the rate in Sps
     * \param chan the channel index 0 to N-1
     */
    virtual void set_tx_rate(double rate, size_t chan = ALL_CHANS) = 0;

    /*!
     * Gets the TX sample rate.
     * \param chan the channel index 0 to N-1
     * \return the rate in Sps
     */
    virtual double get_tx_rate(size_t chan = 0) = 0;

    /*!
     * Set the TX center frequency.
     * \param tune_request tune request instructions
     * \param chan the channel index 0 to N-1
     * \return a tune result object
     */
    virtual tune_result_t set_tx_freq(
        const tune_request_t &tune_request, size_t chan = 0
    ) = 0;

    /*!
     * Get the TX center frequency.
     * \param chan the channel index 0 to N-1
     * \return the frequency in Hz
     */
    virtual double get_tx_freq(size_t chan = 0) = 0;

    /*!
     * Get the TX center frequency range.
     * \param chan the channel index 0 to N-1
     * \return a frequency range object
     */
    virtual freq_range_t get_tx_freq_range(size_t chan = 0) = 0;

    /*!
     * Set the TX gain value for the specified gain element.
     * For an empty name, distribute across all gain elements.
     * \param gain the gain in dB
     * \param name the name of the gain element
     * \param chan the channel index 0 to N-1
     */
    virtual void set_tx_gain(double gain, const std::string &name, size_t chan = 0) = 0;

    //! A convenience wrapper for setting overall TX gain
    void set_tx_gain(double gain, size_t chan = 0){
        return this->set_tx_gain(gain, ALL_GAINS, chan);
    }

    /*!
     * Get the TX gain value for the specified gain element.
     * For an empty name, sum across all gain elements.
     * \param name the name of the gain element
     * \param chan the channel index 0 to N-1
     * \return the gain in dB
     */
    virtual double get_tx_gain(const std::string &name, size_t chan = 0) = 0;

    //! A convenience wrapper for getting overall TX gain
    double get_tx_gain(size_t chan = 0){
        return this->get_tx_gain(ALL_GAINS, chan);
    }

    /*!
     * Get the TX gain range for the specified gain element.
     * For an empty name, calculate the overall gain range.
     * \param name the name of the gain element
     * \param chan the channel index 0 to N-1
     * \return a gain range object
     */
    virtual gain_range_t get_tx_gain_range(const std::string &name, size_t chan = 0) = 0;

    //! A convenience wrapper for getting overall TX gain range
    gain_range_t get_tx_gain_range(size_t chan = 0){
        return this->get_tx_gain_range(ALL_GAINS, chan);
    }

    /*!
     * Get the names of the gain elements in the TX chain.
     * Gain elements are ordered from antenna to FPGA.
     * \param chan the channel index 0 to N-1
     * \return a vector of gain element names
     */
    virtual std::vector<std::string> get_tx_gain_names(size_t chan = 0) = 0;

    /*!
     * Select the TX antenna on the subdevice.
     * \param ant the antenna name
     * \param chan the channel index 0 to N-1
     */
    virtual void set_tx_antenna(const std::string &ant, size_t chan = 0) = 0;

    /*!
     * Get the selected TX antenna on the subdevice.
     * \param chan the channel index 0 to N-1
     * \return the antenna name
     */
    virtual std::string get_tx_antenna(size_t chan = 0) = 0;

    /*!
     * Get a list of possible TX antennas on the subdevice.
     * \param chan the channel index 0 to N-1
     * \return a vector of antenna names
     */
    virtual std::vector<std::string> get_tx_antennas(size_t chan = 0) = 0;

    /*!
     * Get the locked status of the LO on the subdevice.
     * \param chan the channel index 0 to N-1
     * \return true for locked
     */
    UHD_DEPRECATED bool get_tx_lo_locked(size_t chan = 0){
        return this->get_tx_sensor("lo_locked", chan).to_bool();
    }

    /*!
     * Set the TX bandwidth on the subdevice.
     * \param bandwidth the bandwidth in Hz
     * \param chan the channel index 0 to N-1
     */
    virtual void set_tx_bandwidth(double bandwidth, size_t chan = 0) = 0;

    /*!
     * Get the TX bandwidth on the subdevice.
     * \param chan the channel index 0 to N-1
     * \return the bandwidth in Hz
     */
    virtual double get_tx_bandwidth(size_t chan = 0) = 0;

    /*!
     * Get the dboard interface object for the TX subdevice.
     * The dboard interface gives access to GPIOs, SPI, I2C, low-speed ADC and DAC.
     * Use at your own risk!
     * \param chan the channel index 0 to N-1
     * \return the dboard interface sptr
     */
    virtual dboard_iface::sptr get_tx_dboard_iface(size_t chan = 0) = 0;

    /*!
     * Get an TX subdevice sensor value.
     * \param name the name of the sensor
     * \param chan the channel index 0 to N-1
     * \return a sensor value object
     */
    virtual sensor_value_t get_tx_sensor(const std::string &name, size_t chan = 0) = 0;

    /*!
     * Get a list of possible TX subdevice sensor names.
     * \param chan the channel index 0 to N-1
     * \return a vector of sensor names
     */
    virtual std::vector<std::string> get_tx_sensor_names(size_t chan = 0) = 0;
};

}}

#endif /* INCLUDED_UHD_USRP_MULTI_USRP_HPP */

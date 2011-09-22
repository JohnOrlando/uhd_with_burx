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

#ifndef INCLUDED_B100_CTRL_HPP
#define INCLUDED_B100_CTRL_HPP

#include "wb_iface.hpp"
#include <uhd/types/serial.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

class e100_ctrl : boost::noncopyable, public wb_iface{
public:
    typedef boost::shared_ptr<e100_ctrl> sptr;

    //! Make a new controller for E100
    static sptr make(const std::string &node);

    //! Make an i2c iface for the i2c device node
    static uhd::i2c_iface::sptr make_dev_i2c_iface(const std::string &node);

    //! Make an i2c iface for the i2c device node
    static uhd::spi_iface::sptr make_aux_spi_iface(void);

    virtual void ioctl(int request, void *mem) = 0;

    virtual int get_file_descriptor(void) = 0;

};

#endif /* INCLUDED_B100_CTRL_HPP */

//
// Copyright 2011-2011 Ettus Research LLC
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

#include <uhd/convert.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/foreach.hpp>
#include <boost/cstdint.hpp>
#include <complex>
#include <vector>
#include <cstdlib>
#include <iostream>

using namespace uhd;

//typedefs for complex types
typedef std::complex<boost::int16_t> sc16_t;
typedef std::complex<float> fc32_t;
typedef std::complex<double> fc64_t;

#define MY_CHECK_CLOSE(a, b, f) if ((std::abs(a) > (f))) \
    BOOST_CHECK_CLOSE_FRACTION(a, b, f)

/***********************************************************************
 * Loopback runner:
 *    convert input buffer into intermediate buffer
 *    convert intermediate buffer into output buffer
 **********************************************************************/
template <typename Range> static void loopback(
    size_t nsamps,
    const io_type_t &io_type,
    const otw_type_t &otw_type,
    const Range &input,
    Range &output
){
    //item32 is largest device type
    std::vector<boost::uint32_t> interm(nsamps);

    std::vector<const void *> input0(1, &input[0]), input1(1, &interm[0]);
    std::vector<void *> output0(1, &interm[0]), output1(1, &output[0]);

    //convert to intermediate type
    convert::get_converter_cpu_to_otw(
        io_type, otw_type, input0.size(), output0.size()
    )(input0, output0, nsamps, 32767.);

    //convert back to host type
    convert::get_converter_otw_to_cpu(
        io_type, otw_type, input1.size(), output1.size()
    )(input1, output1, nsamps, 1/32767.);
}

/***********************************************************************
 * Test short conversion
 **********************************************************************/
static void test_convert_types_sc16(
    size_t nsamps,
    const io_type_t &io_type,
    const otw_type_t &otw_type
){
    //fill the input samples
    std::vector<sc16_t> input(nsamps), output(nsamps);
    BOOST_FOREACH(sc16_t &in, input) in = sc16_t(
        std::rand()-(RAND_MAX/2),
        std::rand()-(RAND_MAX/2)
    );

    //run the loopback and test
    loopback(nsamps, io_type, otw_type, input, output);
    BOOST_CHECK_EQUAL_COLLECTIONS(input.begin(), input.end(), output.begin(), output.end());
}

BOOST_AUTO_TEST_CASE(test_convert_types_be_sc16){
    io_type_t io_type(io_type_t::COMPLEX_INT16);
    otw_type_t otw_type;
    otw_type.byteorder = otw_type_t::BO_BIG_ENDIAN;
    otw_type.width = 16;

    //try various lengths to test edge cases
    for (size_t nsamps = 1; nsamps < 16; nsamps++){
        test_convert_types_sc16(nsamps, io_type, otw_type);
    }
}

BOOST_AUTO_TEST_CASE(test_convert_types_le_sc16){
    io_type_t io_type(io_type_t::COMPLEX_INT16);
    otw_type_t otw_type;
    otw_type.byteorder = otw_type_t::BO_LITTLE_ENDIAN;
    otw_type.width = 16;

    //try various lengths to test edge cases
    for (size_t nsamps = 1; nsamps < 16; nsamps++){
        test_convert_types_sc16(nsamps, io_type, otw_type);
    }
}

/***********************************************************************
 * Test float conversion
 **********************************************************************/
template <typename data_type>
static void test_convert_types_for_floats(
    size_t nsamps,
    const io_type_t &io_type,
    const otw_type_t &otw_type
){
    typedef typename data_type::value_type value_type;

    //fill the input samples
    std::vector<data_type> input(nsamps), output(nsamps);
    BOOST_FOREACH(data_type &in, input) in = data_type(
        (std::rand()/value_type(RAND_MAX/2)) - 1,
        (std::rand()/value_type(RAND_MAX/2)) - 1
    );

    //run the loopback and test
    loopback(nsamps, io_type, otw_type, input, output);
    for (size_t i = 0; i < nsamps; i++){
        MY_CHECK_CLOSE(input[i].real(), output[i].real(), value_type(0.01));
        MY_CHECK_CLOSE(input[i].imag(), output[i].imag(), value_type(0.01));
    }
}

BOOST_AUTO_TEST_CASE(test_convert_types_be_fc32){
    io_type_t io_type(io_type_t::COMPLEX_FLOAT32);
    otw_type_t otw_type;
    otw_type.byteorder = otw_type_t::BO_BIG_ENDIAN;
    otw_type.width = 16;

    //try various lengths to test edge cases
    for (size_t nsamps = 1; nsamps < 16; nsamps++){
        test_convert_types_for_floats<fc32_t>(nsamps, io_type, otw_type);
    }
}

BOOST_AUTO_TEST_CASE(test_convert_types_le_fc32){
    io_type_t io_type(io_type_t::COMPLEX_FLOAT32);
    otw_type_t otw_type;
    otw_type.byteorder = otw_type_t::BO_LITTLE_ENDIAN;
    otw_type.width = 16;

    //try various lengths to test edge cases
    for (size_t nsamps = 1; nsamps < 16; nsamps++){
        test_convert_types_for_floats<fc32_t>(nsamps, io_type, otw_type);
    }
}

BOOST_AUTO_TEST_CASE(test_convert_types_be_fc64){
    io_type_t io_type(io_type_t::COMPLEX_FLOAT64);
    otw_type_t otw_type;
    otw_type.byteorder = otw_type_t::BO_BIG_ENDIAN;
    otw_type.width = 16;

    //try various lengths to test edge cases
    for (size_t nsamps = 1; nsamps < 16; nsamps++){
        test_convert_types_for_floats<fc64_t>(nsamps, io_type, otw_type);
    }
}

BOOST_AUTO_TEST_CASE(test_convert_types_le_fc64){
    io_type_t io_type(io_type_t::COMPLEX_FLOAT64);
    otw_type_t otw_type;
    otw_type.byteorder = otw_type_t::BO_LITTLE_ENDIAN;
    otw_type.width = 16;

    //try various lengths to test edge cases
    for (size_t nsamps = 1; nsamps < 16; nsamps++){
        test_convert_types_for_floats<fc64_t>(nsamps, io_type, otw_type);
    }
}

/***********************************************************************
 * Test float to short conversion loopback
 **********************************************************************/
BOOST_AUTO_TEST_CASE(test_convert_types_fc32_to_sc16){
    io_type_t io_type_in(io_type_t::COMPLEX_FLOAT32);
    io_type_t io_type_out(io_type_t::COMPLEX_INT16);

    otw_type_t otw_type;
    otw_type.byteorder = otw_type_t::BO_NATIVE;
    otw_type.width = 16;

    const size_t nsamps = 13;
    std::vector<fc32_t> input(nsamps);
    BOOST_FOREACH(fc32_t &in, input) in = fc32_t(
        (std::rand()/float(RAND_MAX/2)) - 1,
        (std::rand()/float(RAND_MAX/2)) - 1
    );
    std::vector<boost::uint32_t> interm(nsamps);
    std::vector<sc16_t> output(nsamps);

    std::vector<const void *> input0(1, &input[0]), input1(1, &interm[0]);
    std::vector<void *> output0(1, &interm[0]), output1(1, &output[0]);

    //convert float to intermediate
    convert::get_converter_cpu_to_otw(
        io_type_in, otw_type, input0.size(), output0.size()
    )(input0, output0, nsamps, 32767.);

    //convert intermediate to short
    convert::get_converter_otw_to_cpu(
        io_type_out, otw_type, input1.size(), output1.size()
    )(input1, output1, nsamps, 1/32767.);

    //test that the inputs and outputs match
    for (size_t i = 0; i < nsamps; i++){
        MY_CHECK_CLOSE(input[i].real(), output[i].real()/float(32767), float(0.01));
        MY_CHECK_CLOSE(input[i].imag(), output[i].imag()/float(32767), float(0.01));
    }
}

/***********************************************************************
 * Test short to float conversion loopback
 **********************************************************************/
BOOST_AUTO_TEST_CASE(test_convert_types_sc16_to_fc32){
    io_type_t io_type_in(io_type_t::COMPLEX_INT16);
    io_type_t io_type_out(io_type_t::COMPLEX_FLOAT32);

    otw_type_t otw_type;
    otw_type.byteorder = otw_type_t::BO_NATIVE;
    otw_type.width = 16;

    const size_t nsamps = 13;
    std::vector<sc16_t> input(nsamps);
    BOOST_FOREACH(sc16_t &in, input) in = sc16_t(
        std::rand()-(RAND_MAX/2),
        std::rand()-(RAND_MAX/2)
    );
    std::vector<boost::uint32_t> interm(nsamps);
    std::vector<fc32_t> output(nsamps);

    std::vector<const void *> input0(1, &input[0]), input1(1, &interm[0]);
    std::vector<void *> output0(1, &interm[0]), output1(1, &output[0]);

    //convert short to intermediate
    convert::get_converter_cpu_to_otw(
        io_type_in, otw_type, input0.size(), output0.size()
    )(input0, output0, nsamps, 32767.);

    //convert intermediate to float
    convert::get_converter_otw_to_cpu(
        io_type_out, otw_type, input1.size(), output1.size()
    )(input1, output1, nsamps, 1/32767.);

    //test that the inputs and outputs match
    for (size_t i = 0; i < nsamps; i++){
        MY_CHECK_CLOSE(input[i].real()/float(32767), output[i].real(), float(0.01));
        MY_CHECK_CLOSE(input[i].imag()/float(32767), output[i].imag(), float(0.01));
    }
}

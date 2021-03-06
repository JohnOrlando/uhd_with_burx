//
// Copyright 2010 Ettus Research LLC
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

#include <uhd/transport/udp_simple.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <iostream>

using namespace uhd::transport;

/***********************************************************************
 * Helper Functions
 **********************************************************************/
/*!
 * Wait for available data or timeout.
 * \param socket the asio socket
 * \param timeout the timeout in seconds
 * \return false for timeout, true for data
 */
static bool wait_available(
    boost::asio::ip::udp::socket &socket, double timeout
){
    #if defined(UHD_PLATFORM_LINUX) || defined(UHD_PLATFORM_WIN32)

    //setup timeval for timeout
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = long(timeout*1e6);

    //setup rset for timeout
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(socket.native(), &rset);

    return ::select(socket.native()+1, &rset, NULL, NULL, &tv) > 0;

    #else /*defined(UHD_PLATFORM_LINUX) || defined(UHD_PLATFORM_WIN32)*/

    //FIXME: why does select fail on macintosh?
    for (size_t i = 0; i < size_t(timeout*1e3); i++){
        if (socket.available()) return true;
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
    return false;

    #endif /*defined(UHD_PLATFORM_LINUX) || defined(UHD_PLATFORM_WIN32)*/
}

/***********************************************************************
 * UDP connected implementation class
 **********************************************************************/
class udp_connected_impl : public udp_simple{
public:
    //structors
    udp_connected_impl(const std::string &addr, const std::string &port);
    ~udp_connected_impl(void);

    //send/recv
    size_t send(const boost::asio::const_buffer &);
    size_t recv(const boost::asio::mutable_buffer &, double);

private:
    boost::asio::ip::udp::socket   *_socket;
    boost::asio::io_service        _io_service;
};

udp_connected_impl::udp_connected_impl(const std::string &addr, const std::string &port){
    //std::cout << boost::format("Creating udp transport for %s %s") % addr % port << std::endl;

    // resolve the address
    boost::asio::ip::udp::resolver resolver(_io_service);
    boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), addr, port);
    boost::asio::ip::udp::endpoint receiver_endpoint = *resolver.resolve(query);

    // Create, open, and connect the socket
    _socket = new boost::asio::ip::udp::socket(_io_service);
    _socket->open(boost::asio::ip::udp::v4());
    _socket->connect(receiver_endpoint);
}

udp_connected_impl::~udp_connected_impl(void){
    delete _socket;
}

size_t udp_connected_impl::send(const boost::asio::const_buffer &buff){
    return _socket->send(boost::asio::buffer(buff));
}

size_t udp_connected_impl::recv(const boost::asio::mutable_buffer &buff, double timeout){
    if (not wait_available(*_socket, timeout)) return 0;
    return _socket->receive(boost::asio::buffer(buff));
}

/***********************************************************************
 * UDP broadcast implementation class
 **********************************************************************/
class udp_broadcast_impl : public udp_simple{
public:
    //structors
    udp_broadcast_impl(const std::string &addr, const std::string &port);
    ~udp_broadcast_impl(void);

    //send/recv
    size_t send(const boost::asio::const_buffer &);
    size_t recv(const boost::asio::mutable_buffer &, double);

private:
    boost::asio::ip::udp::socket   *_socket;
    boost::asio::ip::udp::endpoint _receiver_endpoint;
    boost::asio::io_service        _io_service;
};

udp_broadcast_impl::udp_broadcast_impl(const std::string &addr, const std::string &port){
    //std::cout << boost::format("Creating udp transport for %s %s") % addr % port << std::endl;

    // resolve the address
    boost::asio::ip::udp::resolver resolver(_io_service);
    boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), addr, port);
    _receiver_endpoint = *resolver.resolve(query);

    // Create and open the socket
    _socket = new boost::asio::ip::udp::socket(_io_service);
    _socket->open(boost::asio::ip::udp::v4());

    // Allow broadcasting
    boost::asio::socket_base::broadcast option(true);
    _socket->set_option(option);

}

udp_broadcast_impl::~udp_broadcast_impl(void){
    delete _socket;
}

size_t udp_broadcast_impl::send(const boost::asio::const_buffer &buff){
    return _socket->send_to(boost::asio::buffer(buff), _receiver_endpoint);
}

size_t udp_broadcast_impl::recv(const boost::asio::mutable_buffer &buff, double timeout){
    if (not wait_available(*_socket, timeout)) return 0;
    boost::asio::ip::udp::endpoint sender_endpoint;
    return _socket->receive_from(boost::asio::buffer(buff), sender_endpoint);
}

/***********************************************************************
 * UDP public make functions
 **********************************************************************/
udp_simple::sptr udp_simple::make_connected(
    const std::string &addr, const std::string &port
){
    return sptr(new udp_connected_impl(addr, port));
}

udp_simple::sptr udp_simple::make_broadcast(
    const std::string &addr, const std::string &port
){
    return sptr(new udp_broadcast_impl(addr, port));
}

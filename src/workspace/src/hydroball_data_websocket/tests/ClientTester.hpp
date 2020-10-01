#ifndef CLIENTTESTER_HPP
#define CLIENTTESTER_HPP

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

class ClientTester {

public:
    // implement handlers for testing
    virtual void on_open(websocketpp::client<websocketpp::config::asio_client> * c, websocketpp::connection_hdl hdl)=0;
    virtual void on_fail(websocketpp::client<websocketpp::config::asio_client> * c, websocketpp::connection_hdl hdl)=0;
    virtual void on_message(websocketpp::client<websocketpp::config::asio_client> * c, websocketpp::connection_hdl hdl, websocketpp::config::asio_client::message_type::ptr msg)=0;
    virtual void on_close(websocketpp::client<websocketpp::config::asio_client> * c, websocketpp::connection_hdl hdl)=0;
};

#endif
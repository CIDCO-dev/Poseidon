#ifndef CLIENTWPP
#define CLIENTWPP

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include "ros/ros.h"

class ClientWpp {

public:
    typedef websocketpp::client<websocketpp::config::asio_client> client;
    typedef websocketpp::lib::lock_guard<websocketpp::lib::mutex> scoped_lock;

    ClientWpp(std::string & uri) : uri(uri), done(false), open(false) {
        //Don't use websocketpp logs, use ROS logs instead
        clt.clear_access_channels(websocketpp::log::alevel::all);

        clt.init_asio();

        using websocketpp::lib::bind;
        clt.set_open_handler(bind(&ClientWpp::on_open, this, std::placeholders::_1));
        clt.set_close_handler(bind(&ClientWpp::on_close, this, std::placeholders::_1));
        clt.set_fail_handler(bind(&ClientWpp::on_fail, this, std::placeholders::_1));
    }

    ~ClientWpp() {}

    void on_open(websocketpp::connection_hdl) {
        scoped_lock guard(lock);
        open = true;
    }

    void on_close(websocketpp::connection_hdl) {
        scoped_lock guard(lock);
        done = true;
    }

    void on_fail(websocketpp::connection_hdl) {
        ROS_ERROR_STREAM("Connection failed: " << uri);

        scoped_lock guard(lock);
        done = true;
    }

    void stop() {
        ROS_ERROR_STREAM("Stopping client Connection to: " << uri);
        std::string closingMessage = "Client has closed the connection";

        if(open) {
            scoped_lock guard(lock);
            websocketpp::lib::error_code ec_close_connection;
            clt.close(con_hdl, websocketpp::close::status::normal, closingMessage, ec_close_connection);
            if(ec_close_connection) {
                ROS_ERROR_STREAM("failed to close connection: " << ec_close_connection.message());
            }
        }

        scoped_lock guard(lock);
        done = true;
    }

    void start_client() {
        websocketpp::lib::error_code ec;
        client::connection_ptr con = clt.get_connection(uri, ec);

        if(ec) {
            //log error
            ROS_ERROR_STREAM("Can't establish connection to: " << uri);
            return;
        }

        con_hdl = con->get_handle();

        clt.connect(con);

        std::thread client_thread(std::bind(&client::run,&clt));
        std::thread business_logic_thread(std::bind(&ClientWpp::business,this));

        client_thread.join();
        business_logic_thread.join();
    }

protected:

    virtual void business() = 0;
     /* Example business:
     {
        while(1) {
            bool wait = false;

            { // check status scope
                scoped_lock guard(lock);
                if(done) {
                    break;
                }

                if(!open) {
                    wait = true; // wait until client opens connection
                }
            }

            if(wait) {
                sleep(1000);
                continue;
            }

            // Do client work here
        }
    }
    */

    client clt;
    websocketpp::connection_hdl con_hdl;
    websocketpp::lib::mutex lock;

    std::string uri;

    bool done;
    bool open;
};


#endif
#ifndef MAIN_CPP
#define MAIN_CPP

#include <functional>
#include <mutex>
#include <set>
#include <thread>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::connection_hdl;

class ControlServer {
public:
    ControlServer() {
        srv.init_asio();
        srv.set_open_handler(bind(&ControlServer::on_open,this,std::placeholders::_1));
        srv.set_close_handler(bind(&ControlServer::on_close,this,std::placeholders::_1));
    }

    void on_open(connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx);
        connections.insert(hdl);
    }

    void on_close(connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx);
        connections.erase(hdl);
    }

    void broadcastState() {
        while (1) {
            sleep(1);

	    //TODO: get state
	   std::stringstream ss;
	   ss << "{";

	   ss << "\"" << "position" << "\"" << ":" << "[]" << ",";
	   ss << "\"" << "attitude" << "\"" << ":" << "[]" << ",";
	   ss << "\"" << "depth"    << "\"" << ":" << "[]" << ",";

	   ss << "}";

           std::lock_guard<std::mutex> lock(mtx);
           for (auto it : connections) {
                srv.send(it,ss.str(),websocketpp::frame::opcode::text);
           }
        }
    }

    void run(uint16_t port) {
        srv.listen(port);
        srv.start_accept();
        srv.run();
    }
private:
    typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;

    server srv;
    con_list connections;
    std::mutex mtx;
};

int main() {
    ControlServer server;
    std::thread t(std::bind(&ControlServer::broadcastState,&server));
    server.run(9002);
}

#endif

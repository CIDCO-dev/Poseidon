#pragma once
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <iostream>
#include <stdexcept>

class HttpClient{

public:
	static bool can_reach_server(std::string &host, std::string &port){
		bool status = false;
		try{
			int version = 11;
			
			// The io_context is required for all I/O
			boost::asio::io_context ioService;

			// These objects perform our I/O
			boost::asio::ip::tcp::resolver resolver(ioService);
			boost::beast::tcp_stream stream(ioService);

			// Look up the domain name
			auto const results = resolver.resolve(host, port);

			// Make the connection on the IP address we get from a lookup
			stream.connect(results);

			// Set up an HTTP GET request message
			boost::beast::http::request<boost::beast::http::string_body> req{boost::beast::http::verb::get, "/", version};
			req.set(boost::beast::http::field::host, host);
			req.set(boost::beast::http::field::user_agent, BOOST_BEAST_VERSION_STRING);
			
			// Send the HTTP request to the remote host
			boost::beast::http::write(stream, req);

			// This buffer is used for reading and must be persisted
			boost::beast::flat_buffer buffer;

			// Declare a container to hold the response
			boost::beast::http::response<boost::beast::http::dynamic_body> res;

			// Receive the HTTP response
			boost::beast::http::read(stream, buffer, res);
			boost::beast::error_code ec;

			if(res.result() == boost::beast::http::status::ok){
				stream.socket().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
				if(ec && ec != boost::beast::errc::not_connected){
					throw boost::beast::system_error{ec};
				}
				status = true;
			}
			else{
			 	stream.socket().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
				if(ec && ec != boost::beast::errc::not_connected){
					throw boost::beast::system_error{ec};
				}
				status = false;
//				std::string error = "can_reach_server() response: ";
//				error+=std::to_string(res.result_int());
//				throw std::runtime_error(error);
				std::cerr<<"can_reach_server() response: " << res.result() <<"\n";
			}

		}
		catch(std::exception const& e){
//			std::string error = "Get request error: ";
//			error+=e.what();
//			throw std::runtime_error(error);
			std::cerr<< "Get request error: " << e.what()<<"\n";
		}
		
		return status;
		}
};

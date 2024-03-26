#pragma once
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/asio/ssl/error.hpp>
#include <boost/asio/ssl/stream.hpp>
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


class HttpsClient{

public:
	static bool can_reach_server(std::string &host, std::string &port){
		bool status = false;
		int version = 11;
		try{
			 // The io_context is required for all I/O
			boost::asio::io_context ioc;
			
			// The SSL context is required, and holds certificates
			boost::asio::ssl::context ctx(boost::asio::ssl::context::tlsv13_client);
			ctx.set_options(boost::asio::ssl::context::default_workarounds
								| boost::asio::ssl::context::no_sslv2
								| boost::asio::ssl::context::no_sslv3
								| boost::asio::ssl::context::tlsv12_client);

			// This holds the root certificate used for verification
			//load_root_certificates(ctx);
			ctx.set_default_verify_paths();
			// Verify the remote server's certificate
			ctx.set_verify_mode(boost::asio::ssl::verify_none);
			
			// These objects perform our I/O
			boost::asio::ip::tcp::resolver resolver(ioc);
			boost::beast::ssl_stream<boost::beast::tcp_stream> stream(ioc, ctx);
			
			// Set SNI Hostname (many hosts need this to handshake successfully)
			if(! SSL_set_tlsext_host_name(stream.native_handle(), host.c_str()))
			{
				boost::beast::error_code ec{static_cast<int>(::ERR_get_error()), boost::asio::error::get_ssl_category()};
				throw boost::beast::system_error{ec};
			}
			
			// Look up the domain name
			auto const results = resolver.resolve(host, port);
			// Make the connection on the IP address we get from a lookup
			
			boost::beast::get_lowest_layer(stream).connect(results);
			
			boost::beast::error_code ec1;
			
			// Perform the SSL handshake
			stream.handshake(boost::asio::ssl::stream_base::client, ec1);
			
			if (ec1) {
				std::cerr << "SSL handshake error: " << ec1.message() << std::endl;
				return false;
			}
			
			// Set up an HTTP GET request message
			boost::beast::http::request<boost::beast::http::string_body> req{boost::beast::http::verb::get, "/", version};
			req.set(boost::beast::http::field::host, host);
			req.set(boost::beast::http::field::user_agent, BOOST_BEAST_VERSION_STRING);
			boost::beast::http::write(stream, req);
			boost::beast::flat_buffer buffer;
			boost::beast::http::response<boost::beast::http::dynamic_body> res;
			
			boost::beast::error_code ec2;
			boost::beast::http::read(stream, buffer, res, ec2);
			if(ec2 && ec2 != boost::asio::error::eof && ec2 != boost::asio::ssl::error::stream_errors::stream_truncated){
				ec2 = {};
			}
			
			
			boost::beast::error_code ec;
			if(res.result() == boost::beast::http::status::ok){
				stream.shutdown(ec);
				if(ec && ec != boost::asio::error::eof && ec != boost::asio::ssl::error::stream_errors::stream_truncated){
					throw boost::beast::system_error{ec};
				}
				status = true;
			}
			else{
			 	stream.shutdown(ec);
				if(ec && ec != boost::asio::error::eof && ec != boost::asio::ssl::error::stream_errors::stream_truncated){
					throw boost::beast::system_error{ec};
				}
				status = false;
				ROS_ERROR_STREAM("Can_reach_https_server() response: " << res.result());
			}

		// If we get here then the connection is closed gracefully
		}
		catch(std::exception const& e){
			status = false;
			std::cerr<< "Https Get request error: " << e.what()<<"\n";
		}
		
		return status;
		}
};

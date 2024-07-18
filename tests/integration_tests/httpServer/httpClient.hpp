#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <fstream>
#include <iostream>
#include <string>

class HttpClient
{
  private:
	const int m_kHttpVersion = 10;  // 1.0 or 1.1
	// The io_context is required for all I/O
	boost::asio::io_context m_ioc;
	std::unique_ptr<boost::beast::tcp_stream> m_stream;
	const std::string m_kHost;
	const std::string m_kPort;

  public:
	HttpClient(const std::string& host, const std::string& port) : m_kHost(host), m_kPort(port)
	{
		// These objects perform our I/O
		boost::asio::ip::tcp::resolver resolver(m_ioc);

		m_stream = std::make_unique<boost::beast::tcp_stream>(m_ioc);
		// Look up the domain name

		auto const tcpEndpoint = resolver.resolve(m_kHost, m_kPort);
		// Make the connection on the IP address we get from a lookup
		try
		{
			m_stream->connect(tcpEndpoint);
		}
		catch (const std::exception& e)
		{
			std::cerr << "client exception" << e.what() << '\n';
		}
	}

	~HttpClient()
	{
		// Gracefully close the socket
		boost::beast::error_code ec;
		m_stream->socket().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);

		// not_connected happens sometimes
		// so don't bother reporting it.
		//
		if (ec && ec != boost::beast::errc::not_connected)
		{
			std::cout << boost::beast::system_error{ec}.what() << std::endl;
		}
	};

	boost::beast::http::response<boost::beast::http::dynamic_body> upload(const std::string& filePath,
	                                                                      const std::string& requestURL)
	{
		// open file for reading
		std::ifstream istrm(filePath, std::ios::binary);
		if (!istrm.is_open())
		{
			throw std::runtime_error("failed to open " + filePath);
		}
		std::stringstream fileBuffer;
		fileBuffer << istrm.rdbuf();
		std::string body = fileBuffer.str();

		// Set up an HTTP GET request message
		boost::beast::http::request<boost::beast::http::string_body> req{
		    boost::beast::http::verb::post, requestURL, m_kHttpVersion};
		req.set(boost::beast::http::field::host, m_kHost);
		req.set(boost::beast::http::field::user_agent, BOOST_BEAST_VERSION_STRING);
		req.set(boost::beast::http::field::content_type, "text/plain");
		req.body() = body;
		req.prepare_payload();  // needed to set the correct size of the message

		// Send the HTTP request to the remote host
		boost::beast::http::write(*m_stream, req);

		// This buffer is used for reading and must be persisted
		boost::beast::flat_buffer buffer;

		// Declare a container to hold the response
		boost::beast::http::response<boost::beast::http::dynamic_body> res;

		// Receive the HTTP response
		boost::beast::http::read(*m_stream, buffer, res);
		return res;
	}

	boost::beast::http::response<boost::beast::http::dynamic_body> download(const std::string& requestURL,
	                                                                        const boost::beast::http::verb& httpVerb)
	{
		// Set up an HTTP GET request message
		boost::beast::http::request<boost::beast::http::string_body> req{httpVerb, requestURL, m_kHttpVersion};
		req.set(boost::beast::http::field::host, m_kHost);
		req.set(boost::beast::http::field::user_agent, BOOST_BEAST_VERSION_STRING);
		req.set(boost::beast::http::field::content_type, "text/plain");
		req.body() = "";
		req.prepare_payload();  // needed to set the correct size of the message

		// Send the HTTP request to the remote host
		boost::beast::http::write(*m_stream, req);

		// This buffer is used for reading and must be persisted
		boost::beast::flat_buffer buffer;

		// Declare a container to hold the response
		boost::beast::http::response<boost::beast::http::dynamic_body> res;

		// Receive the HTTP response
		boost::beast::http::read(*m_stream, buffer, res);

		return res;
	}
};

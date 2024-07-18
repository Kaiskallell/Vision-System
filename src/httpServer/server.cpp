#include "server.hpp"

#include <memory>
#include <thread>

namespace net = boost::asio;       // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;  // from <boost/asio/ip/tcp.hpp>

#include <openssl/sha.h>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/config.hpp>
#include <cstdio>
#include <cstdlib>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

namespace beast = boost::beast;  // from <boost/beast.hpp>
namespace http = beast::http;    // from <boost/beast/http.hpp>
namespace fs = std::experimental::filesystem;

enum class FileTransferMode
{
	OnnxUpload = 0,
	EngineUpload = 1,
	NetworkConfigUpload = 2,
	Sha1SumDownload = 3,
	LogDownload = 4,
	GetAllNetworkPaths = 5,
	GetAllNetworkConfigPaths = 6,
	FileDeletion = 7,
	Unknown
};

// Return a reasonable mime type based on the extension of a file.
beast::string_view mimeType(beast::string_view path)
{
	using beast::iequals;
	auto const ext = [&path] {
		auto const pos = path.rfind(".");
		if (pos == beast::string_view::npos)
		{
			return beast::string_view{};
		}
		return path.substr(pos);
	}();
	if (iequals(ext, ".htm"))
	{
		return "text/html";
	}
	if (iequals(ext, ".html"))
	{
		return "text/html";
	}
	if (iequals(ext, ".php"))
	{
		return "text/html";
	}
	if (iequals(ext, ".txt"))
	{
		return "text/plain";
	}
	if (iequals(ext, ".log"))
	{
		return "text/plain";
	}
	if (iequals(ext, ".json"))
	{
		// return "application/json";
		return "application/text";
	}
	return "application/text";
}

template <class Body, class Allocator>
http::response<http::string_body> createResponse(const http::request<Body, http::basic_fields<Allocator>>& req,
                                                 const beast::string_view why,
                                                 const http::status status)
{
	http::response<http::string_body> res{status, req.version()};
	res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
	res.set(http::field::content_type, "text/html");
	res.keep_alive(req.keep_alive());
	res.body() = std::string(req.target()) + ": " + std::string(why);
	res.prepare_payload();
	return res;
}

template <class Body, class Allocator>
http::response<http::file_body> fileDownload(http::request<Body, http::basic_fields<Allocator>>& req,
                                             const fs::path& filePath)
{
	std::string reqFilePath = filePath.string();

	// Attempt to open the file
	beast::error_code ec;
	http::file_body::value_type body;
	body.open(reqFilePath.c_str(), beast::file_mode::scan, ec);
	// Handle the case where the file doesn't exist
	if (ec == beast::errc::no_such_file_or_directory)
	{
		std::cout << "no such file or directory" << std::endl;
		body = {};
	}

	uint64_t bodySize = body.size();  // needed because of std::move
	http::response<http::file_body> res{
	    std::piecewise_construct, std::make_tuple(std::move(body)), std::make_tuple(http::status::ok, req.version())};
	res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
	res.set(http::field::content_type, mimeType(reqFilePath));
	res.content_length(bodySize);
	res.keep_alive(req.keep_alive());
	return res;
}

// works only for json url
// e.g. /format/1/networksConfig.json
int getFormatIdFromURL(const std::string& kRequestURL)
{
	std::stringstream targetStream(kRequestURL);
	// split the url string into multiple substrings and remove the slash letter
	std::vector<std::string> seglist;
	std::string segment;
	while (std::getline(targetStream, segment, '/'))
	{
		seglist.push_back(segment);
	}

	int formatId = -1;
	try
	{
		constexpr int kFormatPosition = 2;  // works only for json url
		formatId = std::stoi(seglist.at(kFormatPosition));
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		formatId = -2;
	}
	return formatId;
}

// this function processes the incoming URLs and determines which action should
// be executed
template <class Body, class Allocator>
FileTransferMode whichFileTransferMode(const http::request<Body, http::basic_fields<Allocator>>& req)
{
	// Build the path to the requested file
	const std::string kRequestURL = req.target().to_string();
	std::cout << "kRequestURL is : " << kRequestURL << std::endl;

	FileTransferMode fileTransferMode = FileTransferMode::Unknown;
	fs::path filePath = kRequestURL;
	auto it = filePath.begin();
	it++;  // fist iterator was "/"" now it is e.g. "resources"
	if (filePath.extension() == ".onnx" && *it == "resources" && *std::next(it, 1) == "networks")
	{
		if (req.method() == http::verb::post)
		{
			fileTransferMode = FileTransferMode::OnnxUpload;
		}
		if (req.method() == http::verb::delete_)
		{
			fileTransferMode = FileTransferMode::FileDeletion;
		}
	}
	else if (filePath.extension() == ".engine" && *it == "resources" && *std::next(it, 1) == "networks")
	{
		if (req.method() == http::verb::post)
		{
			fileTransferMode = FileTransferMode::EngineUpload;
		}
		if (req.method() == http::verb::delete_)
		{
			fileTransferMode = FileTransferMode::FileDeletion;
		}
	}
	else if (filePath.extension() == ".json" && *it == "format" && getFormatIdFromURL(kRequestURL) >= 0)
	{
		if (req.method() == http::verb::post)
		{
			fileTransferMode = FileTransferMode::NetworkConfigUpload;
		}
		if (req.method() == http::verb::delete_)
		{
			fileTransferMode = FileTransferMode::FileDeletion;
		}
	}
	else if (filePath.extension() == ".sha1")
	{
		fileTransferMode = FileTransferMode::Sha1SumDownload;
	}
	else if (*it == "getLogs")
	{
		fileTransferMode = FileTransferMode::LogDownload;
	}
	else if (*it == "getAllNetworkPaths")
	{
		fileTransferMode = FileTransferMode::GetAllNetworkPaths;
	}
	else if (*it == "getAllNetworkConfigPaths")
	{
		fileTransferMode = FileTransferMode::GetAllNetworkConfigPaths;
	}

	return fileTransferMode;
}

// save the body of the request to disk in the right folder
template <class Body, class Allocator>
http::status onnxOrConfigFileUpload(const http::request<Body, http::basic_fields<Allocator>>& req,
                                    beast::string_view doc_root)
{
	if (req.body().empty())
	{
		std::cerr << "File which should be uploaded has size == 0!" << std::endl;
		return http::status::not_acceptable;
	}
	std::string docRoot(doc_root);
	fs::path filePath = docRoot + req.target().to_string();
	if (fs::exists(filePath))
	{
		std::cerr << "File exitst already. You need to delete it first before uploading a new file!" << std::endl;
		return http::status::conflict;
	}
	if (!fs::exists(filePath))
	{
		std::cout << "creating dir in = " << filePath.parent_path() << std::endl;
		fs::create_directories(filePath.parent_path());
	}
	// check if there is enough space on disk
	std::error_code ec;
	fs::space_info spaceInfo = fs::space(filePath.parent_path(), ec);
	if (spaceInfo.available < req.body().size())
	{
		return http::status::insufficient_storage;
	}

	std::ofstream file;
	file.open(filePath);
	file << req.body();
	file.close();

	// create sha1 file for the network
	std::string str = req.body();
	unsigned char hash[SHA_DIGEST_LENGTH];
	SHA1((unsigned char*)str.c_str(), str.size(), hash);
	// create file with sha1sum in it
	fs::path fileShaPath = docRoot + req.target().to_string();
	fileShaPath.replace_extension(".sha1");
	std::cout << "sha1 file is = " << fileShaPath << std::endl;

	FILE* fp;
	fp = fopen(fileShaPath.c_str(), "w");
	for (int i = 0; i < SHA_DIGEST_LENGTH; i++)
	{
		printf("%02x", hash[i]);
		fprintf(fp, "%02x", hash[i]);
	}
	fclose(fp);
	printf("\n");
	return http::status::ok;
}

fs::path getAllNetworkPathsFile()
{
	fs::path root = "../../resources/networks";
	fs::path outFilePath = "../../resources/networks/allNetworkPaths.txt";
	std::ofstream outfile(outFilePath);

	std::vector<fs::path> paths;
	if (fs::exists(root) && fs::is_directory(root))
	{
		for (auto const& entry : fs::recursive_directory_iterator(root))
		{
			if (fs::is_regular_file(entry) && entry.path().extension() == ".onnx")
			{
				outfile << entry.path().string() << std::endl;
			}
			if (fs::is_regular_file(entry) && entry.path().extension() == ".engine")
			{
				outfile << entry.path().string() << std::endl;
			}
		}
	}
	outfile.close();

	return outFilePath;
}

fs::path getAllNetworkConfigPathsFile()
{
	fs::path root = "../../format";
	fs::path outFilePath = "../../format/allNetworkConfigPaths.txt";
	std::ofstream outfile(outFilePath);

	std::vector<fs::path> paths;
	if (fs::exists(root) && fs::is_directory(root))
	{
		for (auto const& entry : fs::recursive_directory_iterator(root))
		{
			if (fs::is_regular_file(entry) && entry.path().extension() == ".json"
			    && entry.path().filename() != "format.json")
			{
				outfile << entry.path().string() << std::endl;
			}
		}
	}
	outfile.close();

	return outFilePath;
}

// This function produces an HTTP response for the given
// request. The type of the response object depends on the
// contents of the request, so the interface requires the
// caller to pass a generic lambda for receiving the response.
template <class Body, class Allocator, class Send>
void handleRequest(beast::string_view doc_root, http::request<Body, http::basic_fields<Allocator>>&& req, Send&& send)
{
	// Make sure we can handle the method
	if (req.method() != http::verb::post && req.method() != http::verb::get && req.method() != http::verb::delete_)
	{
		return send(createResponse(req, "Unknown HTTP-method", http::status::bad_request));
	}
	// Request path must be absolute and not contain "..".
	if (req.target().empty() || req.target()[0] != '/' || req.target().find("..") != beast::string_view::npos)
	{
		return send(createResponse(req, "Unknown HTTP-method", http::status::bad_request));
	}

	FileTransferMode fileTransferMode = whichFileTransferMode(req);
	std::cout << "FileTransferMode = " << static_cast<int>(fileTransferMode) << std::endl;
	if (fileTransferMode == FileTransferMode::Sha1SumDownload)
	{
		// Attempt to open the file
		std::string docRoot{doc_root};
		fs::path fileShaPath = docRoot + req.target().to_string();
		std::cout << "checksum download: " << fileShaPath << std::endl;

		if (!fs::exists(fileShaPath))
		{
			std::cout << "file not found" << std::endl;
			return send(createResponse(req, "file not found", http::status::not_found));
		}
		return send(fileDownload(req, fileShaPath));
	}
	if (fileTransferMode == FileTransferMode::OnnxUpload)
	{
		std::cout << "onnx upload" << std::endl;
		http::status errStatus = onnxOrConfigFileUpload(req, doc_root);
		return send(createResponse(req, "Upload done", errStatus));
	}
	if (fileTransferMode == FileTransferMode::EngineUpload)
	{
		std::cout << "engine upload" << std::endl;
		http::status errStatus = onnxOrConfigFileUpload(req, doc_root);
		return send(createResponse(req, "Upload done", errStatus));
	}
	if (fileTransferMode == FileTransferMode::NetworkConfigUpload)
	{
		std::cout << "network config upload" << std::endl;
		http::status errStatus = onnxOrConfigFileUpload(req, doc_root);
		return send(createResponse(req, "Upload done", errStatus));
	}
	if (fileTransferMode == FileTransferMode::LogDownload)
	{
		std::cout << "log download" << std::endl;
		// Attempt to open the file
		fs::path logsPath = std::string(doc_root) + "/build/bin/vsLogs.log";
		if (!fs::exists(logsPath))
		{
			return send(createResponse(req, "file not found", http::status::not_found));
		}
		return send(fileDownload(req, logsPath));
	}
	if (fileTransferMode == FileTransferMode::GetAllNetworkPaths)
	{
		std::cout << "get all networkPaths download" << std::endl;

		fs::path allNetworkPathFile = getAllNetworkPathsFile();
		if (!fs::exists(allNetworkPathFile))
		{
			return send(createResponse(req, "file not found", http::status::not_found));
		}

		return send(fileDownload(req, allNetworkPathFile));
	}
	if (fileTransferMode == FileTransferMode::GetAllNetworkConfigPaths)
	{
		std::cout << "get all networkConfigPaths download" << std::endl;

		fs::path allNetworkConfigPathFile = getAllNetworkConfigPathsFile();
		if (!fs::exists(allNetworkConfigPathFile))
		{
			return send(createResponse(req, "file not found", http::status::not_found));
		}

		return send(fileDownload(req, allNetworkConfigPathFile));
	}
	if (fileTransferMode == FileTransferMode::FileDeletion)
	{
		std::cout << "deleting file" << std::endl;
		std::string docRoot{doc_root};
		fs::path filePath = docRoot + req.target().to_string();
		if (!fs::exists(filePath) || fs::is_directory(filePath))
		{
			return send(createResponse(req, "file not found or directory", http::status::not_found));
		}

		try
		{
			fs::remove(filePath);
			filePath = filePath.replace_extension(".sha1");
			fs::remove(filePath);
			filePath = filePath.replace_extension(".engine");
			fs::remove(filePath);
			if (fs::is_empty(filePath.parent_path()))
			{
				fs::remove(filePath.parent_path());
			}
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what() << '\n';
			return send(createResponse(req, "delete task exception occured", http::status::unknown));
		}

		return send(createResponse(req, "delete task done", http::status::ok));
	}
	// Handle an unknown fileTransferMode
	return send(
	    createResponse(req, "No valid fileTransferMode. Check your request url!", http::status::internal_server_error));
}

// Report a failure
void fail(beast::error_code ec, char const* what) { std::cerr << what << ": " << ec.message() << "\n"; }

// This is the C++11 equivalent of a generic lambda.
// The function object is used to send an HTTP message.
template <class Stream>
struct SendLambda
{
	Stream& m_stream;
	bool& m_close;
	beast::error_code& m_ec;

	explicit SendLambda(Stream& stream, bool& close, beast::error_code& ec) : m_stream(stream), m_close(close), m_ec(ec)
	{
	}

	template <bool IsRequest, class Body, class Fields>
	void operator()(http::message<IsRequest, Body, Fields>&& msg) const
	{
		// Determine if we should close the connection after
		m_close = msg.need_eof();

		// We need the serializer here because the serializer requires
		// a non-const file_body, and the message oriented version of
		// http::write only works with const messages.
		http::serializer<IsRequest, Body, Fields> sr{msg};
		http::write(m_stream, sr, m_ec);
	}
};

// Handles an HTTP server connection
void doSession(tcp::socket& socket, std::shared_ptr<std::string const> const& doc_root)
{
	bool close = false;
	beast::error_code ec;

	// This buffer is required to persist across reads
	beast::flat_buffer buffer;

	// This lambda is used to send messages
	SendLambda<tcp::socket> lambda{socket, close, ec};

	for (;;)
	{
		// Read a request
		http::request<http::string_body> req;

		http::request_parser<http::string_body> parser;
		// Allow for an unlimited body size
		parser.body_limit((std::numeric_limits<std::uint64_t>::max)());
		http::read(socket, buffer, parser, ec);
		// http::read(socket, buffer, req, ec);
		if (ec == http::error::end_of_stream)
		{
			break;
		}
		if (ec)
		{
			return fail(ec, "read");
		}

		req = parser.get();

		// Send the response
		handleRequest(*doc_root, std::move(req), lambda);
		if (ec)
		{
			return fail(ec, "write");
		}
		if (close)
		{
			// This means we should close the connection, usually because
			// the response indicated the "Connection: close" semantic.
			break;
		}
	}

	// Send a TCP shutdown
	socket.shutdown(tcp::socket::shutdown_send, ec);

	// At this point the connection is closed gracefully
}

HttpServer::HttpServer(const char* address, const unsigned short port, const std::string& docRoot)
    : m_address(net::ip::make_address(address)), m_port(port), m_docRoot(std::make_shared<std::string>(docRoot))
{
}

void HttpServer::run()
{
	// The io_context is required for all I/O
	boost::asio::io_context ioc{1};
	boost::asio::ip::tcp::acceptor acceptor{ioc, {m_address, m_port}};

	for (;;)
	{
		// This will receive the new connection
		tcp::socket socket{ioc};

		// Block until we get a connection
		acceptor.accept(socket);

		// Launch the session, transferring ownership of the socket
		std::thread{std::bind(&doSession, std::move(socket), m_docRoot)}.detach();
	}
}

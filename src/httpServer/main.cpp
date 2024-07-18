#include <iostream>
#include <opencv2/opencv.hpp>

#include "server.hpp"

void parseCommandLineInput(int argc, char* argv[], std::string& address, size_t& port)
{
	// overwriting the default values with the command line arguments/values
	std::string keys = "{address   |                 | camera configuration path}"  // default value ""
	                   "{port   |      | networks configuration path}"              // default value ""
	                   "{help   |      | show help message}";

	cv::CommandLineParser parser(argc, argv, keys);
	if (parser.has("help"))
	{
		parser.printMessage();
		exit(-1);
	}
	if (parser.has("address"))
	{
		address = parser.get<std::string>("address");
	}
	if (parser.has("port"))
	{
		port = parser.get<size_t>("port");
	}
}

int main(int argc, char* argv[])
{
	try
	{
		size_t port = 55000;
		std::string address = "192.168.52.120";
		parseCommandLineInput(argc, argv, address, port);

		std::string docRoot = "../..";  // assuming that executable is in VisionSystem/build/bin/

		HttpServer server(address.c_str(), port, docRoot);

		server.run();
	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}

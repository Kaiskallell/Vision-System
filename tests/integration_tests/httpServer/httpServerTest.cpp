/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "httpServerTest.hpp"

#include <unistd.h>

#include <experimental/filesystem>

#include "httpClient.hpp"
#include "projectPaths.h"

namespace bp = boost::process;

HttpServerTest::HttpServerTest()
{
	// start httpServer Process
	std::experimental::filesystem::path httpServerExecPath = utils::getProjectRootDir() / "build/bin/vsHttpServer";
	if (!std::experimental::filesystem::exists(httpServerExecPath))
	{
		std::cerr << httpServerExecPath << " does not exist! Aborting ..." << std::endl;
		std::abort();
	}
	std::string ip4Address = "127.0.0.1";
	size_t port = 55000;
	std::vector<std::string> cmdArgs = {"-address=" + ip4Address, "-port=" + std::to_string(port)};
	m_httpServerProcess = bp::child(httpServerExecPath.string(), cmdArgs, m_httpServerGroup);

	sleep(1);
}

HttpServerTest::~HttpServerTest()
{
	try
	{
		m_httpServerGroup.terminate();
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
}

TEST_F(HttpServerTest, getLogsFromServer)
{
	std::string host = "127.0.0.1";
	std::string port = "55000";
	HttpClient client(host, port);
	std::string requestUrl = "/getLogs";
	boost::beast::http::response<boost::beast::http::dynamic_body> res =
	    client.download(requestUrl, boost::beast::http::verb::post);
	EXPECT_NE(res.body().size(), 0);
}

TEST_F(HttpServerTest, uploadOnnx)
{
	std::string host = "127.0.0.1";
	std::string port = "55000";
	HttpClient client(host, port);
	std::string requestUrl =
	    "/resources/networks/DrOetker/dummy.onnx";  // this is where it should be saved after uploading
	std::string filePath = utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/httpServer/dummy.onnx";
	boost::beast::http::response<boost::beast::http::dynamic_body> res = client.upload(filePath, requestUrl);
	EXPECT_NE(res.body().size(), 0);
}

TEST_F(HttpServerTest, uploadEngine)
{
	std::string host = "127.0.0.1";
	std::string port = "55000";
	HttpClient client(host, port);
	std::string requestUrl =
	    "/resources/networks/DrOetker/dummy.engine";  // this is where it should be saved after uploading
	std::string filePath = utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/httpServer/dummy.engine";
	boost::beast::http::response<boost::beast::http::dynamic_body> res = client.upload(filePath, requestUrl);
	EXPECT_NE(res.body().size(), 0);
}

TEST_F(HttpServerTest, getHashFromServer)
{
	std::string host = "127.0.0.1";
	std::string port = "55000";
	HttpClient client(host, port);
	// std::string requestUrl = "/resources/networks/DrOetker/dummy.sha1";
	std::string requestUrl = "/resources/networks/DrOetker/dummy.sha1";
	boost::beast::http::response<boost::beast::http::dynamic_body> res =
	    client.download(requestUrl, boost::beast::http::verb::post);
	std::string hashsum = boost::beast::buffers_to_string(res.body().data());
	std::string expectedHashSum = "532ac76cfe8c6f7565a11fccdb70567126f73e80";
	std::cout << "read sha sum :" << hashsum << std::endl;
	std::cout << "expectedHashSum :" << expectedHashSum << std::endl;
	EXPECT_NE(res.body().size(), 0);
	EXPECT_EQ(hashsum, expectedHashSum);
}

TEST_F(HttpServerTest, uploadConfigJson)
{
	std::string host = "127.0.0.1";
	std::string port = "55000";
	HttpClient client(host, port);
	std::string requestUrl = "/format/0/networksConfig.json";  // this is where it should be saved after uploading
	std::string filePath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/httpServer/networksConfig.json";
	boost::beast::http::response<boost::beast::http::dynamic_body> res = client.upload(filePath, requestUrl);
	EXPECT_NE(res.body().size(), 0);
}

TEST_F(HttpServerTest, invalidFileDownload)
{
	std::string host = "127.0.0.1";
	std::string port = "55000";
	HttpClient client(host, port);
	std::string requestUrl = "/irgendwasUnsinniges";
	boost::beast::http::response<boost::beast::http::dynamic_body> ret =
	    client.download(requestUrl, boost::beast::http::verb::post);
	boost::beast::http::status httpStatusCode = ret.result();
	EXPECT_EQ(httpStatusCode, boost::beast::http::status::internal_server_error);
}

TEST_F(HttpServerTest, invalidFileUpload)
{
	std::string host = "127.0.0.1";
	std::string port = "55000";
	HttpClient client(host, port);
	std::string requestUrl = "/wrong/Path/2354/networksConfig.json";
	std::string filePath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/httpServer/dummy.wrongFileExtension";
	boost::beast::http::response<boost::beast::http::dynamic_body> ret = client.upload(filePath, requestUrl);
	boost::beast::http::status httpStatusCode = ret.result();
	EXPECT_EQ(httpStatusCode, boost::beast::http::status::internal_server_error);
}

TEST_F(HttpServerTest, getAllNetworkPaths)
{
	std::string host = "127.0.0.1";
	std::string port = "55000";
	HttpClient client(host, port);
	std::string requestUrl = "/getAllNetworkPaths";
	boost::beast::http::response<boost::beast::http::dynamic_body> res =
	    client.download(requestUrl, boost::beast::http::verb::post);
	EXPECT_NE(res.body().size(), 0);
}

TEST_F(HttpServerTest, deleteNetworkFiles)
{
	std::string host = "127.0.0.1";
	std::string port = "55000";
	HttpClient client(host, port);
	std::string requestUrl =
	    "/resources/networks/DrOetker/dummy.onnx";  // based on upload test, needs to be executed after upload test
	boost::beast::http::response<boost::beast::http::dynamic_body> res =
	    client.download(requestUrl, boost::beast::http::verb::delete_);

	// both sha1 and onnx file needs to be deleted
	bool ret = fs::exists("../.." + requestUrl);
	EXPECT_FALSE(ret);
	std::string shaFilePath = "/resources/networks/DrOetker/dummy.onnx";
	ret = fs::exists("../.." + shaFilePath);
	EXPECT_FALSE(ret);
	std::string engineFilePath = "/resources/networks/DrOetker/dummy.engine";
	ret = fs::exists("../.." + engineFilePath);
	EXPECT_FALSE(ret);
}

TEST_F(HttpServerTest, deleteConfigFiles)
{
	std::string host = "127.0.0.1";
	std::string port = "55000";
	HttpClient client(host, port);
	std::string requestUrl =
	    "/format/0/networksConfig.json";  // based on upload test, needs to be executed after upload test
	boost::beast::http::response<boost::beast::http::dynamic_body> res =
	    client.download(requestUrl, boost::beast::http::verb::delete_);

	// both sha1 and onnx file needs to be deleted
	bool ret = fs::exists("../.." + requestUrl);
	EXPECT_FALSE(ret);
	std::string configFilePath = "/format/0/networksConfig.json";
	ret = fs::exists("../.." + configFilePath);
	EXPECT_FALSE(ret);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

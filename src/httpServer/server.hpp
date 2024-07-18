#include <boost/asio/ip/tcp.hpp>

class HttpServer {
public:
  HttpServer(const char *address, const unsigned short port,
             const std::string &docRoot);
  ~HttpServer(){};
  void run();

private:
  boost::asio::ip::address m_address;
  unsigned short m_port = 0;
  std::shared_ptr<std::string> m_docRoot = nullptr;
};
#pragma once

#include <unordered_map>

#include <boost/asio.hpp>

class StatusServer
{
public:
    StatusServer(boost::asio::io_context &io_context);

    void write_message(const std::string &name, const std::string &text);

private:
    void listen_for_client();

    boost::asio::io_context &io_context_;
    boost::asio::ip::tcp::acceptor acceptor_;
    std::unordered_map<int, boost::asio::ip::tcp::socket> clients_;
};

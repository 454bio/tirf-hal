#include <iostream>

#include <boost/asio.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "status_server.hpp"

using boost::asio::ip::tcp;

constexpr static unsigned short STATUS_PORT = 45403;

StatusServer::StatusServer(boost::asio::io_context &io_context)
    : io_context_(io_context)
    , acceptor_(io_context_, tcp::endpoint(tcp::v4(), STATUS_PORT))
    , clients_()
{
    listen_for_client();
}

void StatusServer::listen_for_client()
{
    acceptor_.async_accept([this](boost::system::error_code ec, tcp::socket socket)
    {
        if (!ec)
        {
            std::cerr << "New connection from GUI at " << socket.remote_endpoint().address() << std::endl;
            clients_.emplace(socket.native_handle(), std::move(socket));
        }

        // Start listening for the next one. This way, we won't need a separate thread just to wait for new clients.
        listen_for_client();
    });
}

void StatusServer::write_message(const std::string &name, const std::string &text)
{
    std::cerr << "Status (" << clients_.size() << " clients): " << name << " " << text << std::endl;

    boost::property_tree::ptree request;
    request.add<std::string>("name", name);
    request.add<std::string>("text", text);

    // Format the request...
    std::stringstream request_stream;
    boost::property_tree::write_json(request_stream, request, false);
    std::string request_string = request_stream.str();

    for (auto &peer_socket : clients_)
    {
        try
        {
            // ... and send it.
            boost::asio::write(peer_socket.second, boost::asio::buffer(request_string.c_str(), request_string.size()));
        }
        catch (const std::exception &e)
        {
            std::cerr << "Lost connection with GUI: " << e.what() << std::endl;
            clients_.erase(peer_socket.first);
        }
        catch (...)
        {
            std::cerr << "Lost connection with GUI: Unknown error" << std::endl;
            clients_.erase(peer_socket.first);
        }
    }
}

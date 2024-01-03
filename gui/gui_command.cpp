#include "gui_command.hpp"

using boost::asio::ip::tcp;

constexpr static unsigned short MESSAGE_PORT = 45402;
constexpr static size_t MAX_RESPONSE_SIZE = 1024;

std::shared_ptr<tcp::socket> send_command_message(boost::asio::io_context &io_context, const boost::property_tree::ptree &request, const boost::asio::ip::address &peer_address)
{
    // Format the request...
    std::stringstream request_stream;
    boost::property_tree::write_json(request_stream, request);
    std::string request_string = request_stream.str();

    // ... and send it.
    // If std::functions accepted movable but not copyable types, the shared_ptr would not be necessary.
    // The shared_ptr can be replaced with unique_ptr (or removed entirely) once we have move_only_function from C++23.
    std::shared_ptr<tcp::socket> socket = std::make_shared<tcp::socket>(io_context);
    socket->connect(tcp::endpoint(peer_address, MESSAGE_PORT));
    boost::asio::write(*socket, boost::asio::buffer(request_string.c_str(), request_string.size()));

    return socket;
}

boost::property_tree::ptree wait_for_command_response(std::shared_ptr<tcp::socket> &&socket)
{
    // Read the response...
    // The entire response must be read at once.
    boost::asio::streambuf response_stream;
    boost::asio::streambuf::mutable_buffers_type response_buffer = response_stream.prepare(MAX_RESPONSE_SIZE);
    size_t bytes_read = socket->receive(response_buffer);
    response_stream.commit(bytes_read);

    // ... parse it...
    std::istream response_istream(&response_stream);
    boost::property_tree::ptree response;
    boost::property_tree::read_json(response_istream, response);

    // ... and throw on failure.
    if (!response.get<bool>("success"))
    {
        throw std::logic_error("User prompt failed: " + response.get<std::string>("error"));
    }

    return response;
}

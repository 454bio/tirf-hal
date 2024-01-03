#pragma once

#include <boost/asio.hpp>
#include <boost/property_tree/json_parser.hpp>

// Send a command message to the GUI.
// The caller is responsible for later calling `wait_for_command_response` below with the returned socket.
std::shared_ptr<boost::asio::ip::tcp::socket> send_command_message(boost::asio::io_context &io_context, const boost::property_tree::ptree &request, const boost::asio::ip::address &peer_address);

// Retrieve the command response from the GUI, blocking until we receive one.
boost::property_tree::ptree wait_for_command_response(std::shared_ptr<boost::asio::ip::tcp::socket> &&socket);

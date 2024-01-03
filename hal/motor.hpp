#pragma once

#include <memory>

#include <boost/property_tree/json_parser.hpp>

class StatusServer;

class IMotor
{
public:
    // Do any post-constructor work needed to put the motor into a consistent state, like moving it to a known position.
    // Users are expected to call this after construction but before the first `move_async`.
    virtual void initialize() {}

    virtual void move_async(int position, const std::string &position_name) = 0;
    virtual void wait_for_move() = 0;
    virtual void nudge_base_position(int position_delta) = 0;
};

std::unique_ptr<IMotor> motor_factory(const std::string &name, StatusServer &status_server, const boost::property_tree::ptree &parsed);

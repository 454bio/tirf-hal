#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "icamera.hpp"

class PylablibCamera : public ICamera
{
public:
    PylablibCamera(boost::asio::io_context &io_context, const boost::property_tree::ptree &camera_config);

    virtual bool can_override_exposure() const;
    virtual std::optional<ICameraOptions> options() const;

    virtual void start_capture(const boost::asio::ip::address &peer_address, const std::optional<size_t> &exposure_time_ms_override);
    virtual void stop_capture(const boost::asio::ip::address &peer_address, const std::optional<std::string> &path);

    virtual void capture_image(ShutterCallback shutter_callback, const boost::asio::ip::address &peer_address, const std::optional<std::string> &path);

private:
    static void shutter_callback_impl(int user_gpio, int level, uint32_t tick, void *user_data);

    constexpr static int SCHEMA_VERSION = 1;

    boost::asio::io_context &io_context_;

    unsigned int rolling_shutter_wait_ms_;
    // Output pin we can use to trigger a capture.
    unsigned int trigger_pin_;
    // Input pin that indicates when the capture has actually started.
    unsigned int shutter_pin_;

    ShutterCallback shutter_callback_;
    std::mutex shutter_callback_mutex_; // std::function assignment is not atomic
    std::promise<void> shutter_callback_done_;
};

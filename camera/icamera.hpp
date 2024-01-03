#pragma once

#include <chrono>
#include <future>
#include <optional>
#include <string>

#include <boost/asio/ip/address.hpp>
#include <boost/property_tree/json_parser.hpp>

struct ICameraOptions
{
    size_t shutter_time_ms;
};

class ICamera
{
public:
    // Placeholder for any work needed to set up the camera that *must* happen outside of the constructor.
    // Users must call this exactly once after constructing the camera and before capturing images.
    virtual void initialize(const boost::property_tree::ptree &camera_config) {}

    // Whether the shutter speed can be overriden at capture time.
    virtual bool can_override_exposure() const = 0;

    virtual std::optional<ICameraOptions> options() const
    {
        return std::nullopt;
    }

    virtual void start_capture(const boost::asio::ip::address &peer_address, const std::optional<size_t> &exposure_time_ms_override = std::nullopt) {}
    virtual void stop_capture(const boost::asio::ip::address &peer_address, const std::optional<std::string> &path) {}

    typedef std::function<void(const std::chrono::steady_clock::time_point &)> ShutterCallback;

    // Capture and optionally save an image, ensuring that `shutter_callback` is run while the shutter is open.
    // If provided, the captured image will be saved to `path`.
    virtual void capture_image(ShutterCallback shutter_callback, const boost::asio::ip::address &peer_address, const std::optional<std::string> &path) = 0;
};

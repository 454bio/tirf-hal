#pragma once

#include <vector>

#include <boost/asio/ip/address.hpp>
#include <boost/property_tree/ptree_fwd.hpp>

class GpioHal;
enum class GpioLed;
enum class LensFilter;
class ICamera;
class LibcameraApp;

class Cleaving
{
public:
    Cleaving(const boost::property_tree::ptree &parsed);
    void run(GpioHal *hal, ICamera *camera, const std::optional<std::string> &output_dir, const boost::asio::ip::address &peer_address);

private:
    constexpr static int SCHEMA_VERSION = 0;

    unsigned int capture_period_ms_;
    LensFilter filter_;
    unsigned int cleaving_duration_ms_;
    GpioLed cleaving_led_;
    unsigned short cleaving_pwm_;
    std::string filename_format_;
};

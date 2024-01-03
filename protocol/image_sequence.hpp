#pragma once

#include <optional>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/property_tree/ptree_fwd.hpp>

class GpioHal;
enum class GpioLed;
enum class LensFilter;
class ICamera;

struct FlashParameters
{
    FlashParameters(const boost::property_tree::ptree &parsed);

    GpioLed led;
    unsigned int duration_ms;
    unsigned short pwm;
};

struct ImageParameters
{
    ImageParameters(const boost::property_tree::ptree &parsed);

    std::string label;
    std::vector<FlashParameters> flashes;
    LensFilter filter;
    std::string filename_format;

    unsigned int longest_flash_duration_ms;
};

class ImageSequence
{
public:
    ImageSequence(const boost::property_tree::ptree &parsed, bool live_preview = false);
    static ImageSequence from_file(const std::string &filename);

    void run(GpioHal *hal, ICamera *camera, const std::optional<size_t> &exposure_time_ms_override, const std::optional<std::string> &output_dir, std::function<bool()> &interrupt_requested, const boost::asio::ip::address &peer_address) const;

private:
    constexpr static int SCHEMA_VERSION = 0;

    std::string label_;
    std::vector<ImageParameters> images_;
    bool live_preview_;
};

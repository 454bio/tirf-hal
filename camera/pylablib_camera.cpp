#include <boost/property_tree/json_parser.hpp>
#include <pigpio.h>

#include "gui/gui_command.hpp"
#include "utils/c_utils.hpp"

#include "pylablib_camera.hpp"

constexpr static size_t DEFAULT_SHUTTER_TIME_MS = 5000;

PylablibCamera::PylablibCamera(boost::asio::io_context &io_context, const boost::property_tree::ptree &camera_config)
    : io_context_(io_context)
{
    assert(camera_config.get<int>("schema_version") == SCHEMA_VERSION);
    trigger_pin_ = camera_config.get<unsigned int>("trigger_pin");
    shutter_pin_ = camera_config.get<unsigned int>("shutter_pin");
    rolling_shutter_wait_ms_ = camera_config.get<unsigned int>("rolling_shutter_wait_ms");

    // Set up trigger pin using pigpio.
    // Assumes that GPIO was already initialized using `gpioInitialise()`.
    CHECK_RC(gpioSetMode(trigger_pin_, PI_OUTPUT), "Could not set up camera shutter");

    // Assume GPIO is active low.
    CHECK_RC(gpioWrite(trigger_pin_, true), "Could not reset camera trigger");

    CHECK_RC(gpioSetMode(shutter_pin_, PI_INPUT), "Could not set up shutter input");
    CHECK_RC(gpioSetISRFuncEx(shutter_pin_, FALLING_EDGE, 0 /*timeout*/, PylablibCamera::shutter_callback_impl, this), "Could not set up shutter callback");
}

bool PylablibCamera::can_override_exposure() const
{
    // Exposure time must be provided manually since we might be using an external light source (like a mercury lamp).
    return true;
}

std::optional<ICameraOptions> PylablibCamera::options() const
{
    ICameraOptions options;
    // TODO: Is this even still necessary?
    options.shutter_time_ms = DEFAULT_SHUTTER_TIME_MS;
    return options;
}

void PylablibCamera::start_capture(const boost::asio::ip::address &peer_address, const std::optional<size_t> &exposure_time_ms_override)
{
    boost::property_tree::ptree request;
    request.add<std::string>("command", "camera_setup_and_start");
    request.add<size_t>("camera_parameters.exposure_time_ms", (exposure_time_ms_override) ? *exposure_time_ms_override : DEFAULT_SHUTTER_TIME_MS);

    auto socket = send_command_message(io_context_, request, peer_address);
    wait_for_command_response(std::move(socket));
}

void PylablibCamera::stop_capture(const boost::asio::ip::address &peer_address, const std::optional<std::string> &path)
{
    boost::property_tree::ptree request;
    request.add<std::string>("command", "camera_stop_and_save");
    if (path)
    {
        request.add<std::string>("path", *path);
    }

    auto socket = send_command_message(io_context_, request, peer_address);
    wait_for_command_response(std::move(socket));
}

void PylablibCamera::capture_image(ShutterCallback shutter_callback, const boost::asio::ip::address &peer_address, const std::optional<std::string> &path)
{
    // Set up the callback...
    // XXX If the last future was not completed before setting a new callback (such as by calling this without waiting for the last one), that future is abandoned and will *never* be completed.
    shutter_callback_done_ = std::promise<void>();
    {
        std::lock_guard<std::mutex> shutter_callback_lock(shutter_callback_mutex_);
        shutter_callback_ = shutter_callback;
    }

    // ... trigger the camera...
    CHECK_RC(gpioWrite(trigger_pin_, false), "Could not pulse camera trigger");
    CHECK_RC(gpioWrite(trigger_pin_, true), "Could not pulse camera trigger");

    // ... save the images...
    boost::property_tree::ptree request;
    request.add<std::string>("command", "camera_wait");
    if (path)
    {
        request.add<std::string>("path", *path);
    }
    auto socket = send_command_message(io_context_, request, peer_address);
    wait_for_command_response(std::move(socket));

    // ... and ensure the callback finished successfully.
    shutter_callback_done_.get_future().get();
}

void PylablibCamera::shutter_callback_impl(int user_gpio, int level, uint32_t tick, void *user_data)
{
    PylablibCamera *self = static_cast<PylablibCamera *>(user_data);

    // Make sure this is the callback we're looking for.
    // pigpio uses `unsigned int`s in `gpiosSetISRFuncEx` but signed `int`s in `gpioISRFuncEx_t`.
    // Seems like an oversight in the library.
    if (self->shutter_callback_ && (unsigned int) user_gpio == self->shutter_pin_)
    {
        std::lock_guard<std::mutex> shutter_callback_lock(self->shutter_callback_mutex_);
        auto rolling_shutter_complete = std::chrono::steady_clock::now() + std::chrono::duration<unsigned int, std::milli>(self->rolling_shutter_wait_ms_);
        self->shutter_callback_(rolling_shutter_complete);

        // Unregister the callback to ensure oneshot behavior.
        // Use-after-free warning:
        // TODO: This also needs to be unregistered when anything the callback uses falls out of scope.
        self->shutter_callback_ = nullptr;

        self->shutter_callback_done_.set_value();
    }
}

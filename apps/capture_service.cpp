#include <iostream>
#include <sstream>
#include <unordered_map>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio.hpp>
#include <boost/property_tree/json_parser.hpp>


#include "camera/icamera.hpp"
#include "camera/pi_camera.hpp"
#include "camera/pylablib_camera.hpp"
#include "camera/vc_camera.hpp"
#include "gui/status_server.hpp"
#include "hal/hal.hpp"
#include "protocol/cleaving.hpp"
#include "protocol/image_sequence.hpp"
#include "utils/c_utils.hpp"

using boost::asio::ip::tcp;

const static std::string VERSION = "7.1.0";
const static unsigned short API_PORT = 45400;
const static unsigned short ADJUSTMENTS_API_PORT = 45404;

// 16 kilobytes. Needs to be large enough to fit an entire ImageSequence.
constexpr size_t MAX_REQUEST_SIZE = 16 << 10;

// Retrieve the serial number as reported by the "Serial" entry in /proc/cpuinfo.
// Returns an empty string if one does not exist.
std::string get_serial_number()
{
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string cpuinfo_line;
    std::string serial;
    while(std::getline(cpuinfo, cpuinfo_line))
    {
        if (boost::starts_with(cpuinfo_line, "Serial"))
        {
            const static std::string SEPARATOR = ": ";
            size_t separator_index = cpuinfo_line.find(SEPARATOR);
            size_t serial_start = separator_index + SEPARATOR.size();
            if (cpuinfo_line.size() > serial_start)
            {
                serial = cpuinfo_line.substr(serial_start);
            }
        }
    }

    return serial;
}

// TODO: Use this to implement a configuration side channel (for focusing)
class IpRpcService
{
public:
    typedef std::function<boost::property_tree::ptree(const boost::property_tree::ptree &, std::function<bool()> &, const boost::asio::ip::address &)> CommandType;
    typedef std::unordered_map<std::string, CommandType> CommandMap;

    IpRpcService(boost::asio::io_context &io_context, short api_port);

    void set_commands(const CommandMap &commands);

private:
    void listen_and_handle_request();

    boost::asio::io_context &io_context_;
    tcp::acceptor acceptor_;
    CommandMap commands_;
};

IpRpcService::IpRpcService(boost::asio::io_context &io_context, short api_port)
    : io_context_(io_context)
    , acceptor_(io_context_, tcp::endpoint(tcp::v4(), api_port))
{
    listen_and_handle_request();
}

void IpRpcService::set_commands(const CommandMap &commands)
{
    commands_ = commands;
}

void IpRpcService::listen_and_handle_request()
{
    acceptor_.async_accept([this](boost::system::error_code ec, tcp::socket peer_socket)
    {
        // Keep listening for more requests after this one completes, regardless of whether it succeeded.
        // TODO: Is this eligible for tail call optimization? Look here if we're trashing the stack after running for too long.
        CleanupHelper accept_next_peer(std::bind(&IpRpcService::listen_and_handle_request, this));

        if (ec)
        {
            std::cerr << "Error waiting for client: " << ec << std::endl;
            return;
        }

        boost::property_tree::ptree response;
        try
        {
            // Read a request. The entire request must be read at once.
            boost::asio::streambuf request_stream;
            boost::asio::streambuf::mutable_buffers_type request_buffer = request_stream.prepare(MAX_REQUEST_SIZE);
            size_t bytes_read = peer_socket.receive(request_buffer);
            request_stream.commit(bytes_read);

            // Parse the request.
            std::istream request_istream(&request_stream);
            boost::property_tree::ptree request_json;
            boost::property_tree::read_json(request_istream, request_json);

            // Log it, re-encoding so that we can format it all on one line.
            std::stringstream reencoded_request;
            boost::property_tree::write_json(reencoded_request, request_json, false);
            std::cerr << "Received command: " << reencoded_request.str() << std::endl;

            // Callback that determines whether the command should be aborted.
            // Commands should periodically call this to check.
            std::function<bool()> interrupt_requested = [&peer_socket]()
            {
                // `peer_socket->is_open()` only checks that any previous operations did not fail.
                // Instead, fall back on the old-school UNIX socket interface which directly checks the socket's status.
                timeval timeout
                {
                    0, // seconds
                    0 // microseconds
                };
                fd_set socketFdSet;
                FD_ZERO(&socketFdSet);
                int native_socket = peer_socket.native_handle();
                FD_SET(native_socket, &socketFdSet);
                int rc = select(native_socket + 1, &socketFdSet, nullptr, nullptr, &timeout);
                // 1 or greater: this many file descriptors are available for I/O or have an exception (like closing the connection).
                // We interpret either case as the client requesting interruption.
                // 0: No file descriptors became available during the timeout period (i.e. the client did not write anything but also didn't close the connection).
                // -1: Some other error.
                return rc != 0;
            };

            // Try to do the request.
            const std::string &command = request_json.get<std::string>("command");
            const boost::property_tree::ptree &args = request_json.get_child("args");
            response.add_child("response", commands_.at(command)(args, interrupt_requested, peer_socket.remote_endpoint().address()));
            response.add<bool>("success", true);
        }
        catch (const std::exception &e)
        {
            response.add<std::string>("error_message", e.what());
            response.add<bool>("success", false);
            std::cerr << "Command failed: " << e.what() << std::endl;
        }
        catch (...)
        {
            response.add<std::string>("error_message", "Unknown error");
            response.add<bool>("success", false);
            std::cerr << "Command failed for unknown reason" << std::endl;
        }

        try
        {
            // Send the response.
            std::stringstream response_stream;
            boost::property_tree::write_json(response_stream, response, false);
            std::string response_string = response_stream.str();
            std::cerr << "Sending response: " << response_string << std::endl;
            boost::asio::write(peer_socket, boost::asio::buffer(response_string.c_str(), response_string.size()));
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error sending the response: " << e.what() << std::endl;
        }
        catch(...)
        {
            std::cerr << "Unknown error sending the response" << std::endl;
        }
    });
}

// Implements socket-based APIs to use the hardware at a high level
class CaptureService
{
public:
    CaptureService(boost::asio::io_context &io_context, short api_port, std::unique_ptr<GpioHal> &&hal, std::unique_ptr<ICamera> &&camera);

private:
    void adjustments_service_loop();

    IpRpcService service_;
    std::unique_ptr<GpioHal> hal_;
    std::unique_ptr<ICamera> camera_;

    boost::asio::io_context adjustments_service_context_;
    IpRpcService adjustments_service_;
    std::thread adjustments_service_thread_;
};

CaptureService::CaptureService(boost::asio::io_context &io_context, short api_port, std::unique_ptr<GpioHal> &&hal, std::unique_ptr<ICamera> &&camera)
    : service_(io_context, api_port)
    , hal_(std::move(hal))
    , camera_(std::move(camera))
    , adjustments_service_(adjustments_service_context_, ADJUSTMENTS_API_PORT)
{
    // Main commands
    const static auto image_sequence_or_live_preview = [this](bool live_preview, const boost::property_tree::ptree &args, std::function<bool()> &interrupt_requested, const boost::asio::ip::address &peer_address)
    {
        const boost::property_tree::ptree &sequence_json = args.get_child("sequence");
        auto boost_output_dir = args.get_optional<std::string>("output_dir");
        std::optional<std::string> output_dir = (boost_output_dir.is_initialized()) ? std::make_optional<std::string>(*boost_output_dir) : std::nullopt;
        auto boost_exposure_time_ms_override = args.get_optional<size_t>("exposure_time_ms_override");
        std::optional<size_t> exposure_time_ms_override = (boost_exposure_time_ms_override.is_initialized()) ? std::make_optional<size_t>(*boost_exposure_time_ms_override) : std::nullopt;
        ImageSequence sequence(sequence_json, live_preview);

        // TODO: Need a way to continue receiving commands in live preview to change focus -- adjustments thread and port?
        // TODO: Need to prevent SIGINT from killing us if we haven't finished saving images

        sequence.run(hal_.get(), camera_.get(), exposure_time_ms_override, output_dir, interrupt_requested, peer_address);
        return boost::property_tree::ptree();
    };

    const static IpRpcService::CommandMap COMMANDS {
        {"get_metadata", [this](const boost::property_tree::ptree &args, std::function<bool()> &interrupt_requested, const boost::asio::ip::address &peer_address)
        {
            boost::property_tree::ptree response;
            response.add<std::string>("serial_number", get_serial_number());
            // TODO: Formalize this after ripping out of libcamera-apps
            response.add<std::string>("hal_version", VERSION);
            response.add<bool>("filter_control", hal_->filter_control());
            response.add<bool>("temperature_control", hal_->has_temperature_controller());
            response.add<bool>("focus_control", hal_->has_focus_controller());
            response.add<bool>("can_override_exposure", camera_->can_override_exposure());
            std::optional<ICameraOptions> camera_options = camera_->options();
            if (camera_options)
            {
                // TODO: Other fields?
                // TODO: Might want a method on this that populates a ptree directly
                response.add<size_t>("camera_options.shutter_time_ms", camera_options->shutter_time_ms);
            }
            return response;
        }},
        {"run_image_sequence", std::bind(image_sequence_or_live_preview, false, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)},
        {"run_live_preview", std::bind(image_sequence_or_live_preview, true, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)},
        {"cleave", [this](const boost::property_tree::ptree &args, std::function<bool()> &interrupt_requested, const boost::asio::ip::address &peer_address)
        {
            const boost::property_tree::ptree &cleave_args = args.get_child("cleave_args");
            auto boost_output_dir = args.get_optional<std::string>("output_dir");
            std::optional<std::string> output_dir = (boost_output_dir.is_initialized()) ? std::make_optional<std::string>(*boost_output_dir) : std::nullopt;
            Cleaving cleaving(cleave_args);
            cleaving.run(hal_.get(), camera_.get(), output_dir, peer_address);
            return boost::property_tree::ptree();
        }},
        {"flash_leds", [this](const boost::property_tree::ptree &args, std::function<bool()> &interrupt_requested, const boost::asio::ip::address &peer_address)
        {
            std::vector<FlashParameters> flashes;
            for (const auto &parsed_flash : args.get_child("flashes"))
            {
                flashes.emplace_back(parsed_flash.second);
            }

            std::vector<std::future<void>> flash_futures;
            for (const FlashParameters &flash : flashes)
            {
                flash_futures.push_back(hal_->flash_led_async(flash.led, flash.duration_ms, flash.pwm));
            }
            for (auto &future : flash_futures)
            {
                future.get();
            }

            return boost::property_tree::ptree();
        }},
        {"wait_for_temperature", [this](const boost::property_tree::ptree &args, std::function<bool()> &interrupt_requested, const boost::asio::ip::address &peer_address)
        {
            const boost::property_tree::ptree &temperature_args = args.get_child("temperature_args");
            float target_temperature_kelvin = temperature_args.get<float>("target_temperature_kelvin");
            auto wait_time = std::chrono::duration<size_t>(temperature_args.get<size_t>("wait_time_s"));
            auto hold_time = std::chrono::duration<size_t>(temperature_args.get<size_t>("hold_time_s"));

            if (hal_->has_temperature_controller())
            {
                hal_->temperature_controller().set_temp_kelvin_for(target_temperature_kelvin, wait_time, hold_time, interrupt_requested);
            }
            else
            {
                throw std::logic_error("Temperature controller not configured");
            }
            return boost::property_tree::ptree();
        }},
        {"disable_heater", [this](const boost::property_tree::ptree &args, std::function<bool()> &interrupt_requested, const boost::asio::ip::address &peer_address)
        {
            if (hal_->has_temperature_controller())
            {
                hal_->temperature_controller().disable();
            }
            return boost::property_tree::ptree();
        }},
        {"reset_filter_wheel", [this](const boost::property_tree::ptree &args, std::function<bool()> &interrupt_requested, const boost::asio::ip::address &peer_address)
        {
            if (hal_->filter_control())
            {
                hal_->reset_filter_wheel();
            }
            return boost::property_tree::ptree();
        }},
        {"reconfigure_camera", [](const boost::property_tree::ptree &args, std::function<bool()> &interrupt_requested, const boost::asio::ip::address &peer_address)
        {
            // TODO: Implement
            // TODO: Be careful with this one -- likely need to block until all ongoing saves are complete
            throw std::logic_error("Not implemented");
            return boost::property_tree::ptree();
        }}
    };

    // Adjustments commands
    service_.set_commands(COMMANDS);

    const static IpRpcService::CommandMap ADJUSTMENTS_COMMANDS {
        {"nudge_base_focus", [this](const boost::property_tree::ptree &args, std::function<bool()> &interrupt_requested, const boost::asio::ip::address &peer_address)
        {
            const boost::property_tree::ptree &nudge_base_focus_args = args.get_child("nudge_base_focus_args");
            int steps = nudge_base_focus_args.get<int>("steps");
            hal_->focus_controller().nudge_base_focus(steps);
            hal_->focus_controller().wait_for_focus_move();
            return boost::property_tree::ptree();
        }}
    };

    adjustments_service_.set_commands(ADJUSTMENTS_COMMANDS);

    // TODO: This can be avoided by switching to non-blocking IO
    adjustments_service_thread_ = std::thread(&CaptureService::adjustments_service_loop, this);
}

void CaptureService::adjustments_service_loop()
{
    adjustments_service_context_.run();
}

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage:\n";
        std::cerr << argv[0] << " hal_config_path.json pi_camera_config_path.ini [--color]\n";
        std::cerr << argv[0] << " hal_config_path.json pylablib_camera_config_path.json\n";
        return -1;
    }

    const std::string &hal_config_path = argv[1];
    const std::string &camera_config_path = argv[2];

    boost::asio::io_context io_context;
    StatusServer status_server(io_context);

    // HAL *must* be constructed first since it initializes GPIO.
    std::cerr << "Setting up GPIO from " << hal_config_path << std::endl;
    auto hal = GpioHal::from_file(io_context, status_server, hal_config_path);
    std::unique_ptr<ICamera> camera;

    boost::property_tree::ptree camera_config;
    boost::property_tree::read_json(camera_config_path, camera_config);
    auto vc_camera_config = camera_config.get_child_optional("vc_camera_config");
    auto libcamera_config = camera_config.get_child_optional("libcamera_config");
    auto pylablib_config = camera_config.get_child_optional("pylablib_config");
    if (vc_camera_config.is_initialized())
    {
        std::cerr << "Setting up a Vision Components camera from config at " << camera_config_path << std::endl;
        camera = std::make_unique<VisionComponentsCamera>(*vc_camera_config);
        camera->initialize(*vc_camera_config);
    }
    else if (libcamera_config.is_initialized())
    {
        std::cerr << "Setting up a libcamera-based camera from config at " << camera_config_path << std::endl;
        camera = std::make_unique<PiCamera>(*libcamera_config);
        camera->initialize(*libcamera_config);
    }
    else if (pylablib_config.is_initialized())
    {
        std::cerr << "Setting up a pylablib-based camera from config at " << camera_config_path << std::endl;
        camera = std::make_unique<PylablibCamera>(io_context, *pylablib_config);
        camera->initialize(*pylablib_config);
    }
    else
    {
        throw std::logic_error("No compatible camera configured");
    }

    CaptureService service(io_context, API_PORT, std::move(hal), std::move(camera));
    std::cerr << "Listening on port " << API_PORT << std::endl;
    io_context.run();
}

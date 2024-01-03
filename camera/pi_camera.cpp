#include <filesystem>

#include "core/options.hpp"

#include "utils/c_utils.hpp"

#include "pi_camera.hpp"

PiCamera::PiCamera(const boost::property_tree::ptree &camera_config)
    : V4l2Camera(camera_config)
{
    std::string sensor_type = camera_config.get<std::string>("sensor_type");
    if (sensor_type == "imx477")
    {
        output_color_ = true;
    }
    else if (sensor_type == "imx477_debayered")
    {
        output_color_ = false;
    }
    else
    {
        throw std::logic_error("Unknown libcamera sensor type " + sensor_type);
    }
}

void PiCamera::connect_camera(const boost::property_tree::ptree &camera_config)
{
    std::string libcamera_ini_path = camera_config.get<std::string>("libcamera_ini_path");
    std::vector<const char *> camera_options_argv {
        "PiCamera::connect_camera",
        "--config",
        libcamera_ini_path.c_str()
    };

    Options *camera_options = camera_.GetOptions();
    // const_cast: Matching the `int main(int argc, char **argv)` interface. `Parse()` never modifies the contents.
    camera_options->Parse(camera_options_argv.size(), const_cast<char **>(camera_options_argv.data()));
    camera_options->nopreview = true;
    camera_options->Print(); // XXX
    assert(camera_options->shutter);
    camera_.OpenCamera();
    camera_.ConfigureStill(LibcameraApp::FLAG_STILL_RAW);
    camera_.StartCamera();
}

V4l2Camera::ImageData PiCamera::block_and_receive_image()
{
    LibcameraApp::Msg msg = camera_.Wait();
    auto &completed_request = std::get<CompletedRequestPtr>(msg.payload);
    // TODO: This can be retrieved from completed_request->metadata directly
    uint64_t libcamera_timestamp = completed_request->buffers.begin()->second->metadata().timestamp;
    auto timestamp = std::chrono::steady_clock::time_point(std::chrono::duration<uint64_t, std::nano>(libcamera_timestamp));
    LibcameraApp::Stream *stream = camera_.RawStream();
    std::vector<libcamera::Span<uint8_t>> spans = camera_.Mmap(completed_request->buffers[stream]);
    const StreamInfo &info = camera_.GetStreamInfo(stream);
    LOG(1, std::chrono::steady_clock::now().time_since_epoch().count() << ": Received an image at " << timestamp.time_since_epoch().count());

    return {
        spans,
        info,
        timestamp,
        // The buffers inside `spans` are only valid during the lifetime of `completed_request`.
        CleanupHelper([completed_request = std::move(completed_request)]() {
            // No-op to ensure that the lambda capture is not optimized away.
            volatile auto sequence = completed_request->sequence;
            ++sequence;
        })
    };
}

std::optional<ICameraOptions> PiCamera::options() const
{
    Options *camera_options = camera_.GetOptions();
    ICameraOptions options;
    options.shutter_time_ms = camera_options->shutter / 1000;
    return options;
}
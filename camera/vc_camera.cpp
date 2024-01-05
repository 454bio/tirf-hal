#include <iostream>

#include "utils/c_utils.hpp"

#include "vc_camera.hpp"

constexpr unsigned int VC_BUFFER_COUNT = 3;

VisionComponentsCamera::VisionComponentsCamera(const boost::property_tree::ptree &camera_config)
    : V4l2Camera(camera_config)
    , camera_device_path_(camera_config.get<std::string>("camera_device_path"))
    , exposure_time_us_(camera_config.get<uint32_t>("parameters.exposure_time_us"))
    , buffer_index_(0)
{}

std::optional<ICameraOptions> VisionComponentsCamera::options() const
{
    ICameraOptions options;
    options.shutter_time_ms = exposure_time_us_ / 1000;
    return options;
}

void VisionComponentsCamera::connect_camera(const boost::property_tree::ptree &camera_config)
{
    CHECK_RC(sensor_open(camera_device_path_.data(), &sensor_, VC_BUFFER_COUNT), "Could not read sensor config");

    // TODO: Use this instead once implemented
    // set_parameters(camera_config.get_child("parameters"));
    int gain = camera_config.get<int>("parameters.gain");
    int shutter = camera_config.get<int>("parameters.exposure_time_us");
    sensor_set_shutter_gain(&sensor_, gain, shutter);
    std::cerr << "Full parameters not yet implemented\n";
    std::cerr << "Only setting gain=" << gain << " and shutter=" << shutter << std::endl;

    for (unsigned int buffer_index = 0; buffer_index < sensor_.qbufCount; ++buffer_index)
    {
        CHECK_RC(capture_buffer_enqueue(buffer_index, &sensor_), "Could not buffer initial images");
    }

    CHECK_RC(sensor_streaming_start(&sensor_), "Could not start streaming from camera");
}

V4l2Camera::ImageData VisionComponentsCamera::block_and_receive_image()
{
    constexpr static int SLEEP_TIMEOUT_US = 1000;
    timeval timestamp_tv;
    int rc;

    // These may need to run a few times until they successfully retrieve a frame.
    do {
        rc = sleep_for_next_capture(&sensor_, SLEEP_TIMEOUT_US);
        CHECK_RC(rc, "Error waiting for frame");
    } while (rc > 0);

    do {
        rc = capture_buffer_dequeue_ts(buffer_index_, &sensor_, &timestamp_tv);
        CHECK_RC(rc, "Error retrieving image");
    } while (rc > 0);

    CleanupHelper reenqueue_buffer([this]()
    {
        int enqueue_rc = capture_buffer_enqueue(buffer_index_, &sensor_);
        buffer_index_ = (buffer_index_ + 1) % sensor_.qbufCount;
        CHECK_RC(enqueue_rc, "Could not recycle buffer");
    });

    auto timestamp = std::chrono::steady_clock::time_point(std::chrono::duration<uint64_t, std::nano>(
        (uint64_t) timestamp_tv.tv_sec * 1000000000 + timestamp_tv.tv_usec * 1000));
    std::cerr << std::chrono::steady_clock::now().time_since_epoch().count() << ": Received an image at " << timestamp.time_since_epoch().count() << std::endl;

    // Convert the image into something our pipeline will accept by emulating libcamera's data structures.

    // First, read the metadata.
    StreamInfo info;
    info.width = sensor_.format.fmt.pix.width;
    info.height = sensor_.format.fmt.pix.height;
    // Unsure where this is normally derived, but all of the *12P formats have 3-byte stride.
    info.stride = 3; // XXX
    info.pixel_format = libcamera::PixelFormat(sensor_.format.fmt.pix.pixelformat);
    // Unsure of the mapping here, but our application is grayscale anyway.
    // info.colour_space = sensor_.format.fmt.pix.colorspace;

    // VC and libcamera both receive images from V4L2, resulting in a very similar memory layout.
    std::vector<libcamera::Span<uint8_t>> spans;
    spans.reserve(sensor_.qbuf[buffer_index_].planeCount);
    for (unsigned int i = 0; i < sensor_.qbuf[buffer_index_].planeCount; ++i)
    {
        // reinterpret_cast: the (unsigned) chars are used like raw bytes, not numeric values or text
        spans.emplace_back(
            reinterpret_cast<unsigned char *>(sensor_.qbuf[buffer_index_].st[i]),
            static_cast<size_t>(sensor_.qbuf[buffer_index_].plane[i].length));
    }

    return {
        spans,
        info,
        timestamp,
        // The buffers from the capture are only valid as long as the buffer is dequeued.
        std::move(reenqueue_buffer)
    };
}

void VisionComponentsCamera::set_parameters(const boost::property_tree::ptree &parameters)
{
    // Directly configure the camera using the V4L2 ioctl interface.
    // Necessary because the sensor_set_shutter_gain function provided by VC doesn't disable AGC.
    // This implementation is a fusion of the following:
    // * V4L2Device::setControls and V4L2Device::listControls in libcamera
    // * Options::Init and LibcameraApp::StartCamera in libcamera-apps
    // * sensor_set_shutter_gain in the Vision Components example code

    // We care about these:
    // "brightness": 0.0,
    // "contrast": 0.0,
    // "saturation": 1.0,
    // "sharpness": 1.0,
    // "awb": "custom",
    // "awbgains": "0,0",
    // "exposure": "normal",
    // "exposure_time_us": 250000,
    // "ev": 0,
    // "denoise": "off",
    // "gain": 8.0

    if (parameters.empty())
    {
        return;
    }

    std::vector<v4l2_ext_control> v4l2_controls(parameters.size());
    memset(v4l2_controls.data(), 0, sizeof(v4l2_ext_control) * parameters.size());

    for (const auto parameter : parameters)
    {
        // TODO: Actually convert the parameter
        // TODO: Need its name? ID? Type?
    }

    v4l2_ext_controls v4l2_controls_container = {};
    v4l2_controls_container.which = V4L2_CTRL_WHICH_CUR_VAL;
    v4l2_controls_container.controls = v4l2_controls.data();
    v4l2_controls_container.count = v4l2_controls.size();

    int rc = ioctl(sensor_.fd, VIDIOC_S_EXT_CTRLS, &v4l2_controls_container);
    CHECK_RC(rc, "Failed to configure camera");
    // TODO: Which one failed? See the end of V4L2Device::setControls
}

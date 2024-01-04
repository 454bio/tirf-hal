#pragma once

#include <chrono>
#include <map>
#include <optional>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "utils/c_utils.hpp"

#include "icamera.hpp"
#include "image_buffer.hpp"

class ImagePreviewServer
{
public:
    ImagePreviewServer(unsigned short port);

    void send_image(std::istream &image_stream);

private:
    void listen_loop();

    unsigned short port_;
    std::thread listen_thread_;
    std::optional<boost::asio::ip::tcp::socket> peer_socket_;
    std::mutex peer_socket_mutex_;
};

// Controller for Raspberry Pi-compatible cameras.
// Even though we require trigger control, we can't guarantee that the next image from the camera is the most recent.
// Instead, this object acts as a FIFO of the last `image_buffer_size` images.
// Images can be saved by their capture time, ensuring the correct image is retrieved.
class V4l2Camera : public ICamera
{
public:
    typedef std::chrono::duration<unsigned int, std::micro> duration_type;

    V4l2Camera(const boost::property_tree::ptree &camera_config);
    virtual void initialize(const boost::property_tree::ptree &camera_config);

    virtual bool can_override_exposure() const;
    virtual void capture_image(ShutterCallback shutter_callback, const boost::asio::ip::address &peer_address, const std::optional<std::string> &path);

protected:
    struct ImageData
    {
        std::vector<libcamera::Span<uint8_t>> data_ptr;
        StreamInfo stream_info;
        std::chrono::steady_clock::time_point timestamp;
        CleanupHelper deleter;
    };

    // Open and configure the camera itself.
    virtual void connect_camera(const boost::property_tree::ptree &camera_config) = 0;
    // Wait for and return the image data.
    // The ImageData from the last call must be destroyed prior to the next call to block_and_receive_image.
    virtual ImageData block_and_receive_image() = 0;

    bool output_color_;

private:
    constexpr static int SCHEMA_VERSION = 0;

    void camera_thread_loop();
    void watchdog_thread_loop();

    // Thread polling the camera for images
    std::thread camera_thread_;

    // Thread ensuring libcamera doesn't shut us down
    std::thread watchdog_thread_;
    std::chrono::duration<size_t> watchdog_period_;

    // Mapping from capture time to a reference to the image buffer...
    // `multimap` needed for the pre-run condition where all times are initialized to their defaults.
    std::multimap<std::chrono::steady_clock::time_point, OwningImageBuffer &> capture_times_;
    std::mutex capture_times_mutex_;

    // ... which is stored here.
    std::vector<OwningImageBuffer> image_buffers_;

    // Time between each shutter open -- not the same as the shutter speed!
    std::chrono::duration<float, std::milli> capture_period_;

    duration_type rolling_shutter_wait_;
    unsigned int trigger_pin_;

    std::optional<ImagePreviewServer> preview_server_;
    unsigned int preview_downsampling_;
};

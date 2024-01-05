#include <filesystem>

#include <boost/asio.hpp>
#include <pigpio.h>

#include "core/options.hpp"

#include "utils/c_utils.hpp"

#include "v4l2_camera.hpp"

using boost::asio::ip::tcp;

// Define an empty buffer as one that was created at the default-constructed time (i.e. time zero).
// This will make our logic always use an empty buffer first without needing a special case for empty buffers.
constexpr static auto UNPOPULATED_TIME = std::chrono::steady_clock::time_point();

// Maximum time to wait for the requested image to arrive.
constexpr static auto MAX_WAIT = std::chrono::duration<size_t>(4);

// Period to check for new images.
constexpr static auto POLLING_WAIT = std::chrono::duration<size_t, std::milli>(125);

// Period to capture images to keep libcamera alive.
constexpr static auto DEFAULT_WATCHDOG_PERIOD = std::chrono::duration<size_t>(300);

// How long to hold XVS down for when pulsing it.
constexpr static uint32_t TRIGGER_PULSE_DURATION_US = 125;

constexpr static size_t DEFAULT_IMAGE_BUFFER_SIZE = 24;
constexpr static auto CAMERA_INITIALIZATION_WAIT = std::chrono::duration<unsigned int>(3);

constexpr static unsigned int DEFAULT_PREVIEW_DOWNSAMPLING = 1;

class CameraTrigger
{
public:
    CameraTrigger(
        unsigned int gpio_pin,
        const V4l2Camera::duration_type &rolling_shutter_duration,
        const V4l2Camera::duration_type &capture_period);
    ~CameraTrigger();

private:
    void pulse();

    unsigned int gpio_pin_;
    V4l2Camera::duration_type rolling_shutter_duration_;
    V4l2Camera::duration_type capture_period_;
    std::chrono::steady_clock::time_point earliest_stop_time_;

    // static: Only one instance should trigger at a time
    inline static std::mutex mutex_;
    std::optional<std::lock_guard<std::mutex>> lock_;
};

CameraTrigger::CameraTrigger(
    unsigned int gpio_pin,
    const V4l2Camera::duration_type &rolling_shutter_duration,
    const V4l2Camera::duration_type &capture_period
)
    : gpio_pin_(gpio_pin)
    , rolling_shutter_duration_(rolling_shutter_duration)
    , capture_period_(capture_period)
{
    lock_.emplace(mutex_);
    earliest_stop_time_ = std::chrono::steady_clock::now() + capture_period_;
    pulse();
}

CameraTrigger::~CameraTrigger()
{
    // Wait for rolling shutter again...
    std::this_thread::sleep_for(rolling_shutter_duration_);

    // ... wait until the shutter has been open for at least the minimum configured time in the libcamera parameters...
    std::this_thread::sleep_until(earliest_stop_time_);

    // ... stop the exposure by pulsing trigger_pin_ again...
    pulse();

    // ... and give the camera some time to offload the image before requesting a new one.
    std::this_thread::sleep_for(capture_period_);

    // Mutex is released automatically.
}

void CameraTrigger::pulse()
{
    CHECK_RC(gpioWrite(gpio_pin_, false), "Could not pulse camera trigger");
    gpioDelay(TRIGGER_PULSE_DURATION_US);
    CHECK_RC(gpioWrite(gpio_pin_, true), "Could not pulse camera trigger");
}

ImagePreviewServer::ImagePreviewServer(unsigned short port)
    : port_(port)
    , listen_thread_(&ImagePreviewServer::listen_loop, this)
{}

void ImagePreviewServer::listen_loop()
{
    // Set up the socket endpoint
    boost::asio::io_context io_context;

    // No additional threading: one client at a time.
    // TODO: This doesn't have to be the case -- there's no technical reason we can't have more than one preview client
    // TODO: Avoid this busy loop by passing in the main io_context from CaptureService
    tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), port_));
    while (true)
    {
        tcp::endpoint peer_endpoint;
        CleanupHelper close_socket([this](){
            peer_socket_.reset();
        });

        try
        {
            peer_socket_ = acceptor.accept(peer_endpoint);

            // The client should never send us anything but this will allow us to wait until it closed.
            boost::asio::streambuf request_stream;
            boost::asio::streambuf::mutable_buffers_type request_buffer = request_stream.prepare(1);
            size_t bytes_read = peer_socket_->receive(request_buffer);
            request_stream.commit(bytes_read);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error waiting for client: " << e.what() << std::endl;
            continue;
        }
        catch (...)
        {
            std::cerr << "Unknown error waiting for client" << std::endl;
            continue;
        }
    }
}

void ImagePreviewServer::send_image(std::istream &image_stream)
{
    if (peer_socket_)
    {
        // Convert the image into an ASIO buffer
        boost::asio::streambuf image_buffer;
        std::ostream image_buffer_stream(&image_buffer);
        image_buffer_stream << image_stream.rdbuf() << std::flush;

        // Calculate its size and create a buffer to send just the size
        uint32_t image_size = image_buffer.size();
        boost::asio::streambuf size_buffer;
        std::ostream size_buffer_stream(&size_buffer);
        // Force the stream to make the 4 byte uint32_t actually take up only 4 bytes
        size_buffer_stream.write((char*)(&image_size), sizeof(image_size));
        {
            std::lock_guard<std::mutex> peer_socket_lock(peer_socket_mutex_);
            // Send the size first...
            boost::asio::write(*peer_socket_, size_buffer);
            // ... so the client knows how much to read here.
            boost::asio::write(*peer_socket_, image_buffer);
        }
    }
}

V4l2Camera::V4l2Camera(const boost::property_tree::ptree &camera_config)
    : output_color_(false)
{
    assert(camera_config.get<int>("schema_version") == SCHEMA_VERSION);
    trigger_pin_ = camera_config.get<unsigned int>("trigger_pin");
    unsigned int rolling_shutter_wait_us = camera_config.get<unsigned int>("rolling_shutter_wait_ms") * 1000;
    rolling_shutter_wait_ = V4l2Camera::duration_type(rolling_shutter_wait_us);

    auto preview_port = camera_config.get_optional<unsigned short>("preview_port");
    auto preview_downsampling = camera_config.get_optional<unsigned int>("preview_downsampling");
    preview_downsampling_ = preview_downsampling.is_initialized() ? *preview_downsampling : DEFAULT_PREVIEW_DOWNSAMPLING;
    if (preview_downsampling_ != 1 && preview_downsampling_ % 2 != 0)
    {
        throw std::logic_error("preview_downsampling must be equal to 1 or even");
    }

    auto watchdog_period_from_config = camera_config.get_optional<unsigned int>("watchdog_period_s");
    watchdog_period_ = watchdog_period_from_config.is_initialized() ? std::chrono::duration<size_t>(*watchdog_period_from_config) : DEFAULT_WATCHDOG_PERIOD;

    // Set up trigger pin using pigpio.
    // Assumes that GPIO was already initialized using `gpioInitialise()`.
    CHECK_RC(gpioSetMode(trigger_pin_, PI_OUTPUT), "Could not set up camera shutter");

    // Assume GPIO is active low.
    CHECK_RC(gpioWrite(trigger_pin_, true), "Could not reset camera trigger");

    // Set up the preview server.
    if (preview_port.is_initialized())
    {
        LOG(1, "Setting up preview on port " << *preview_port);
        preview_server_.emplace(*preview_port);
    }
}

void V4l2Camera::initialize(const boost::property_tree::ptree &camera_config)
{
    auto image_buffer_size_from_config = camera_config.get_optional<size_t>("image_buffer_size");
    size_t image_buffer_size = (image_buffer_size_from_config.is_initialized()) ? *image_buffer_size_from_config : DEFAULT_IMAGE_BUFFER_SIZE;
    assert(image_buffer_size > 0);

    // Initialize the camera...
    connect_camera(camera_config);

    // ... set up the image buffer...
    image_buffers_ = std::vector<OwningImageBuffer>(image_buffer_size);
    for (auto &buffer : image_buffers_)
    {
        capture_times_.insert(decltype(capture_times_)::value_type {UNPOPULATED_TIME, buffer});
    }

    // ... start capturing images...
    camera_thread_ = std::thread(&V4l2Camera::camera_thread_loop, this);
    capture_period_ = std::chrono::duration<float, std::milli>(options()->shutter_time_ms * 1.2);

    // ... "prime" the camera by triggering with the goal of filling all of the buffers...
    std::this_thread::sleep_for(CAMERA_INITIALIZATION_WAIT);
    for (size_t i = 0; i < image_buffer_size; ++i)
    {
        CameraTrigger trigger(trigger_pin_, rolling_shutter_wait_, std::chrono::duration_cast<V4l2Camera::duration_type>(capture_period_));
    }
    std::this_thread::sleep_for(CAMERA_INITIALIZATION_WAIT);
    size_t empty_buffer_count;
    {
        std::lock_guard<std::mutex> capture_times_lock(capture_times_mutex_);
        empty_buffer_count = capture_times_.count(UNPOPULATED_TIME);
    }
    LOG(1, "Camera initialized after " << image_buffer_size-empty_buffer_count << " of " << image_buffer_size << " images were primed, continuing");

    // ... and start a watchdog thread that periodically captures images to convince libcamera that everything is OK.
    if (watchdog_period_.count() != 0)
    {
        watchdog_thread_ = std::thread(&V4l2Camera::watchdog_thread_loop, this);
    }
}

void V4l2Camera::camera_thread_loop()
{
    // TODO: Stop condition
    while (true)
    {
        // Wait for and retrieve the image...
        ImageData image = block_and_receive_image();

        {
            std::lock_guard<std::mutex> capture_times_lock(capture_times_mutex_);

            // ... choose the first buffer, which is always the oldest...
            decltype(capture_times_)::iterator buffer_it = capture_times_.begin();

            // ... replace it...
            OwningImageBuffer &element = buffer_it->second;
            element.replace_with_libcamera_spans(image.data_ptr, image.stream_info);

            // ... and update the timestamp.
            capture_times_.erase(buffer_it);
            capture_times_.insert(decltype(capture_times_)::value_type {image.timestamp, element});
        }
    }
}

void V4l2Camera::watchdog_thread_loop()
{
    while (true)
    {
        std::this_thread::sleep_for(watchdog_period_);
        CameraTrigger trigger(trigger_pin_, rolling_shutter_wait_, std::chrono::duration_cast<V4l2Camera::duration_type>(capture_period_));
    }
}

bool V4l2Camera::can_override_exposure() const
{
    // Exposure time will be automatically determined from the flashes.
    return false;
}

void V4l2Camera::capture_image(ShutterCallback shutter_callback, const boost::asio::ip::address &peer_address, const std::optional<std::string> &path)
{
    std::chrono::steady_clock::time_point flash_end_time;
    {
        CameraTrigger trigger(trigger_pin_, rolling_shutter_wait_, std::chrono::duration_cast<V4l2Camera::duration_type>(capture_period_));

        // Actually call the callback.
        // The callback must not return until all of the flashes are complete.
        if (shutter_callback)
        {
            auto rolling_shutter_complete = std::chrono::steady_clock::now() + rolling_shutter_wait_;
            LOG(1, "Starting flashes at " << std::chrono::steady_clock::now().time_since_epoch().count());
            shutter_callback(rolling_shutter_complete);
        }

        // "SensorTimestamp is sampled when the frame start packet is picked up by the Unicam CSI-2 peripheral."
        // https://forums.raspberrypi.com/viewtopic.php?t=355707#p2131765
        // https://github.com/raspberrypi/linux/blob/9b79cb06ad370cfd5fc52f7d85d82bdcff701828/drivers/media/platform/bcm2835/bcm2835-unicam.c#L984
        // The camera doesn't send anything to the Pi until the exposure is finished.
        // This makes `trigger`'s second `pulse()`, called shortly after this scope exit, a reasonable estimate for the expected timestamp.
        flash_end_time = std::chrono::steady_clock::now();
        LOG(1, "Shutter closing at " << flash_end_time.time_since_epoch().count());
    }

    // Finally, save the corresponding image.
    auto save_impl = [this, flash_end_time, path]()
    {
        auto give_up_time = flash_end_time + MAX_WAIT;
        do
        {
            // Try to find it...
            {
                std::lock_guard<std::mutex> capture_times_lock(capture_times_mutex_);

                // ... skipping if there's no way that we have it...
                const auto& first_timestamp = capture_times_.cbegin()->first;
                if (flash_end_time + capture_period_ < first_timestamp)
                {
                    LOG(1, "Requested a save of an image that is too old -- first is " << first_timestamp.time_since_epoch().count());
                    throw std::runtime_error("Requested a save of an image that is too old");
                }

                decltype(capture_times_)::const_iterator capture_it = capture_times_.upper_bound(flash_end_time);
                if (capture_it != capture_times_.cend())
                {
                    // ... and write it to both the preview and to disk.
                    if (preview_server_)
                    {
                        std::stringstream preview_stream;
                        capture_it->second.save_tiff(preview_stream, output_color_, preview_downsampling_);
                        preview_stream.seekg(0);
                        preview_server_->send_image(preview_stream);
                    }
                    if (path)
                    {
                        std::ofstream tiff_file(*path);
                        capture_it->second.save_tiff(tiff_file, output_color_);
                    }
                    return;
                }
            }

            // If we couldn't find it yet, wait and try again.
            std::this_thread::sleep_for(POLLING_WAIT);
        }
        while (std::chrono::steady_clock::now() < give_up_time);

        LOG(1, "Image didn't arrive on time, giving up");
        throw std::runtime_error("Image didn't arrive on time, giving up");
    };
    // TODO: Maybe this should return a std::future from std::async instead
    save_impl();
}

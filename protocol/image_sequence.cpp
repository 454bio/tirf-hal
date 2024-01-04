#include <boost/algorithm/string/replace.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "core/libcamera_app.hpp"

#include "camera/icamera.hpp"
#include "hal/hal.hpp"
#include "utils/c_utils.hpp"

#include "image_sequence.hpp"

const static std::string DEFAULT_FILENAME_FORMAT = "$timestamp-ImageSequence-$sequenceLabel-$imageIndex-$imageLabel.tif";
constexpr static size_t MAX_CAPTURE_TRIES = 5;

FlashParameters::FlashParameters(const boost::property_tree::ptree &parsed)
    : led(GPIO_LED_NAME_TO_ENUM.at(parsed.get<std::string>("led")))
    , duration_ms(parsed.get<unsigned int>("duration_ms"))
{
    const auto parsed_pwm = parsed.get_optional<unsigned short>("intensity_per_mille");
    pwm = (parsed_pwm.is_initialized()) ? *parsed_pwm : PWM_MAX;
}

ImageParameters::ImageParameters(const boost::property_tree::ptree &parsed)
    : label(parsed.get<std::string>("label"))
{
    auto loaded_filename_format = parsed.get_optional<std::string>("filename");
    filename_format = (loaded_filename_format.is_initialized()) ? *loaded_filename_format : DEFAULT_FILENAME_FORMAT;

    const auto &filter_name = parsed.get_optional<std::string>("filter");
    if (filter_name)
    {
        filter = FILTER_NAME_TO_ENUM.at(*filter_name);
    }
    else
    {
        filter = LensFilter::NO_FILTER;
    }

    const auto &parsed_flashes = parsed.get_child_optional("flashes");
    longest_flash_duration_ms = 0;
    if (parsed_flashes)
    {
        for (const auto &flash : *parsed_flashes)
        {
            flashes.emplace_back(flash.second);
            unsigned int duration_ms = flashes.back().duration_ms;
            if (duration_ms > longest_flash_duration_ms)
            {
                longest_flash_duration_ms = duration_ms;
            }
        }
    }
}

ImageSequence::ImageSequence(const boost::property_tree::ptree &parsed, bool live_preview)
    : live_preview_(live_preview)
{
    assert(parsed.get<int>("schema_version") == SCHEMA_VERSION);

    label_ = parsed.get<std::string>("label");
    for (const auto &parsed_image : parsed.get_child("images"))
    {
        images_.emplace_back(parsed_image.second);
    }
}

ImageSequence ImageSequence::from_file(const std::string &filename)
{
    boost::property_tree::ptree image_sequence_json;
    boost::property_tree::read_json(filename, image_sequence_json);
    return ImageSequence(image_sequence_json);
}

void ImageSequence::run(GpioHal *hal, ICamera *camera, const std::optional<size_t> &exposure_time_ms_override, const std::optional<std::string> &output_dir, std::function<bool()> &interrupt_requested, const boost::asio::ip::address &peer_address) const
{
    assert(hal);
    assert(camera);

    // TODO: Calculate an appropriate exposure time from the requested flashes (max flash + some fixed amount of time around 100ms)
    camera->start_capture(peer_address, exposure_time_ms_override);
    CleanupHelper stop_capture([&output_dir, &camera, &peer_address]()
    {
        std::optional<std::string> remaining_images_prefix;
        if (output_dir)
        {
            // If we have images remaining, we don't know where they came from. Just output them with a timestamp.
            auto timestamp = std::chrono::system_clock::now();
            std::string timestamp_str = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(timestamp.time_since_epoch()).count());
            remaining_images_prefix = *output_dir + "/" + timestamp_str;
        }
        camera->stop_capture(peer_address, remaining_images_prefix);
    });

    do
    {
        for (size_t image_index = 0; image_index < images_.size(); ++image_index)
        {
            if (interrupt_requested())
            {
                return;
            }

            // TODO: This should return a future from the camera so that we can wait for all of the images to arrive at once.
            // This should also make it possible to retry individual images, and disable retrying and waiting for live preview.
            run_one(image_index, hal, camera, output_dir, peer_address);

            if (live_preview_)
            {
                // Live preview will effectively keep the LED on continuously.
                // This is not safe for our hardware, make sure it's off for twice the amount of time as it was on.
                std::this_thread::sleep_for(std::chrono::duration<unsigned int, std::milli>(images_[image_index].longest_flash_duration_ms * 2));
            }
        }
    }
    while (live_preview_);
}

void ImageSequence::run_one(size_t image_index, GpioHal *hal, ICamera *camera, const std::optional<std::string> &output_dir, const boost::asio::ip::address &peer_address) const
{
    const ImageParameters &params = images_[image_index];

    // Simultaneously move the filter and focus into position.
    hal->change_filter_async(params.filter, peer_address);
    if (hal->has_focus_controller())
    {
        hal->focus_controller().change_focus_async(FILTER_TO_FOCUS_ENUM.at(params.filter));
    }
    hal->wait_for_filter_move();
    if (hal->has_focus_controller())
    {
        hal->focus_controller().wait_for_focus_move();
    }

    auto shutter_callback = [this, &hal, &params, &peer_address]
        (const std::chrono::steady_clock::time_point &rolling_shutter_complete)
    {
        std::this_thread::sleep_until(rolling_shutter_complete);

        // Simultaneously flash all of the requested LEDs -- all of the light is going to be integrated into the same frame, anyway.
        std::vector<std::future<void>> flash_futures;
        for (const FlashParameters &flash : params.flashes)
        {
            flash_futures.push_back(hal->flash_led_async(flash.led, flash.duration_ms, flash.pwm));
        }
        for (auto &future : flash_futures)
        {
            future.get();
        }
    };

    std::optional<std::string> filename;
    if (output_dir)
    {
        std::string image_index_str = (boost::format("%02d") % image_index).str();
        auto flash_time_for_filename = std::chrono::system_clock::now();
        std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(flash_time_for_filename.time_since_epoch()).count());
        filename = *output_dir + "/" + params.filename_format;
        boost::replace_first(*filename, "$timestamp", timestamp);
        boost::replace_first(*filename, "$imageIndex", image_index_str);
        boost::replace_first(*filename, "$imageLabel", params.label);
        boost::replace_first(*filename, "$sequenceLabel", label_);
    }

    for (size_t current_try = 0; current_try < MAX_CAPTURE_TRIES; ++current_try)
    {
        try
        {
            camera->capture_image(shutter_callback, peer_address, filename);
            break;
        }
        catch(const std::exception& e)
        {
            LOG(1, "capture_image failed on try " << current_try << ":" << e.what());
            if (current_try == MAX_CAPTURE_TRIES-1)
            {
                // Number of tries exceeded -- we're officially missing the image.
                // This is fatal for sequencing, so simply rethrow.
                throw;
            }
        }
    }
}

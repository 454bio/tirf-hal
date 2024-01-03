#include <chrono>
#include <cmath>
#include <iostream>

#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "camera/icamera.hpp"
#include "hal/hal.hpp"
#include "utils/c_utils.hpp"

#include "cleaving.hpp"

extern const std::unordered_map<std::string, GpioLed> GPIO_LED_NAME_TO_ENUM;
const static std::string DEFAULT_FILENAME_FORMAT = "$timestamp-Cleaving-$imageIndex.tif";

Cleaving::Cleaving(const boost::property_tree::ptree &parsed)
{
    assert(parsed.get<int>("schema_version") == SCHEMA_VERSION);

    capture_period_ms_ = parsed.get<unsigned int>("capture_period_ms");
    cleaving_duration_ms_ = parsed.get<unsigned int>("cleaving_duration_ms");
    auto loaded_filename_format = parsed.get_optional<std::string>("filename");
    filename_format_ = (loaded_filename_format.is_initialized()) ? *loaded_filename_format : DEFAULT_FILENAME_FORMAT;

    auto led = parsed.get_optional<std::string>("cleaving_led");
    cleaving_led_ = (led) ? GPIO_LED_NAME_TO_ENUM.at(*led) : GpioLed::UV;

    auto pwm = parsed.get_optional<unsigned short>("cleaving_intensity_per_mille");
    cleaving_pwm_ = (pwm) ? *pwm : PWM_MAX;

    auto filter_name = parsed.get_optional<std::string>("filter");
    filter_ = (filter_name.is_initialized()) ? FILTER_NAME_TO_ENUM.at(*filter_name) : LensFilter::NO_FILTER;
}

void Cleaving::run(GpioHal *hal, ICamera *camera, const std::optional<std::string> &output_dir, const boost::asio::ip::address &peer_address)
{
    assert(hal);
    assert(camera);

    hal->change_filter_async(filter_, peer_address);
    if (hal->has_focus_controller())
    {
        hal->focus_controller().change_focus_async(FILTER_TO_FOCUS_ENUM.at(filter_));
    }
    hal->wait_for_filter_move();
    if (hal->has_focus_controller())
    {
        hal->focus_controller().wait_for_focus_move();
    }

    size_t image_index = 0;
    std::future<void> flash_future = hal->flash_led_async(cleaving_led_, cleaving_duration_ms_, cleaving_pwm_);

    if (capture_period_ms_ > 0)
    {
        auto capture_period = std::chrono::duration<unsigned int, std::milli>(capture_period_ms_);

        camera->start_capture(peer_address);
        CleanupHelper stop_capture([this, &output_dir, &camera, &peer_address, &image_index]()
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
            auto time_captured_for_filename = std::chrono::system_clock::now();

            auto shutter_callback = [capture_period](const std::chrono::steady_clock::time_point &rolling_shutter_complete)
            {
                std::this_thread::sleep_until(rolling_shutter_complete);

                // LED will be on until flash_future. Just keep the shutter open for the specified amount of time.
                std::this_thread::sleep_for(capture_period);
            };

            std::optional<std::string> filename;
            if (output_dir)
            {
                std::string image_index_str = (boost::format("%02d") % image_index).str();
                std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time_captured_for_filename.time_since_epoch()).count());
                filename = *output_dir + "/" + filename_format_;
                boost::replace_first(*filename, "$timestamp", timestamp);
                boost::replace_first(*filename, "$imageIndex", image_index_str);
            }

            try
            {
                camera->capture_image(shutter_callback, peer_address, filename);
            }
            catch(const std::exception& e)
            {
                std::cerr << "Failed to capture cleave image -- continuing:" << e.what() << std::endl;
            }

            ++image_index;

            // The below wait_for assumes that capture_image blocks at least until the shutter is closed.
        } while (flash_future.wait_for(std::chrono::duration<unsigned int, std::milli>(1)) != std::future_status::ready);
    }
    else
    {
        flash_future.get();
    }
}

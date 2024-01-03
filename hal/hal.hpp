#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include <pigpio.h>

#include <boost/asio.hpp>
#include <boost/property_tree/ptree_fwd.hpp>

extern const unsigned short PWM_MAX;

class IMotor;
class StatusServer;

enum class GpioLed {
    RED, ORANGE, GREEN, BLUE, UV
};

extern const std::unordered_map<std::string, GpioLed> GPIO_LED_NAME_TO_ENUM;

class GpioLedImpl {
public:
    GpioLedImpl(const boost::property_tree::ptree &parsed);

    void setup_gpio();
    std::future<void> flash_async(unsigned int duration_ms, unsigned short pwm);

private:
    std::vector<unsigned int> gpio_pins_;
};

enum class LensFilter {
    UNKNOWN=-1, ANY_FILTER, NO_FILTER, RED, ORANGE, GREEN, BLUE
};

extern const std::unordered_map<std::string, LensFilter> FILTER_NAME_TO_ENUM;

struct LensFilterImpl
{
    LensFilterImpl(const std::string &name, int position);

    std::string name;
    int position;
};

class ITemperatureSensor
{
public:
    virtual std::optional<float> read_temp_kelvin() = 0;
};

extern std::function<bool()> DEFAULT_INTERRUPT_REQUESTED;

class TemperatureController
{
public:
    TemperatureController(StatusServer &status_server, std::vector<unsigned int> &&heater_gpio_pins, std::unique_ptr<ITemperatureSensor> &&temperature_sensor, float p, float i, float bias, unsigned short max_pwm);

    void set_temp_kelvin_for(float temp_kelvin, std::chrono::duration<size_t> wait_time, std::chrono::duration<size_t> hold_time, std::function<bool()> &interrupt_requested = DEFAULT_INTERRUPT_REQUESTED);
    void disable();

private:
    void control_thread_loop();
    void set_heater_state(unsigned int duty_cycle);

    StatusServer &status_server_;

    std::vector<unsigned int> heater_gpio_pins_;
    std::unique_ptr<ITemperatureSensor> temperature_sensor_;

    std::atomic<float> target_temp_kelvin_;
    std::atomic<float> last_temp_kelvin_;
    std::atomic<std::chrono::steady_clock::time_point> hold_temp_until_;
    std::thread control_thread_;

    const float control_p_;
    const float control_i_;
    // float control_d_; // Unused
    const float control_bias_;

    unsigned short heater_max_duty_cycle_;
};

enum class FocusPosition
{
    UNKNOWN=-1, BASE, RED, ORANGE, GREEN, BLUE
};

extern const std::unordered_map<LensFilter, FocusPosition> FILTER_TO_FOCUS_ENUM;

class FocusController
{
public:
    FocusController(StatusServer &status_server, const boost::property_tree::ptree &parsed);

    void change_focus_async(const FocusPosition &position);
    void wait_for_focus_move();
    void nudge_base_focus(int steps);

private:
    // Offset for each focus position away from the base.
    std::unordered_map<FocusPosition, int> position_steps_;

    StatusServer &status_server_;
    std::unique_ptr<IMotor> motor_;
};

class GpioHal
{
public:
    GpioHal(boost::asio::io_context &io_context, StatusServer &status_server, const boost::property_tree::ptree &parsed);
    static std::unique_ptr<GpioHal> from_file(boost::asio::io_context &io_context, StatusServer &status_server, const std::string &filename);
    ~GpioHal();

    bool filter_control() const;
    // The caller is responsible for waiting until the filter is done moving...
    void change_filter_async(const LensFilter& filter, const boost::asio::ip::address &peer_address);
    // ... by calling this. These functions handle both controllable and manual filters.
    void wait_for_filter_move();
    void reset_filter_wheel();

    bool has_focus_controller() const;
    FocusController &focus_controller();

    // Flash a LED for the specified amount of time.
    // Returns a future that is completed when the LED turns back off.
    // This method assumes that the last LED flash has already completed.
    std::future<void> flash_led_async(const GpioLed& led, unsigned int time_ms, unsigned short pwm);

    bool has_temperature_controller() const;
    TemperatureController &temperature_controller();

private:
    constexpr static int SCHEMA_VERSION = 4;

    boost::asio::io_context &io_context_;
    StatusServer &status_server_;

    std::unordered_map<GpioLed, GpioLedImpl> leds_;

    std::unique_ptr<IMotor> filter_motor_;
    std::unordered_map<LensFilter, LensFilterImpl> filters_;
    std::function<void()> filter_move_wait_callback_;

    std::optional<TemperatureController> temperature_controller_;
    std::optional<FocusController> focus_controller_;
};

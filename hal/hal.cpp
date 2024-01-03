#include <chrono>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <glob.h>

#include <boost/asio.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "gui/gui_command.hpp"
#include "gui/status_server.hpp"
#include "utils/c_utils.hpp"

#include "hal.hpp"
#include "motor.hpp"

constexpr unsigned short PWM_MAX = 1000;

const std::unordered_map<std::string, GpioLed> GPIO_LED_NAME_TO_ENUM {
    {"red", GpioLed::RED},
    {"orange", GpioLed::ORANGE},
    {"green", GpioLed::GREEN},
    {"blue", GpioLed::BLUE},
    {"uv", GpioLed::UV}
};

constexpr static float ZERO_CELSIUS = 273.15; // Kelvin

GpioLedImpl::GpioLedImpl(const boost::property_tree::ptree &parsed)
{
    for (const auto &parsed_pin : parsed.get_child("gpio_pins"))
    {
        gpio_pins_.emplace_back(parsed_pin.second.get_value<unsigned int>());
    }
}

void GpioLedImpl::setup_gpio()
{
    for (unsigned int pin : gpio_pins_) {
        CHECK_RC(gpioSetMode(pin, PI_OUTPUT), "Could not set up LED output");
        CHECK_RC(gpioPWM(pin, 0), "Could not initialize LED PWM");
        CHECK_RC(gpioSetPWMrange(pin, PWM_MAX), "Could not set LED PWM range");
    }
}

std::future<void> GpioLedImpl::flash_async(unsigned int duration_ms, unsigned short pwm)
{
    // The timing of this needs to be extremely tight for all users:
    //   * The image quality is strongly dependent on the visible flashes
    //   * The chemistry is strongly affected by the UV flash
    return std::async(std::launch::async, [this, duration_ms, pwm]() {
        for (unsigned int pin : gpio_pins_) {
            CHECK_RC(gpioPWM(pin, pwm), "Could not turn on LED");
        }

        std::this_thread::sleep_for(std::chrono::duration<unsigned int, std::milli>(duration_ms));

        for (unsigned int pin : gpio_pins_) {
            CHECK_RC(gpioPWM(pin, 0), "Could not turn off LED");
        }
    });
}

const std::unordered_map<std::string, LensFilter> FILTER_NAME_TO_ENUM {
    {"any_filter", LensFilter::ANY_FILTER},
    {"no_filter", LensFilter::NO_FILTER},
    {"red", LensFilter::RED},
    {"orange", LensFilter::ORANGE},
    {"green", LensFilter::GREEN},
    {"blue", LensFilter::BLUE}
};

const std::unordered_map<LensFilter, std::string> FILTER_ENUM_TO_READABLE_NAME {
    {LensFilter::ANY_FILTER, "any"},
    {LensFilter::NO_FILTER, "bandpass"},
    {LensFilter::RED, "red"},
    {LensFilter::ORANGE, "orange"},
    {LensFilter::GREEN, "green"},
    {LensFilter::BLUE, "blue"}
};

LensFilterImpl::LensFilterImpl(const std::string &name, int position)
    : name(name)
    , position(position)
{}

// 1-Wire temperature sensor. These directly report a temperature around once per second.
class W1TemperatureSensor : public ITemperatureSensor
{
public:
    W1TemperatureSensor(const std::string &sysfs_device_glob);

    virtual std::optional<float> read_temp_kelvin();

private:
    std::string sysfs_path_;
};

W1TemperatureSensor::W1TemperatureSensor(const std::string &sysfs_device_pattern)
{
    // Find the sysfs device corresponding to the temperature sensor
    // Usually, we expect something like "/sys/bus/w1/devices/28-*/temperature"
    glob_t glob_result;
    CHECK_RC(glob(sysfs_device_pattern.c_str(), 0 /*flags*/, nullptr /*errfunc*/, &glob_result), "Could not search for 1-wire temperature sensors");
    CleanupHelper call_globfree([&glob_result]()
    {
        // glob_result may itself be on the stack, but glob itself allocates things that need to be deleted after the fact.
        globfree(&glob_result);
    });

    if (glob_result.gl_pathc == 0)
    {
        throw std::logic_error("Could not find any compatible 1-wire temperature sensors at the provided path. Did you enable the 1-wire bus?");
    }
    else if (glob_result.gl_pathc > 1)
    {
        std::cerr << "More than one 1-wire temperature sensor found, using the first one" << std::endl;
    }

    sysfs_path_ = glob_result.gl_pathv[0];

    std::cerr << "Configuring a 1-wire temperature sensor at " << sysfs_path_ << std::endl;
}

std::optional<float> W1TemperatureSensor::read_temp_kelvin()
{
    std::ifstream sysfs_file(sysfs_path_);
    int temp_celsius_x1000;
    if (sysfs_file >> temp_celsius_x1000)
    {
        return (float) temp_celsius_x1000 / 1000 + ZERO_CELSIUS;
    }
    else
    {
        return std::nullopt;
    }
}

// Thermistor voltage divider connected to an ADC.
// Implentation adapted from https://learn.adafruit.com/thermistor/using-a-thermistor
class Ads1115TemperatureSensor : public ITemperatureSensor
{
public:
    Ads1115TemperatureSensor(const boost::property_tree::ptree &parsed);

    virtual std::optional<float> read_temp_kelvin();

private:
    constexpr static float DEFAULT_DIVIDER_RESISTOR_OHMS = 10000;
    constexpr static float DEFAULT_ADS_VDD = 3.3;

    int i2c_handle_;
    float divider_resistor_ohms_;
    float ads_vdd_;

    float known_temperature_kelvin_;
    float beta_value_;
    float resistance_at_known_temperature_ohms_;
};

Ads1115TemperatureSensor::Ads1115TemperatureSensor(const boost::property_tree::ptree &parsed)
    : i2c_handle_(PI_NO_HANDLE)
{
    constexpr static char CONFIG_REGISTER = 1;
    // Continuous conversion mode with -/+ 4.096V full scale range and MUX 100 (ground ref)
    constexpr static char CONFIG_DATA[] = {CONFIG_REGISTER, 0b11000010, 0b10000011};

    unsigned int i2c_bus = parsed.get<uint8_t>("i2c_bus");
    unsigned int i2c_address = parsed.get<uint8_t>("i2c_address");

    auto parsed_divider_resistor_ohms = parsed.get_optional<float>("divider_resistor_ohms");
    divider_resistor_ohms_ = (parsed_divider_resistor_ohms.is_initialized()) ? *parsed_divider_resistor_ohms : DEFAULT_DIVIDER_RESISTOR_OHMS;

    auto parsed_ads_vdd = parsed.get_optional<float>("ads_vdd");
    ads_vdd_ = (parsed_ads_vdd.is_initialized()) ? *parsed_ads_vdd : DEFAULT_ADS_VDD;

    known_temperature_kelvin_ = parsed.get<float>("known_temperature_kelvin");
    beta_value_ = parsed.get<float>("beta_value");
    resistance_at_known_temperature_ohms_ = parsed.get<float>("resistance_at_known_temperature_ohms");

    std::cerr << "Configuring a I2C-based temperature sensor on bus " << i2c_bus << " and address " << i2c_address << std::endl;

    i2c_handle_ = i2cOpen(i2c_bus, i2c_address, 0 /*i2cFlags*/);
    CHECK_RC(i2c_handle_, "Could not initialize I2C -- is it enabled?");

    CHECK_RC(
        // const_cast: I'm unsure why this only accepts non-const char* -- it never writes to it anyway
        i2cWriteDevice(i2c_handle_, const_cast<char *>(CONFIG_DATA), sizeof(CONFIG_DATA)),
        "Could not configure ADC");
}

std::optional<float> Ads1115TemperatureSensor::read_temp_kelvin()
{
    constexpr static uint8_t CONVERSION_REGISTER = 0;
    // Configured above
    constexpr static float FULL_SCALE_VOLTAGE = 4.096;

    CHECK_RC(i2cWriteByte(i2c_handle_, CONVERSION_REGISTER), "Could not write to pointer register");

    std::array<char, 2> raw_adc_value;
    CHECK_RC(
        // reinterpret_cast: converting temperature value
        i2cReadDevice(i2c_handle_, raw_adc_value.data(), raw_adc_value.size()),
        "Could not read from ADC");
    int16_t adc_value = raw_adc_value[0] << 8 | raw_adc_value[1];
    std::cerr << "Raw temperature ADC output: " << adc_value << std::endl;

    // ADC reports 0 for 0V and 26385 for 3.3V
    float v_max_steps = (float) std::numeric_limits<int16_t>::max() * ads_vdd_ / FULL_SCALE_VOLTAGE;
    if (adc_value < 512 || adc_value > v_max_steps - 512)
    {
        // A value outside this range suggests that the sensor is disconnected.
        return std::nullopt;
    }

    float thermistor_resistance_ohms = divider_resistor_ohms_ / (v_max_steps / adc_value - 1);
    float inverse_temperature = 1.0 / known_temperature_kelvin_ + std::log(thermistor_resistance_ohms / resistance_at_known_temperature_ohms_) / beta_value_;
    return 1.0 / inverse_temperature;
}

TemperatureController::TemperatureController(StatusServer &status_server, std::vector<unsigned int> &&heater_gpio_pins, std::unique_ptr<ITemperatureSensor> &&temperature_sensor, float p, float i, float bias, unsigned short max_pwm)
    : status_server_(status_server)
    , heater_gpio_pins_(std::move(heater_gpio_pins))
    , temperature_sensor_(std::move(temperature_sensor))
    , control_p_(p)
    , control_i_(i)
    , control_bias_(bias)
    , heater_max_duty_cycle_(max_pwm)
{
    for (unsigned int pin : heater_gpio_pins_)
    {
        CHECK_RC(gpioSetMode(pin, PI_OUTPUT), "Could not set up heater control");
        CHECK_RC(gpioPWM(pin, 0), "Could not initialize heater PWM");
        CHECK_RC(gpioSetPWMrange(pin, PWM_MAX), "Could not set heater PWM range");
    }

    control_thread_ = std::thread(&TemperatureController::control_thread_loop, this);
}

constexpr static std::chrono::duration<size_t> TEMPERATURE_CONTROL_PERIOD(1);
constexpr static float TEMPERATURE_EPSILON = 0.2;

std::function<bool()> DEFAULT_INTERRUPT_REQUESTED = []()
{
    return false;
};

void TemperatureController::set_temp_kelvin_for(float temp_kelvin, std::chrono::duration<size_t> wait_time, std::chrono::duration<size_t> hold_time, std::function<bool()> &interrupt_requested)
{
    target_temp_kelvin_ = temp_kelvin;
    auto temperature_deadline = std::chrono::steady_clock::now() + wait_time;
    while (std::abs(last_temp_kelvin_ - temp_kelvin) > TEMPERATURE_EPSILON)
    {
        if (std::chrono::steady_clock::now() > temperature_deadline || interrupt_requested())
        {
            disable();
            throw std::logic_error("Did not reach target temperature in time");
        }

        std::this_thread::sleep_for(TEMPERATURE_CONTROL_PERIOD);
    }

    hold_temp_until_ = std::chrono::steady_clock::now() + hold_time;
}

void TemperatureController::disable()
{
    target_temp_kelvin_ = 0;
    hold_temp_until_ = std::chrono::steady_clock::time_point();
}

void TemperatureController::set_heater_state(unsigned int duty_cycle)
{
    for (unsigned int pin : heater_gpio_pins_)
    {
        CHECK_RC(gpioPWM(pin, duty_cycle), "Could not change heater state");
    }
}

void TemperatureController::control_thread_loop()
{
    float roundoff = 0;
    auto last_read_time = std::chrono::steady_clock::time_point();
    while(true)
    {
        std::chrono::steady_clock::time_point hold_temp_until = hold_temp_until_.load();
        bool enabled = hold_temp_until != std::chrono::steady_clock::time_point();
        if (enabled && std::chrono::steady_clock::now() > hold_temp_until)
        {
            disable();
        }
        auto period_done = std::chrono::steady_clock::now() + TEMPERATURE_CONTROL_PERIOD;

        std::stringstream temperature_stringstream;
        std::optional<float> temp_kelvin_now = temperature_sensor_->read_temp_kelvin();
        if (temp_kelvin_now.has_value())
        {
            // PID mostly copied from Zion. I do not fully understand how it works, particularly the hacks wrt rounding and windup.
            auto read_time = std::chrono::steady_clock::now();
            std::chrono::duration<float> read_time_delta = read_time - last_read_time;
            float target_temp_kelvin = target_temp_kelvin_.load();
            float temp_error = target_temp_kelvin - *temp_kelvin_now;

            float pid_value = control_bias_
                + control_p_ * temp_error
                // Hack to avoid initial long windup
                + ((std::abs(temp_error) > M_PI) ? 0 : (control_i_ * temp_error * read_time_delta.count()));
            int duty_cycle = (int) pid_value + roundoff;

            if (duty_cycle >= 0 && duty_cycle <= (int) heater_max_duty_cycle_)
            {
                // duty_cycle in range, no need to adjust it.
                roundoff += pid_value - duty_cycle;
            }
            else if (duty_cycle < 0)
            {
                duty_cycle = 0;
                roundoff = 0;
            }
            else  // duty_cycle > heater_max_duty_cycle_
            {
                duty_cycle = (int) heater_max_duty_cycle_;
                roundoff = 0;
            }

            set_heater_state(duty_cycle);

            temperature_stringstream << std::fixed << std::setprecision(2) << *temp_kelvin_now - ZERO_CELSIUS << " ºC ";
            temperature_stringstream << "Heat " << duty_cycle / 10 << "%";
            last_temp_kelvin_ = *temp_kelvin_now;
            last_read_time = read_time;
        }
        else
        {
            temperature_stringstream << "Unknown temperature";
        }

        status_server_.write_message("temperature", temperature_stringstream.str());

        std::this_thread::sleep_until(period_done);
    }
}

const std::unordered_map<std::string, FocusPosition> FOCUS_POSITION_NAME_TO_ENUM {
    {"base", FocusPosition::BASE},
    {"red", FocusPosition::RED},
    {"orange", FocusPosition::ORANGE},
    {"green", FocusPosition::GREEN},
    {"blue", FocusPosition::BLUE}
};

const std::unordered_map<FocusPosition, std::string> FOCUS_POSITION_ENUM_TO_READABLE_NAME {
    {FocusPosition::BASE, "base"},
    {FocusPosition::RED, "red"},
    {FocusPosition::ORANGE, "orange"},
    {FocusPosition::GREEN, "green"},
    {FocusPosition::BLUE, "blue"}
};

const std::unordered_map<LensFilter, FocusPosition> FILTER_TO_FOCUS_ENUM {
    {LensFilter::ANY_FILTER, FocusPosition::BASE},
    {LensFilter::NO_FILTER, FocusPosition::BASE},
    {LensFilter::RED, FocusPosition::RED},
    {LensFilter::ORANGE, FocusPosition::ORANGE},
    {LensFilter::GREEN, FocusPosition::GREEN},
    {LensFilter::BLUE, FocusPosition::BLUE}
};

FocusController::FocusController(StatusServer &status_server, const boost::property_tree::ptree &parsed)
    : status_server_(status_server)
    , motor_(motor_factory("focus", status_server, parsed))
{
    // Load position configuration
    position_steps_.emplace(FocusPosition::BASE, 0);
    for (const auto &parsed_position : parsed.get_child("positions"))
    {
        FocusPosition position_id = FOCUS_POSITION_NAME_TO_ENUM.at(parsed_position.first);
        int position_step = parsed_position.second.get_value<int>();
        position_steps_.emplace(position_id, position_step);
    }

    motor_->initialize();
}

void FocusController::change_focus_async(const FocusPosition &position)
{
    motor_->move_async(position_steps_.at(position), FOCUS_POSITION_ENUM_TO_READABLE_NAME.at(position));
}

void FocusController::wait_for_focus_move()
{
    motor_->wait_for_move();
}

void FocusController::nudge_base_focus(int steps)
{
    motor_->nudge_base_position(steps);
}

constexpr static float DEFAULT_CONTROL_P = 2000;
constexpr static float DEFAULT_CONTROL_I = 0.2;
// constexpr static float DEFAULT_CONTROL_D = 0;  // Unused
constexpr static float DEFAULT_CONTROL_BIAS = 0;

GpioHal::GpioHal(boost::asio::io_context &io_context, StatusServer &status_server, const boost::property_tree::ptree &parsed)
    : io_context_(io_context)
    , status_server_(status_server)
{
    // Parse configuration
    assert(parsed.get<int>("schema_version") == SCHEMA_VERSION);

    for (const auto &parsed_led : parsed.get_child("leds"))
    {
        GpioLed led_id = GPIO_LED_NAME_TO_ENUM.at(parsed_led.first);
        leds_.emplace(led_id, parsed_led.second);
    }

    // Initialize GPIO
    CHECK_RC(gpioInitialise(), "Could not initialize GPIO");

    for (auto &led_entry : leds_)
    {
        led_entry.second.setup_gpio();
    }

    // Set up the heater
    // TODO: This configuration reading and setup should live in TemperatureController
    std::vector<unsigned int> heater_gpio_pins;
    for (const auto &parsed_pin : parsed.get_child("heater_gpio_pins"))
    {
        heater_gpio_pins.push_back(parsed_pin.second.get_value<unsigned int>());
    }
    unsigned short heater_max_pwm = PWM_MAX;
    auto parsed_heater_max_pwm = parsed.get_optional<unsigned int>("heater_max_pwm");
    if (parsed_heater_max_pwm.is_initialized())
    {
        heater_max_pwm = *parsed_heater_max_pwm;
    }
    float p, i, bias;
    auto control_pid_override = parsed.get_child_optional("control_pid_override");
    if (control_pid_override.is_initialized())
    {
        p = control_pid_override->get<float>("p");
        i = control_pid_override->get<float>("i");
        bias = control_pid_override->get<float>("bias");
    }
    else
    {
        p = DEFAULT_CONTROL_P;
        i = DEFAULT_CONTROL_I;
        bias = DEFAULT_CONTROL_BIAS;
    }
    std::cerr << "Configuring temperature controller with p=" << p << ", i=" << i << ", and bias=" << bias << std::endl;
    std::unique_ptr<ITemperatureSensor> temperature_sensor;
    auto temperature_sensor_w1_glob = parsed.get_optional<std::string>("temperature_sensor_w1_glob");
    auto temperature_sensor_i2c_adc = parsed.get_child_optional("temperature_sensor_i2c_adc");
    if (temperature_sensor_w1_glob.is_initialized())
    {
        temperature_sensor = std::make_unique<W1TemperatureSensor>(*temperature_sensor_w1_glob);
    }
    else if (temperature_sensor_i2c_adc.is_initialized())
    {
        temperature_sensor = std::make_unique<Ads1115TemperatureSensor>(*temperature_sensor_i2c_adc);
    }
    else
    {
        std::cerr << "No compatible temperature sensor configured, heater control will be disabled" << std::endl;
    }

    if (temperature_sensor)
    {
        temperature_controller_.emplace(status_server_, std::move(heater_gpio_pins), std::move(temperature_sensor), p, i, bias, heater_max_pwm);
    }

    auto filter_control_config = parsed.get_child_optional("filter_controller");
    if (filter_control_config.is_initialized())
    {
        for (const auto &parsed_filter : filter_control_config->get_child("positions"))
        {
            LensFilter filter_id = FILTER_NAME_TO_ENUM.at(parsed_filter.first);
            filters_.try_emplace(filter_id, parsed_filter.first, parsed_filter.second.get_value<int>());
        }
        filter_motor_ = motor_factory("filter", status_server_, *filter_control_config);
        reset_filter_wheel();
        std::cerr << "Automatic filter control configured" << std::endl;
    }
    else
    {
        std::cerr << "Manual filter control -- image sequences will prompt the client to confirm filter placement" << std::endl;
    }

    auto focus_control_config = parsed.get_child_optional("focus_controller");
    if (focus_control_config.is_initialized())
    {
        focus_controller_.emplace(status_server_, *focus_control_config);
        // Pre-moving the focus controller will do nothing since we have no way of knowing the stepper's absolute position.
        // Instead, we assume the current position *is* the base position.
        // It is up to the user to `nudge_base_position()` so that it is in focus prior to a sequencing run.
        // TODO: Add a limit switch so we can calibrate on first boot
        std::cerr << "Focus controller configured" << std::endl;
    }
}

std::unique_ptr<GpioHal> GpioHal::from_file(boost::asio::io_context &io_context, StatusServer &status_server, const std::string &filename)
{
    boost::property_tree::ptree hardware_configuration_json;
    boost::property_tree::read_json(filename, hardware_configuration_json);
    return std::make_unique<GpioHal>(io_context, status_server, hardware_configuration_json);
}

GpioHal::~GpioHal()
{
    gpioTerminate();
}

bool GpioHal::filter_control() const
{
    return bool(filter_motor_);
}

void GpioHal::change_filter_async(const LensFilter& filter, const boost::asio::ip::address &peer_address)
{
    if (filter == LensFilter::ANY_FILTER)
    {
        // Do nothing -- we don't care what filter is selected.
    }
    else if (filter_motor_)
    {
        filter_motor_->move_async(filters_.at(filter).position, FILTER_ENUM_TO_READABLE_NAME.at(filter));
        filter_move_wait_callback_ = [this, filter]()
        {
            filter_motor_->wait_for_move();
        };
    }
    else
    {
        // Prompt the user to move the filter manually.
        boost::property_tree::ptree request;
        request.add<std::string>("command", "confirmation_prompt");
        request.add<std::string>("text", "Ensure that the camera is armed and the " + filters_.at(filter).name + " filter is selected.");

        auto socket = send_command_message(io_context_, request, peer_address);

        // ... and set up a callback for the wait below to pick up.
        filter_move_wait_callback_ = [socket = std::move(socket)]() mutable
        {
            wait_for_command_response(std::move(socket));
        };
    }
}

void GpioHal::wait_for_filter_move()
{
    if (filter_move_wait_callback_)
    {
        CleanupHelper delete_callback([this]()
        {
            // The function *must* be deleted; otherwise, the socket will never be closed.
            filter_move_wait_callback_ = nullptr;
        });

        filter_move_wait_callback_();
    }
}

void GpioHal::reset_filter_wheel()
{
    if (filter_motor_)
    {
        filter_motor_->initialize();
        // Put the filter in a known position.
        // Since we have motor control, we will never prompt. It doesn't matter what IP address goes in here.
        change_filter_async(LensFilter::NO_FILTER, boost::asio::ip::address::from_string("0.0.0.0"));
        wait_for_filter_move();
    }
}

bool GpioHal::has_focus_controller() const
{
    return focus_controller_.has_value();
}

FocusController &GpioHal::focus_controller()
{
    return *focus_controller_;
}

std::future<void> GpioHal::flash_led_async(const GpioLed& led, unsigned int time_ms, unsigned short pwm)
{
    return leds_.at(led).flash_async(time_ms, pwm);
}

bool GpioHal::has_temperature_controller() const
{
    return temperature_controller_.has_value();
}

TemperatureController &GpioHal::temperature_controller()
{
    if (!temperature_controller_)
    {
        throw std::logic_error("No temperature controller configured");
    }

    return *temperature_controller_;
}

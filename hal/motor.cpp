#include <chrono>
#include <mutex>
#include <thread>

#include <pigpio.h>

#include "gui/status_server.hpp"
#include "utils/c_utils.hpp"

#include "motor.hpp"

std::string capitalize(const std::string &in)
{
    std::string out(in);
    if (out[0] >= 'a' && out[0] <= 'z')
    {
        out[0] -= 0x20;
    }
    return out;
}

class ServoMotor : public IMotor
{
public:
    ServoMotor(const std::string &name, StatusServer &status_server, const boost::property_tree::ptree &parsed);

    virtual void move_async(int position, const std::string &position_name);
    virtual void wait_for_move();
    virtual void nudge_base_position(int position_delta);

private:
    std::string name_;
    StatusServer &status_server_;

    unsigned int gpio_pin_;
    // TODO: This really should be an angular velocity instead.
    // Time between preset positions doesn't really make sense when we don't even have the positions here anymore.
    std::chrono::duration<unsigned int, std::milli> time_between_positions_;

    std::string requested_position_name_;
    int last_position_;
    std::chrono::steady_clock::time_point last_move_completed_;
};

ServoMotor::ServoMotor(const std::string &name, StatusServer &status_server, const boost::property_tree::ptree &parsed)
    : name_(name)
    , status_server_(status_server)
    , gpio_pin_(parsed.get<unsigned int>("gpio_pin"))
    , time_between_positions_(std::chrono::duration<unsigned int, std::milli>(parsed.get<unsigned int>("time_between_positions_ms")))
    , last_position_(std::numeric_limits<int>::min())
{
    CHECK_RC(gpioSetMode(gpio_pin_, PI_OUTPUT), "Could not set up filter servo");
}

void ServoMotor::move_async(int position, const std::string &position_name)
{
    requested_position_name_ = position_name;
    auto delay = (position == last_position_) ? std::chrono::duration<unsigned int, std::milli>() : time_between_positions_;
    status_server_.write_message(name_, capitalize(name_) + " moving to " + requested_position_name_ + " position");
    CHECK_RC(gpioServo(gpio_pin_, position), "Could not move servo");
    last_position_ = position;
    last_move_completed_ = std::chrono::steady_clock::now() + delay;
}

void ServoMotor::wait_for_move()
{
    std::this_thread::sleep_until(last_move_completed_);
    status_server_.write_message(name_, capitalize(name_) + " at " + requested_position_name_ + " position");
}

void ServoMotor::nudge_base_position(int position_delta)
{
    // TODO
}

class StepperMotor : public IMotor
{
public:
    StepperMotor(const std::string &name, StatusServer &status_server_, const boost::property_tree::ptree &parsed);
    virtual void initialize();

    virtual void move_async(int position, const std::string &position_name);
    virtual void wait_for_move();
    virtual void nudge_base_position(int position_delta);

private:
    std::string name_;
    StatusServer &status_server_;

    constexpr static unsigned short PWM_MAX = 1000;

    void control_thread_loop();

    // Assumes H-bridges for each winding and direction of a 2-winding bipolar motor: [A-, A+, B-, B+].
    constexpr static size_t NUM_STEPPER_GPIO_PINS = 4;
    std::array<unsigned int, NUM_STEPPER_GPIO_PINS> stepper_gpio_pins_;
    // Adapted from https://www.monolithicpower.com/bipolar-stepper-motors-part-i-control-modes
    constexpr static std::array<std::array<bool, NUM_STEPPER_GPIO_PINS>, 8> STEPPER_PATTERN
    {{
        // A-   A+     B-     B+
        {false, true , false, true },
        {false, false, false, true },
        {true , false, false, true },
        {true , false, false, false},
        {true , false, true , false},
        {false, false, true , false},
        {false, true , true , false},
        {false, true , false, false}
    }};
    unsigned int pwm_pin_;
    unsigned short pwm_standby_;
    unsigned short pwm_on_;

    // A step to move to that will always put the motor in a known reference position.
    int reset_step_;

    std::chrono::duration<size_t, std::micro> step_wait_;

    std::thread control_thread_;
    std::mutex control_mutex_;
    int base_step_;
    int current_absolute_step_;
    int requested_relative_step_;
    bool full_power_move_;

    std::string requested_position_name_;
};

StepperMotor::StepperMotor(const std::string &name, StatusServer &status_server, const boost::property_tree::ptree &parsed)
    : name_(name)
    , status_server_(status_server)
    , reset_step_(0)
    , base_step_(0)
    , current_absolute_step_(0)
    , requested_relative_step_(0)
    , full_power_move_(false)
{
    // Load GPIO pin configuration
    size_t pin_index = 0;
    for (const auto &parsed_pin : parsed.get_child("gpio_pins"))
    {
        stepper_gpio_pins_[pin_index] = parsed_pin.second.get_value<unsigned int>();
        ++pin_index;
    }
    if (pin_index != NUM_STEPPER_GPIO_PINS)
    {
        throw std::logic_error("Incorrect number of stepper motor pins");
    }
    pwm_pin_ = parsed.get<unsigned int>("pwm_pin");
    pwm_standby_ = parsed.get<unsigned short>("pwm_standby");
    pwm_on_ = parsed.get<unsigned short>("pwm_on");

    auto reset_step = parsed.get_optional<int>("reset_step");
    if (reset_step.is_initialized())
    {
        reset_step_ = *reset_step;
    }
    if (reset_step_ % STEPPER_PATTERN.size() != 0)
    {
        throw std::logic_error("reset_step must be a whole rotation sequence of the motor");
    }

    step_wait_ = std::chrono::duration<size_t, std::micro>(parsed.get<size_t>("step_time_us"));

    // Initialize GPIO
    for (unsigned int pin : stepper_gpio_pins_)
    {
        CHECK_RC(gpioSetMode(pin, PI_OUTPUT), "Could not set up stepper pin");
    }
    CHECK_RC(gpioSetMode(pwm_pin_, PI_OUTPUT), "Could not set up stepper pin");
    CHECK_RC(gpioSetPWMrange(pwm_pin_, PWM_MAX), "Could not set stepper PWM range");
    CHECK_RC(gpioPWM(pwm_pin_, pwm_standby_), "Could not put stepper motor in low power mode");

    control_thread_ = std::thread(&StepperMotor::control_thread_loop, this);
}

void StepperMotor::initialize()
{
    // Move to reset_step_...
    {
        std::lock_guard<std::mutex> control_lock(control_mutex_);
        full_power_move_ = false;
    }
    move_async(reset_step_, "initial");
    wait_for_move();

    // ... and convince ourselves that it is "home".
    {
        std::lock_guard<std::mutex> control_lock(control_mutex_);
        full_power_move_ = true;
        requested_relative_step_ = 0;
        current_absolute_step_ = 0;
    }

    wait_for_move();
}

void StepperMotor::move_async(int position, const std::string &position_name)
{
    requested_position_name_ = position_name;
    {
        std::lock_guard<std::mutex> control_lock(control_mutex_);
        requested_relative_step_ = position;
    }
}

void StepperMotor::wait_for_move()
{
    constexpr static unsigned int POLLING_STEPS = 10;

    int requested_step;
    int current_step;
    while (true)
    {
        {
            std::lock_guard<std::mutex> control_lock(control_mutex_);
            requested_step = base_step_ + requested_relative_step_;
            current_step = current_absolute_step_;
        }

        if (current_step == requested_step)
        {
            break;
        }

        status_server_.write_message(name_,
            capitalize(name_) + " at " + std::to_string(current_step) + ", moving to "
            + requested_position_name_ + " position (" + std::to_string(requested_step) + ")");

        std::this_thread::sleep_for(step_wait_ * POLLING_STEPS);
    }

    status_server_.write_message(name_,
        capitalize(name_) + " at " + requested_position_name_ + " position (" + std::to_string(requested_step) + ")");
}

void StepperMotor::nudge_base_position(int position_delta)
{
    std::lock_guard<std::mutex> control_lock(control_mutex_);
    base_step_ += position_delta;
}

void StepperMotor::control_thread_loop()
{
    // TODO: Constantly running the controller like this may use a lot of CPU
    while (true)
    {
        int requested_step;
        int current_step;
        bool full_power_move;
        {
            std::lock_guard<std::mutex> control_lock(control_mutex_);
            requested_step = base_step_ + requested_relative_step_;

            if (current_absolute_step_ < requested_step)
            {
                ++current_absolute_step_;
            }
            else if (current_absolute_step_ > requested_step)
            {
                --current_absolute_step_;
            }

            current_step = current_absolute_step_;
            full_power_move = full_power_move_;
        }

        unsigned short pwm_value = (full_power_move && current_step != requested_step) ? pwm_on_ : pwm_standby_;
        CHECK_RC(gpioPWM(pwm_pin_, pwm_value), "Could not set focus stepper power");

        size_t pattern_index = (STEPPER_PATTERN.size() + (current_step % STEPPER_PATTERN.size())) % STEPPER_PATTERN.size();
        for (size_t pin_index = 0; pin_index < NUM_STEPPER_GPIO_PINS; ++pin_index)
        {
            CHECK_RC(gpioWrite(stepper_gpio_pins_[pin_index], STEPPER_PATTERN[pattern_index][pin_index]), "Could not move focus stepper");
        }

        std::this_thread::sleep_for(step_wait_);
    }
}

std::unique_ptr<IMotor> motor_factory(const std::string &name, StatusServer &status_server, const boost::property_tree::ptree &parsed)
{
    auto stepper_config = parsed.get_child_optional("stepper");
    auto servo_config = parsed.get_child_optional("servo");
    if (stepper_config.is_initialized())
    {
        return std::make_unique<StepperMotor>(name, status_server, *stepper_config);
    }
    else if (servo_config.is_initialized())
    {
        return std::make_unique<ServoMotor>(name, status_server, *servo_config);
    }
    else
    {
        throw std::logic_error("Called motor_factory without a compatible motor");
    }
}

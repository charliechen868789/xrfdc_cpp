#include "gpio.hpp"
#include <iostream>
#include <thread>
#include <chrono>

namespace gpio {

Gpio::Gpio(int gpio_number)
    : gpio_number_(gpio_number)
    , enabled_(false)
{
}

Gpio::~Gpio() {
    if (enabled_) {
        disable();
    }
}

bool Gpio::enable() {
    if (enabled_) {
        return true;  // Already enabled
    }
    
    if (!write_to_sysfs(GPIO_EXPORT, std::to_string(gpio_number_))) {
        std::cerr << "Failed to export GPIO " << gpio_number_ << "\n";
        return false;
    }
    
    // Give the system time to create the GPIO files
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    enabled_ = true;
    return true;
}

bool Gpio::disable() {
    if (!enabled_) {
        return true;  // Already disabled
    }
    
    if (!write_to_sysfs(GPIO_UNEXPORT, std::to_string(gpio_number_))) {
        std::cerr << "Failed to unexport GPIO " << gpio_number_ << "\n";
        return false;
    }
    
    enabled_ = false;
    return true;
}

bool Gpio::set_direction(Direction dir) {
    std::string direction_path = get_gpio_path("direction");
    std::string direction_str = (dir == Direction::Output) ? "out" : "in";
    
    if (!write_to_sysfs(direction_path, direction_str)) {
        std::cerr << "Failed to set direction for GPIO " << gpio_number_ << "\n";
        return false;
    }
    
    return true;
}

bool Gpio::set_value(Value val) {
    std::string value_path = get_gpio_path("value");
    std::string value_str = (val == Value::High) ? "1" : "0";
    
    if (!write_to_sysfs(value_path, value_str)) {
        std::cerr << "Failed to set value for GPIO " << gpio_number_ << "\n";
        return false;
    }
    
    return true;
}

bool Gpio::get_value(Value& val) {
    std::string value_path = get_gpio_path("value");
    std::string value_str;
    
    if (!read_from_sysfs(value_path, value_str)) {
        std::cerr << "Failed to get value for GPIO " << gpio_number_ << "\n";
        return false;
    }
    
    val = (value_str[0] == '1') ? Value::High : Value::Low;
    return true;
}

std::string Gpio::get_gpio_path(const std::string& attribute) const {
    return std::string(GPIO_PATH) + "gpio" + std::to_string(gpio_number_) + "/" + attribute;
}

bool Gpio::write_to_sysfs(const std::string& path, const std::string& value) {
    std::ofstream file(path);
    if (!file.is_open()) {
        return false;
    }
    
    file << value;
    file.close();
    
    return !file.fail();
}

bool Gpio::read_from_sysfs(const std::string& path, std::string& value) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }
    
    std::getline(file, value);
    file.close();
    
    return !file.fail();
}

} // namespace gpio
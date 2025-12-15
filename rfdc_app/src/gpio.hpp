#pragma once

#include <string>
#include <cstdint>
#include <fstream>

namespace gpio {

/**
 * @brief Simple GPIO control class for Linux sysfs interface
 */
class Gpio {
public:
    enum class Direction {
        Input,
        Output
    };
    
    enum class Value {
        Low = 0,
        High = 1
    };
    
    /**
     * @brief Construct a GPIO object
     * @param gpio_number The GPIO number to control
     */
    explicit Gpio(int gpio_number);
    
    /**
     * @brief Destructor - unexports the GPIO
     */
    ~Gpio();
    
    // Disable copy
    Gpio(const Gpio&) = delete;
    Gpio& operator=(const Gpio&) = delete;
    
    /**
     * @brief Export the GPIO (make it available)
     * @return true on success, false on failure
     */
    bool enable();
    
    /**
     * @brief Unexport the GPIO (release it)
     * @return true on success, false on failure
     */
    bool disable();
    
    /**
     * @brief Set GPIO direction
     * @param dir Direction (Input or Output)
     * @return true on success, false on failure
     */
    bool set_direction(Direction dir);
    
    /**
     * @brief Set GPIO value (for output pins)
     * @param val Value to set (Low or High)
     * @return true on success, false on failure
     */
    bool set_value(Value val);
    
    /**
     * @brief Get GPIO value (for input pins)
     * @param val Reference to store the read value
     * @return true on success, false on failure
     */
    bool get_value(Value& val);
    
    /**
     * @brief Get the GPIO number
     * @return GPIO number
     */
    int number() const { return gpio_number_; }
    
private:
    int gpio_number_;
    bool enabled_;
    
    static constexpr const char* GPIO_PATH = "/sys/class/gpio/";
    static constexpr const char* GPIO_EXPORT = "/sys/class/gpio/export";
    static constexpr const char* GPIO_UNEXPORT = "/sys/class/gpio/unexport";
    
    std::string get_gpio_path(const std::string& attribute) const;
    bool write_to_sysfs(const std::string& path, const std::string& value);
    bool read_from_sysfs(const std::string& path, std::string& value);
};

} // namespace gpio
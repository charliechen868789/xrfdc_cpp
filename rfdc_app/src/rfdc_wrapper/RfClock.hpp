/******************************************************************************
* Copyright (C) 2024 Charlie
* Modern C++14 wrapper for Xilinx RF Clock (xrfclk.h) driver
* SPDX-License-Identifier: MIT
******************************************************************************/

#pragma once

// Include Xilinx headers directly - they have proper extern "C" guards
#include <xrfclk.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <array>
#include <vector>

namespace rfdc {
// Exception class for RF Clock errors
class RFClockException : public std::runtime_error {
public:
    explicit RFClockException(const std::string& msg)
        : std::runtime_error(msg), error_code_(XST_FAILURE) {}
    
    RFClockException(const std::string& msg, uint32_t code)
        : std::runtime_error(msg), error_code_(code) {}
    
    uint32_t error_code() const noexcept { return error_code_; }
    
private:
    uint32_t error_code_;
};

// Type-safe chip IDs
enum class RFClockChip : uint32_t {
    LMX2594_1 = RFCLK_LMX2594_1,  // ADC sample clock
    LMX2594_2 = RFCLK_LMX2594_2,  // DAC sample clock
    LMK04828  = RFCLK_LMK,        // Clock distribution
#ifdef XPS_BOARD_ZCU111
    LMX2594_3 = RFCLK_LMX2594_3   // Additional LMX for ZCU111
#endif
};

// Board types
enum class BoardType {
    ZCU216,
    ZCU111
};

// LMK output port control
enum class LMKPort : uint32_t {
    Port0 = 0,
    Port1 = 1,
    Port2 = 2,
    Port3 = 3,
    Port4 = 4,
    Port5 = 5,
    Port6 = 6,
    Port7 = 7,
    Port8 = 8,
    Port9 = 9,
    Port10 = 10,
    Port11 = 11,
    Port12 = 12,
    Port13 = 13
};

// Port state
enum class PortState : uint32_t {
    Disable = 0,
    Enable = 1
};

/**
 * @brief RAII wrapper for Xilinx RF Clock configuration
 * 
 * Manages LMX2594 (sample clock PLLs) and LMK04828 (clock distribution)
 * for ZCU216/ZCU111 boards.
 */
class RFClock {
public:
    /**
     * @brief Initialize RF Clock system
     * @param gpio_id GPIO device ID (default 0 for Linux)
     * @throws RFClockException on initialization failure
     */
#if defined XPS_BOARD_ZCU111
    explicit RFClock();
#elif defined __BAREMETAL__
    explicit RFClock(uint32_t gpio_mux_base_addr);
#else
    explicit RFClock(int gpio_id = 0);
#endif
    static constexpr uint32_t DEFAULT_RFCLK_LMK_CONFIG =         0;
    /**
     * @brief Cleanup and close RF Clock system
     */
    ~RFClock();
    
    // Disable copy
    RFClock(const RFClock&) = delete;
    RFClock& operator=(const RFClock&) = delete;
    
    // Enable move
    RFClock(RFClock&&) noexcept = default;
    RFClock& operator=(RFClock&&) noexcept = default;
    
    /**
     * @brief Write configuration register to a chip
     * @param chip Target chip ID
     * @param data Data to write
     */
    void write_reg(RFClockChip chip, uint32_t data);
    
    /**
     * @brief Read configuration register from a chip
     * @param chip Target chip ID
     * @return Read data value
     */
    uint32_t read_reg(RFClockChip chip);
    
    /**
     * @brief Reset a clock chip
     * @param chip Chip to reset
     */
    void reset_chip(RFClockChip chip);
    
    /**
     * @brief Configure a chip using a predefined configuration ID
     * @param chip Target chip
     * @param config_id Configuration ID (0 to N-1)
     */
    void set_config(RFClockChip chip, uint32_t config_id);
    
    /**
     * @brief Configure a chip with custom configuration data
     * @param chip Target chip
     * @param config_data Configuration data array
     * @param length Length of configuration data
     */
    void set_config_custom(RFClockChip chip, const uint32_t* config_data, uint32_t length);
    
    /**
     * @brief Get current configuration from a chip
     * @param chip Target chip
     * @param config_data Buffer to store configuration (must be large enough)
     */
    void get_config(RFClockChip chip, uint32_t* config_data);
    
    /**
     * @brief Configure all chips at once
     * @param lmk_config_id LMK04828 configuration ID
     * @param lmx1_config_id LMX2594_1 configuration ID
     * @param lmx2_config_id LMX2594_2 configuration ID
     */
    void set_all_configs(uint32_t lmk_config_id, uint32_t lmx1_config_id,
                        uint32_t lmx2_config_id);
    
    /**
     * @brief Control LMK output port enable/disable
     * @param port Target port
     * @param state Enable or disable
     */
    void control_lmk_port(LMKPort port, PortState state);
    
    /**
     * @brief Configure LMK output divider and MUX settings
     * @param port Target port
     * @param dclk_div DCLK output divider value
     * @param dclk_mux DCLK MUX selection
     * @param sdclk_mux SDCLK MUX selection
     * @param sysref_div SYSREF divider value
     */
    void config_lmk_output(LMKPort port, uint32_t dclk_div, uint32_t dclk_mux,
                          uint32_t sdclk_mux, uint32_t sysref_div);
    
    /**
     * @brief Get driver version string
     */
    static std::string get_version();
    
    /**
     * @brief Get board type
     */
    static BoardType get_board_type();
    
    /**
     * @brief Get number of available LMK frequency configurations
     */
    static constexpr uint32_t get_lmk_freq_count() {
        return LMK_FREQ_NUM;
    }
    
    /**
     * @brief Get number of available LMX ADC configurations
     */
    static constexpr uint32_t get_lmx_adc_count() {
        return LMX_ADC_NUM;
    }
    
    /**
     * @brief Get number of available LMX DAC configurations
     */
    static constexpr uint32_t get_lmx_dac_count() {
        return LMX_DAC_NUM;
    }

private:
    bool initialized_;
    
    // Helper to check status and throw on error
    void check_status(uint32_t status, const std::string& operation) const 
    {
        if (status != XST_SUCCESS) {
            throw RFClockException(
                operation + " failed with status: " + std::to_string(status),
                status
            );
        }
    }
    
    // Convert enum to underlying type
    static constexpr uint32_t to_underlying(RFClockChip chip) {
        return static_cast<uint32_t>(chip);
    }
    
    static constexpr uint32_t to_underlying(LMKPort port) {
        return static_cast<uint32_t>(port);
    }
    
    static constexpr uint32_t to_underlying(PortState state) {
        return static_cast<uint32_t>(state);
    }
};

// Convenient frequency configuration presets for common use cases
struct ClockPreset {
    const char* name;
    uint32_t lmk_config;
    uint32_t lmx_adc_config;
    uint32_t lmx_dac_config;
    double ref_freq_mhz;
    double adc_sample_rate_msps;
    double dac_sample_rate_msps;
};

// Common clock presets for ZCU216
namespace ClockPresets {
    // 4.9152 GSPS for both ADC and DAC
    const ClockPreset MAX_RATE = {
        "Maximum Sample Rate",
        0,    // LMK config ID
        0,    // LMX ADC config ID
        0,    // LMX DAC config ID
        245.76,
        4915.2,
        4915.2
    };
    
    // 2.4576 GSPS for both ADC and DAC
    const ClockPreset MID_RATE = {
        "Medium Sample Rate",
        0,
        1,
        1,
        245.76,
        2457.6,
        2457.6
    };
}

} // namespace rfdc
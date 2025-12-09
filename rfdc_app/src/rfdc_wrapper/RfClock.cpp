/******************************************************************************
* Copyright (C) 2024 Charlie
* Modern C++14 wrapper for Xilinx RF Clock (xrfclk.h) driver
* SPDX-License-Identifier: MIT
******************************************************************************/

#include "RfClock.hpp"
#include <sstream>
#include <iostream>
namespace rfdc 
{

    // Helper for formatting error messages
    template<typename... Args>
    std::string format_string(Args&&... args) {
        std::ostringstream oss;
        using expander = int[];
        (void)expander{0, ((oss << std::forward<Args>(args)), 0)...};
        return oss.str();
    }

    // Constructor implementations based on platform
    #if defined XPS_BOARD_ZCU111
    RFClock::RFClock() : initialized_(false) {
        auto status = XRFClk_Init();
        check_status(status, "XRFClk_Init");
        initialized_ = true;
    }
    #elif defined __BAREMETAL__
    RFClock::RFClock(uint32_t gpio_mux_base_addr) : initialized_(false) {
        auto status = XRFClk_Init(gpio_mux_base_addr);
        check_status(status, "XRFClk_Init");
        initialized_ = true;
    }
    #else
    //use this one for zcu216
    RFClock::RFClock(int gpio_id) : initialized_(false) {
        auto status = XRFClk_Init(gpio_id);
        check_status(status, "XRFClk_Init");
        initialized_ = true;
        // Configure LMK with default config
        if (XRFClk_SetConfigOnOneChipFromConfigId(
                RFCLK_LMK,
                0) != XST_SUCCESS)
        {
            std::cout << "WARNING: LMK config failed\n" << std::flush;
        } else {
            std::cout << "RFCLK Init Done\n" << std::flush;
        }
    }
    #endif

    RFClock::~RFClock() {
        if (initialized_) {
            XRFClk_Close();
            initialized_ = false;
        }
    }

    void RFClock::write_reg(RFClockChip chip, uint32_t data) {
        auto status = XRFClk_WriteReg(to_underlying(chip), data);
        check_status(status, format_string("WriteReg to chip ", to_underlying(chip)));
    }

    uint32_t RFClock::read_reg(RFClockChip chip) {
        uint32_t data = 0;
        auto status = XRFClk_ReadReg(to_underlying(chip), &data);
        check_status(status, format_string("ReadReg from chip ", to_underlying(chip)));
        return data;
    }

    void RFClock::reset_chip(RFClockChip chip) {
        auto status = XRFClk_ResetChip(to_underlying(chip));
        check_status(status, format_string("ResetChip ", to_underlying(chip)));
    }

    void RFClock::set_config(RFClockChip chip, uint32_t config_id) {
        auto status = XRFClk_SetConfigOnOneChipFromConfigId(
            to_underlying(chip),
            config_id
        );
        check_status(status, format_string("SetConfig chip ", to_underlying(chip),
                                        " config ", config_id));
    }

    void RFClock::set_config_custom(RFClockChip chip, const uint32_t* config_data, 
                                    uint32_t length) {
        auto status = XRFClk_SetConfigOnOneChip(
            to_underlying(chip),
            const_cast<uint32_t*>(config_data),
            length
        );
        check_status(status, format_string("SetConfigCustom chip ", to_underlying(chip)));
    }

    void RFClock::get_config(RFClockChip chip, uint32_t* config_data) {
        auto status = XRFClk_GetConfigFromOneChip(
            to_underlying(chip),
            config_data
        );
        check_status(status, format_string("GetConfig chip ", to_underlying(chip)));
    }

    #ifdef XPS_BOARD_ZCU111
    void RFClock::set_all_configs(uint32_t lmk_config_id, uint32_t lmx1_config_id,
                                uint32_t lmx2_config_id, uint32_t lmx3_config_id) {
        auto status = XRFClk_SetConfigOnAllChipsFromConfigId(
            lmk_config_id,
            lmx1_config_id,
            lmx2_config_id,
            lmx3_config_id
        );
        check_status(status, "SetConfigOnAllChips");
    }
    #else
    void RFClock::set_all_configs(uint32_t lmk_config_id, uint32_t lmx1_config_id,
                                uint32_t lmx2_config_id) {
        auto status = XRFClk_SetConfigOnAllChipsFromConfigId(
            lmk_config_id,
            lmx1_config_id,
            lmx2_config_id
        );
        check_status(status, "SetConfigOnAllChips");
    }
    #endif

    void RFClock::control_lmk_port(LMKPort port, PortState state) {
        auto status = XRFClk_ControlOutputPortLMK(
            to_underlying(port),
            to_underlying(state)
        );
        check_status(status, format_string("ControlLMKPort ", to_underlying(port)));
    }

    void RFClock::config_lmk_output(LMKPort port, uint32_t dclk_div, uint32_t dclk_mux,
                                    uint32_t sdclk_mux, uint32_t sysref_div) {
        auto status = XRFClk_ConfigOutputDividerAndMUXOnLMK(
            to_underlying(port),
            dclk_div,
            dclk_mux,
            sdclk_mux,
            sysref_div
        );
        check_status(status, format_string("ConfigLMKOutput port ", to_underlying(port)));
    }

    std::string RFClock::get_version() {
        return std::string(RFCLK_VERSION);
    }

    BoardType RFClock::get_board_type() {
    #ifdef XPS_BOARD_ZCU111
        return BoardType::ZCU111;
    #else
        return BoardType::ZCU216;
    #endif
    }

} // namespace rfdc
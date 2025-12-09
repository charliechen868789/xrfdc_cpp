#pragma once

#include <string>
#include <memory>
#include "rfdc_wrapper/RfDc.hpp"
#include "rfdc_wrapper/RfClock.hpp"

class RfDcApp
{
public:
    explicit RfDcApp(const std::string& name = "RFDC App");
    ~RfDcApp() = default;
    
    // Disable copy
    RfDcApp(const RfDcApp&) = delete;
    RfDcApp& operator=(const RfDcApp&) = delete;
    
    // Main entry point for your RF initialization & test code
    void run();

private:
    std::string name_;
    
    // Internal subsystem wrappers
    std::unique_ptr<rfdc::RFClock> clock_;
    std::unique_ptr<rfdc::RFDC> rfdc_;
    
    // Initialization methods
    void initialize_clocks();
    void initialize_rfdc();
    void configure_dac_tiles();
    void configure_adc_tiles();
    void verify_configuration();
    
    // Test methods
    void run_loopback_test();
    void display_status();
};
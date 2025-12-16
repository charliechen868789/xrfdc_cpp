#pragma once
#include <string>
#include <memory>
#include <fstream>
#include <array>
#include <vector>
#include <random>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "rfdc_wrapper/RfDc.hpp"
#include "rfdc_wrapper/RfClock.hpp"
#include "gpio.hpp"
#include "LocalMem.hpp"
#include "ClockWizard.hpp"

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
    
    // Public memory initialization (matching RFTool API)
    int init_mem();
    int deinit_mem();

private:
    // Hardware base addresses
    static constexpr uint32_t ADC_SINK_I_BASEADDR     = 0xB0400000;
    static constexpr uint32_t DAC_SOURCE_I_BASEADDR   = 0xB0000000;
    static constexpr uint32_t CLK_WIZ_ADC0_BASEADDR   = 0xB4C40000;
    static constexpr uint32_t CLK_WIZ_ADC1_BASEADDR   = 0xB4C50000;
    static constexpr uint32_t CLK_WIZ_ADC2_BASEADDR   = 0xB4C60000;
    static constexpr uint32_t CLK_WIZ_ADC3_BASEADDR   = 0xB4C70000;
    static constexpr uint32_t CLK_WIZ_DAC0_BASEADDR   = 0xB4C00000;
    static constexpr uint32_t CLK_WIZ_DAC1_BASEADDR   = 0xB4C10000;
    static constexpr uint32_t CLK_WIZ_DAC2_BASEADDR   = 0xB4C20000;
    static constexpr uint32_t CLK_WIZ_DAC3_BASEADDR   = 0xB4C30000;
    
    std::array<uint32_t, 4> adc_clk_wiz_addrs_;
    std::array<uint32_t, 4> dac_clk_wiz_addrs_;

public:
    static constexpr uint16_t RFDC_DEVICE_ID = 0;

private:
    // GPIO pin definitions
    static const int MAX_DAC_PER_TILE = 4;
    static const int MAX_DAC_TILE = 4;
    static const int MAX_ADC_TILE = 4;
    
    static const int dac_userselect_gpio[MAX_DAC_PER_TILE * MAX_DAC_TILE];
    static const int dac_mts_clk_en[MAX_DAC_TILE];
    static const int adc_mts_clk_en[MAX_ADC_TILE];
    static const int adc_axiswitchrst;

    // GPIO objects
    std::vector<std::unique_ptr<gpio::Gpio>> dac_userselect_gpios_;
    std::vector<std::unique_ptr<gpio::Gpio>> dac_mts_clk_gpios_;
    std::vector<std::unique_ptr<gpio::Gpio>> adc_mts_clk_gpios_;
    std::unique_ptr<gpio::Gpio> adc_axiswitch_reset_gpio_;

    // UIO memory structures (matching RFTool)
    struct RfSocInfo {
        signed char *map_dac[16];
        signed char *map_adc[16];
        int fd_dac[16];
        int fd_adc[16];
        int fd;  // /dev/mem file descriptor
        unsigned int mem_type_dac;
        unsigned int mem_type_adc;
    };
    
    struct AdcSamples {
        std::vector<int16_t> I;
        std::vector<int16_t> Q;
        bool is_iq = false;

        // Compatibility helpers
        size_t size() const { return I.size(); }
        int16_t operator[](size_t i) const { return I[i]; }
        auto begin() const { return I.begin(); }
        auto end() const { return I.end(); }
    };

    RfSocInfo info_;
    
    // Memory size constants
    static constexpr size_t DAC_MAP_SZ = (1024 * 1024 * 1024) / 8;  // 128MB
    static constexpr size_t ADC_MAP_SZ = (1024 * 1024 * 1024) / 8;  // 128MB
    static constexpr size_t FIFO_SIZE = 16 * 1024 * 2;
    static constexpr size_t ADC_DAC_SZ_ALIGNMENT = 32;
    
    std::string name_;
    
    // Internal subsystem wrappers
    std::unique_ptr<rfdc::RFClock> clock_;
    std::unique_ptr<rfdc::RFDC> rfdc_;
    std::unique_ptr<local_mem::LocalMem> local_mem_; 
    std::unique_ptr<clock_wizard::ClockWizard> clock_wiz_;
    
    // Initialization methods
    void initialize_clocks();
    void initialize_rfdc();
    void initialize_memory_mapping();
    void initialize_mmcm_adc();
    void initialize_mmcm_dac();
    void configure_dac_tiles();
    void configure_adc_tiles();
    void verify_configuration();
    
    // GPIO methods
    void init_gpio();
    void deinit_gpio();

    
    // Data transfer using UIO (matching RFTool pattern)
    int write_data_to_memory_bram(uint32_t block_id, int tile_id, 
                                   uint32_t size, const std::vector<int16_t>& samples);
    int read_data_from_memory_bram(uint32_t block_id, int tile_id,
                                    uint32_t size, std::vector<int16_t>& samples);
    
    int read_adc_bram_rftool_style(
        uint32_t tile_id,
        uint32_t block_id,
        uint32_t size_bytes,
        AdcSamples& out  // empty in REAL mode
    );
    std::vector<int16_t> read_adc_samples(uint32_t tile, uint32_t block,
                                               size_t num_samples);
    // FIFO control
    int change_fifo_stat(int fifo_id, int tile_id, int stat);
    
    // Trigger functions
    void local_mem_trigger(rfdc::TileType type, uint32_t tile_id,
                          uint32_t num_samples, uint32_t channel_mask);
    void set_local_mem_sample(rfdc::TileType type, uint32_t tile_id,
                             uint32_t block_id, uint32_t num_samples);
    
    // Helper to write to sysfs
    int write_to_file(const std::string& path, const std::string& value);
    
    // Test methods
    void run_loopback_test();
    void display_status();
    void save_samples_to_csv(const std::vector<int16_t>& samples,
                            double sample_rate_hz,
                            const std::string& filename,
                            const std::string& metadata = "");
    void save_samples_to_csv(
        const std::vector<int16_t>& I,
        const std::vector<int16_t>& Q,
        double fabric_sample_rate_hz,
        const std::string& filename,
        const std::string& metadata
    );  
    // Data transfer helper methods
    void write_dac_samples(uint32_t tile, uint32_t block,
                          const std::vector<int16_t>& samples);
    //std::vector<int16_t> read_adc_samples(uint32_t tile, uint32_t block,
    //                                      size_t num_samples);
    //AdcSamples read_adc_samples(
    //    uint32_t tile,
    //    uint32_t block,
    //    size_t num_samples
    //);
    // Generate sine wave accounting for DAC interpolation
    std::vector<int16_t> generate_sine_wave(
            double frequency_hz,
            double dac_pll_rate_hz,
            uint32_t dac_interpolation,
            size_t num_samples,
            int16_t amplitude,
            double noise_dbfs /* = -80.0 */
    );
    std::vector<int16_t> generate_dc_offset(int16_t value, size_t num_samples);
};
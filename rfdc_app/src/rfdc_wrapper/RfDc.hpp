/******************************************************************************
* Copyright (C) 2024 Charlie
* Modern C++14 wrapper for Xilinx RFDC driver v2024.1
* SPDX-License-Identifier: MIT
******************************************************************************/

#pragma once

// Include Xilinx headers directly - they have proper extern "C" guards
#include <xrfdc.h>

// Include libmetal headers for initialization
#include <metal/sys.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <memory>
#include <array>
#include <cmath>
namespace rfdc {

// Modern type aliases
using TileId = uint32_t;
using BlockId = uint32_t;

// Type-safe enumerations
enum class TileType : uint32_t {
    ADC = XRFDC_ADC_TILE,
    DAC = XRFDC_DAC_TILE
};

enum class MixerMode : uint32_t {
    Off = XRFDC_MIXER_MODE_OFF,
    C2C = XRFDC_MIXER_MODE_C2C,
    C2R = XRFDC_MIXER_MODE_C2R,
    R2C = XRFDC_MIXER_MODE_R2C,
    R2R = XRFDC_MIXER_MODE_R2R
};

enum class MixerType : uint32_t {
    Off = XRFDC_MIXER_TYPE_OFF,
    Coarse = XRFDC_MIXER_TYPE_COARSE,
    Fine = XRFDC_MIXER_TYPE_FINE,
    Disabled = XRFDC_MIXER_TYPE_DISABLED
};

enum class EventSource : uint32_t {
    Immediate = XRFDC_EVNT_SRC_IMMEDIATE,
    Slice = XRFDC_EVNT_SRC_SLICE,
    Tile = XRFDC_EVNT_SRC_TILE,
    SysRef = XRFDC_EVNT_SRC_SYSREF,
    Marker = XRFDC_EVNT_SRC_MARKER,
    PL = XRFDC_EVNT_SRC_PL
};

enum class DataType : uint32_t {
    Real = XRFDC_DATA_TYPE_REAL,
    IQ = XRFDC_DATA_TYPE_IQ
};

enum class NyquistZone : uint32_t {
    Zone1 = 1,
    Zone2 = 2
};

enum class CalibrationMode : uint8_t {
    Mode1 = 1,
    Mode2 = 2
};

enum class ThresholdMode : uint32_t {
    Off = XRFDC_TRSHD_OFF,
    StickyOver = XRFDC_TRSHD_STICKY_OVER,
    StickyUnder = XRFDC_TRSHD_STICKY_UNDER,
    Hysteresis = XRFDC_TRSHD_HYSTERISIS
};

enum class DecoderMode : uint32_t {
    Max = 0
};

// Exception class for RFDC errors
class RFDCException : public std::runtime_error {
public:
    explicit RFDCException(const std::string& msg) 
        : std::runtime_error(msg), error_code_(XRFDC_FAILURE) {}
    
    RFDCException(const std::string& msg, uint32_t code)
        : std::runtime_error(msg), error_code_(code) {}
    
    uint32_t error_code() const noexcept { return error_code_; }
    
private:
    uint32_t error_code_;
};

// RAII wrapper for mixer settings
class MixerSettings {
public:
    MixerSettings() = default;
    
    MixerSettings(double freq, double phase_offset, EventSource event_src,
                  uint32_t coarse_mix_freq, MixerMode mode, 
                  uint8_t fine_mixer_scale, MixerType type) {
        settings_.Freq = freq;
        settings_.PhaseOffset = phase_offset;
        settings_.EventSource = static_cast<uint32_t>(event_src);
        settings_.CoarseMixFreq = coarse_mix_freq;
        settings_.MixerMode = static_cast<uint32_t>(mode);
        settings_.FineMixerScale = fine_mixer_scale;
        settings_.MixerType = static_cast<uint8_t>(type);
    }
    
    XRFdc_Mixer_Settings* get() { return &settings_; }
    const XRFdc_Mixer_Settings* get() const { return &settings_; }
    
    // Accessors with modern types
    double frequency() const { return settings_.Freq; }
    void set_frequency(double freq) { settings_.Freq = freq; }
    
    double phase_offset() const { return settings_.PhaseOffset; }
    void set_phase_offset(double offset) { settings_.PhaseOffset = offset; }
    
    MixerMode mode() const { return static_cast<MixerMode>(settings_.MixerMode); }
    void set_mode(MixerMode mode) { settings_.MixerMode = static_cast<uint32_t>(mode); }
    
    MixerType type() const { return static_cast<MixerType>(settings_.MixerType); }
    void set_type(MixerType type) { settings_.MixerType = static_cast<uint8_t>(type); }

    uint32_t coarse_mix_freq() const { return settings_.CoarseMixFreq; }
    void set_coarse_mix_freq(uint32_t freq) { settings_.CoarseMixFreq = freq; }
    
private:
    XRFdc_Mixer_Settings settings_{};
};

// RAII wrapper for QMC settings
class QMCSettings {
public:
    QMCSettings() = default;
    
    QMCSettings(bool enable_phase, bool enable_gain, 
                double gain_correction, double phase_correction,
                int32_t offset_correction, EventSource event_src) {
        settings_.EnablePhase = enable_phase ? 1 : 0;
        settings_.EnableGain = enable_gain ? 1 : 0;
        settings_.GainCorrectionFactor = gain_correction;
        settings_.PhaseCorrectionFactor = phase_correction;
        settings_.OffsetCorrectionFactor = offset_correction;
        settings_.EventSource = static_cast<uint32_t>(event_src);
    }
    
    XRFdc_QMC_Settings* get() { return &settings_; }
    const XRFdc_QMC_Settings* get() const { return &settings_; }
    
    bool phase_enabled() const { return settings_.EnablePhase != 0; }
    bool gain_enabled() const { return settings_.EnableGain != 0; }
    double gain_correction() const { return settings_.GainCorrectionFactor; }
    double phase_correction() const { return settings_.PhaseCorrectionFactor; }
    
private:
    XRFdc_QMC_Settings settings_{};
};

// RAII wrapper for threshold settings
class ThresholdSettings {
public:
    ThresholdSettings() = default;
    
    XRFdc_Threshold_Settings* get() { return &settings_; }
    const XRFdc_Threshold_Settings* get() const { return &settings_; }
    
    void set_mode(uint32_t threshold_idx, ThresholdMode mode) {
        if (threshold_idx > 1) {
            throw RFDCException("Invalid threshold index (must be 0 or 1)");
        }
        settings_.ThresholdMode[threshold_idx] = static_cast<uint32_t>(mode);
    }
    
    void set_threshold_values(uint32_t threshold_idx, uint32_t avg_val, 
                             uint32_t under_val, uint32_t over_val) {
        if (threshold_idx > 1) {
            throw RFDCException("Invalid threshold index (must be 0 or 1)");
        }
        settings_.ThresholdAvgVal[threshold_idx] = avg_val;
        settings_.ThresholdUnderVal[threshold_idx] = under_val;
        settings_.ThresholdOverVal[threshold_idx] = over_val;
    }
    
private:
    XRFdc_Threshold_Settings settings_{};
};

// RAII wrapper for PLL settings
class PLLSettings {
public:
    PLLSettings() = default;
    
    XRFdc_PLL_Settings* get() { return &settings_; }
    const XRFdc_PLL_Settings* get() const { return &settings_; }
    
    bool enabled() const { return settings_.Enabled != 0; }
    double ref_clk_freq() const { return settings_.RefClkFreq; }
    double sample_rate() const { return settings_.SampleRate; }
    double sample_rate_mhz() const { return settings_.SampleRate * 1000.0; }
    uint32_t feedback_div() const { return settings_.FeedbackDivider; }
    uint32_t output_div() const { return settings_.OutputDivider; }
    
private:
    XRFdc_PLL_Settings settings_{};
};

// RAII wrapper for block status
class BlockStatus {
public:
    BlockStatus() = default;
    
    XRFdc_BlockStatus* get() { return &status_; }
    const XRFdc_BlockStatus* get() const { return &status_; }
    
    double sampling_freq() const { return status_.SamplingFreq; }
    bool digital_path_clocks_enabled() const { return status_.DataPathClocksStatus != 0; }
    bool fifo_flags_enabled() const { return status_.IsFIFOFlagsEnabled != 0; }
    bool fifo_flags_asserted() const { return status_.IsFIFOFlagsAsserted != 0; }
    
private:
    XRFdc_BlockStatus status_{};
};

// RAII wrapper for IP status
class IPStatus {
public:
    explicit IPStatus(const XRFdc_IPStatus& st) : status_(st) {}

    uint32_t state() const { return status_.State; }

    bool dac_tile_enabled(TileId tile) const {
        if (tile > 3) throw RFDCException("Invalid tile ID");
        return status_.DACTileStatus[tile].IsEnabled != 0;
    }

    bool adc_tile_enabled(TileId tile) const {
        if (tile > 3) throw RFDCException("Invalid tile ID");
        return status_.ADCTileStatus[tile].IsEnabled != 0;
    }

private:
    XRFdc_IPStatus status_{};
};


// Main RFDC wrapper class
class RFDC {
public:
    // Constructor - initializes the RFDC
    explicit RFDC(uint16_t device_id = 0);
    
    // Destructor - cleanup metal device
    ~RFDC();
    
    // Disable copy
    RFDC(const RFDC&) = delete;
    RFDC& operator=(const RFDC&) = delete;
    
    // Enable move
    RFDC(RFDC&&) noexcept = default;
    RFDC& operator=(RFDC&&) noexcept = default;
    
    // ===== Startup/Shutdown Operations =====
    void startup(TileType type, TileId tile_id);
    void shutdown(TileType type, TileId tile_id);
    void reset(TileType type, TileId tile_id);
    
    // ===== Status Operations =====
    IPStatus get_ip_status() const;
    BlockStatus get_block_status(TileType type, TileId tile_id, BlockId block_id) const;
    bool check_tile_enabled(TileType type, TileId tile_id) const;
    bool check_block_enabled(TileType type, TileId tile_id, BlockId block_id) const;
    
    // ===== PLL Operations =====
    void set_pll_config(TileType type, TileId tile_id, double ref_clk_freq, double sample_rate);
    PLLSettings get_pll_config(TileType type, TileId tile_id) const;
    bool get_pll_lock_status(TileType type, TileId tile_id) const;
    
    // ===== Mixer Operations =====
    void set_mixer_settings(TileType type, TileId tile_id, BlockId block_id, 
                           const MixerSettings& settings);
    MixerSettings get_mixer_settings(TileType type, TileId tile_id, BlockId block_id) const;
    void reset_nco_phase(TileType type, TileId tile_id, BlockId block_id);
    
    // ===== QMC Operations =====
    void set_qmc_settings(TileType type, TileId tile_id, BlockId block_id,
                         const QMCSettings& settings);
    QMCSettings get_qmc_settings(TileType type, TileId tile_id, BlockId block_id) const;
    
    // ===== Nyquist Zone Operations =====
    void set_nyquist_zone(TileType type, TileId tile_id, BlockId block_id, NyquistZone zone);
    NyquistZone get_nyquist_zone(TileType type, TileId tile_id, BlockId block_id) const;
    
    // ===== Interpolation/Decimation Operations =====
    void set_interpolation_factor(TileId tile_id, BlockId block_id, uint32_t factor);
    uint32_t get_interpolation_factor(TileId tile_id, BlockId block_id) const;
    void set_decimation_factor(TileId tile_id, BlockId block_id, uint32_t factor);
    uint32_t get_decimation_factor(TileId tile_id, BlockId block_id) const;
    
    // ===== FIFO Operations =====
    void setup_fifo(TileType type, TileId tile_id, bool enable);
    bool get_fifo_status(TileType type, TileId tile_id) const;
    
    // ===== Fabric Clock Operations =====
    void set_fabric_clk_out_div(TileType type, TileId tile_id, uint16_t div);
    uint16_t get_fabric_clk_out_div(TileType type, TileId tile_id) const;
    double get_fabric_clk_freq(TileType type, TileId tile_id) const;
    
    // ===== ADC-Specific Operations =====
    void set_threshold_settings(TileId tile_id, BlockId block_id,
                               const ThresholdSettings& settings);
    ThresholdSettings get_threshold_settings(TileId tile_id, BlockId block_id) const;
    void set_calibration_mode(TileId tile_id, BlockId block_id, CalibrationMode mode);
    CalibrationMode get_calibration_mode(TileId tile_id, BlockId block_id) const;
    
    // ===== DAC-Specific Operations =====
    void set_decoder_mode(TileId tile_id, BlockId block_id, uint32_t mode);
    uint32_t get_decoder_mode(TileId tile_id, BlockId block_id) const;
    void set_inverse_sinc_filter(TileId tile_id, BlockId block_id, uint16_t mode);
    uint16_t get_inverse_sinc_filter(TileId tile_id, BlockId block_id) const;
    
    // ===== Interrupt Operations =====
    void enable_interrupts(TileType type, TileId tile_id, BlockId block_id, uint32_t mask);
    void disable_interrupts(TileType type, TileId tile_id, BlockId block_id, uint32_t mask);
    void clear_interrupts(TileType type, TileId tile_id, BlockId block_id, uint32_t mask);
    uint32_t get_interrupt_status(TileType type, TileId tile_id, BlockId block_id) const;
    
    // ===== Multi-Tile Sync Operations =====
    uint32_t multi_converter_sync(TileType type, XRFdc_MultiConverter_Sync_Config* config);
    
    // ===== Update Event =====
    void update_event(TileType type, TileId tile_id, BlockId block_id, uint32_t event);
    
    // ===== Utility Functions =====
    void dump_registers(TileType type, int tile_id) const;
    static std::string get_driver_version();
    
    // Access to underlying instance (for advanced use)
    XRFdc* get_instance() { return &instance_; }
    const XRFdc* get_instance() const { return &instance_; }
    uint32_t get_ip_type() const { return instance_.RFdc_Config.IPType; }
    // ===== Memory Mapping Operations =====
    // Initialize memory mapping for ADC/DAC data buffers and clock wizards
    void initialize_memory_mapping(
        uint32_t adc_base_addr,
        uint32_t dac_base_addr,
        const std::array<uint32_t, 4>& adc_clk_wiz_addrs,
        const std::array<uint32_t, 4>& dac_clk_wiz_addrs
    );
    
    // Get memory map info (for advanced users)
    struct ChannelMap {
        uint32_t addr_I;
        uint32_t addr_Q;
        uint32_t Channel_I;
        uint32_t Channel_Q;
        
        ChannelMap() : addr_I(0xFFFFFFFF), addr_Q(0xFFFFFFFF),
                       Channel_I(0xFFFFFFFF), Channel_Q(0xFFFFFFFF) {}
    };
    
    const std::array<ChannelMap, 16>& get_adc_map() const { return adc_map_; }
    const std::array<ChannelMap, 16>& get_dac_map() const { return dac_map_; }
    void* get_adc_vaddr() const { return mem_info_.vaddr_adc; }
    void* get_dac_vaddr() const { return mem_info_.vaddr_dac; }
    
    // Initialize MMCM (clock wizard) for ADC/DAC tiles
    void initialize_mmcm_adc();
    void initialize_mmcm_dac();

    void set_mmcm(TileType type, TileId tile_id);
    bool check_high_speed_adc(TileId tile_id) const;

    // Get calculated MMCM input frequencies
    const std::array<uint32_t, 8>& get_mmcm_frequencies() const { return mmcm_fin_; }

    // ===== Fabric Interface Operations =====
    void get_fab_rd_vld_words(TileType type, TileId tile_id, BlockId block_id, uint32_t& words) const;
    void get_fab_wr_vld_words(TileType type, TileId tile_id, BlockId block_id, uint32_t& words) const;

    void* get_clk_wiz_base(TileType type, TileId tile_id);

    // Get data path mode (Gen3+ only)
    uint32_t get_data_path_mode(TileId tile_id, BlockId block_id) const;    
private:
    XRFdc instance_{};
    std::unique_ptr<XRFdc_Config> config_;
    struct metal_device* metal_device_{nullptr};
    
    // Memory mapping for ADC/DAC data and clock wizards
    struct MemoryInfo {
        int fd;                    // /dev/mem file descriptor
        void* base_adc;           // ADC mmap base
        void* base_dac;           // DAC mmap base
        void* vaddr_adc;          // ADC virtual address
        void* vaddr_dac;          // DAC virtual address
        uint32_t paddr_adc;       // ADC physical address
        uint32_t paddr_dac;       // DAC physical address
        
        // Clock wizard mappings for each tile
        void* clk_wiz_adc[4];     // ADC clock wizards
        void* clk_wiz_dac[4];     // DAC clock wizards
        
        MemoryInfo() : fd(-1), base_adc(nullptr), base_dac(nullptr),
                       vaddr_adc(nullptr), vaddr_dac(nullptr),
                       paddr_adc(0), paddr_dac(0) {
            for (int i = 0; i < 4; ++i) {
                clk_wiz_adc[i] = nullptr;
                clk_wiz_dac[i] = nullptr;
            }
        }
    };
    
    MemoryInfo mem_info_;
    std::array<ChannelMap, 16> adc_map_;
    std::array<ChannelMap, 16> dac_map_;
    std::array<uint32_t, 16> adc_mem_map_;
    std::array<uint32_t, 16> dac_mem_map_;
    std::array<uint32_t, 16> adc_init_datatype_;
    // MMCM input frequencies [0-3: ADC tiles, 4-7: DAC tiles]
    std::array<uint32_t, 8> mmcm_fin_;
    // Helper functions
    void check_status(uint32_t status, const std::string& operation) const {
        if (status != XRFDC_SUCCESS) {
            throw RFDCException(
                operation + " failed with status: " + 
                std::to_string(status), 
                status
            );
        }
    }
    
    static constexpr uint32_t to_underlying(TileType type) {
        return static_cast<uint32_t>(type);
    }
    
    // Memory mapping helpers
    void cleanup_memory_mapping();
    void build_channel_maps();
    // MMCM programming helpers
    int mmcm_reprogram(TileType type, TileId tile_id,
                      uint32_t mult, uint32_t frac_mult, uint32_t div,
                      uint32_t clkout_div, uint32_t clk0_div_frac);
    uint16_t mmcm_reset(TileType type, TileId tile_id);
    
};

} // namespace rfdc
/******************************************************************************
* Copyright (C) 2024 Charlie
* Modern C++14 wrapper for Xilinx RFDC driver v2024.1
* SPDX-License-Identifier: MIT
******************************************************************************/

#include "RfDc.hpp"
#include <sstream>

namespace rfdc {

// Helper function to format strings (replaces std::format for C++14)
template<typename... Args>
std::string format_string(Args&&... args) {
    std::ostringstream oss;
    using expander = int[];
    (void)expander{0, ((oss << std::forward<Args>(args)), 0)...};
    return oss.str();
}

// Constructor
RFDC::RFDC(uint16_t device_id) {
    // Initialize libmetal first
    struct metal_init_params init_param = {
        .log_handler = nullptr,  // Use default log handler
        .log_level = METAL_LOG_ERROR,  // Can be changed to METAL_LOG_DEBUG if needed
    };
    
    if (metal_init(&init_param) != 0) {
        throw RFDCException(
            "metal_init failed - METAL_LOG_INIT_FAILURE"
        );
    }
    
    // Look up configuration first (before RegisterMetal)
    config_ = std::make_unique<XRFdc_Config>();
    
#ifndef SDT
    auto* cfg_ptr = XRFdc_LookupConfig(device_id);
    if (!cfg_ptr) {
        throw RFDCException(
            format_string("Failed to find configuration for device ", device_id)
        );
    }
    *config_ = *cfg_ptr;
#else
    // For SDT builds, would need different initialization
    throw RFDCException("SDT-based initialization not yet implemented");
#endif
    
    // Register libmetal for memory-mapped I/O (required for Linux userspace)
    auto metal_status = XRFdc_RegisterMetal(&instance_, device_id, &metal_device_);
    if (metal_status != XRFDC_SUCCESS) {
        throw RFDCException(
            format_string("XRFdc_RegisterMetal failed for device ", device_id, 
                         " with status ", metal_status),
            metal_status
        );
    }
    
    // Initialize the driver (controller)
    auto status = XRFdc_CfgInitialize(&instance_, config_.get());
    if (status != XRFDC_SUCCESS) {
        throw RFDCException(
            format_string("XRFdc_CfgInitialize failed with status ", status,
                         " - RFDC_CFG_INIT_FAILURE"),
            status
        );
    }
}

// Destructor
RFDC::~RFDC() {
    // Cleanup libmetal
    metal_finish();
}

// ===== Startup/Shutdown Operations =====

void RFDC::startup(TileType type, TileId tile_id) {
    auto status = XRFdc_StartUp(&instance_, to_underlying(type), tile_id);
    check_status(status, format_string("StartUp tile ", tile_id, " type ", 
                                       to_underlying(type)));
}

void RFDC::shutdown(TileType type, TileId tile_id) {
    auto status = XRFdc_Shutdown(&instance_, to_underlying(type), tile_id);
    check_status(status, format_string("Shutdown tile ", tile_id, " type ", 
                                       to_underlying(type)));
}

void RFDC::reset(TileType type, TileId tile_id) {
    auto status = XRFdc_Reset(&instance_, to_underlying(type), tile_id);
    check_status(status, format_string("Reset tile ", tile_id, " type ", 
                                       to_underlying(type)));
}

// ===== Status Operations =====

IPStatus RFDC::get_ip_status() const {
    IPStatus status;
    auto result = XRFdc_GetIPStatus(
        const_cast<XRFdc*>(&instance_), 
        status.get()
    );
    check_status(result, "GetIPStatus");
    return status;
}

BlockStatus RFDC::get_block_status(TileType type, TileId tile_id, BlockId block_id) const {
    BlockStatus status;
    auto result = XRFdc_GetBlockStatus(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        block_id,
        status.get()
    );
    check_status(result, format_string("GetBlockStatus tile ", tile_id, 
                                       " block ", block_id));
    return status;
}

bool RFDC::check_tile_enabled(TileType type, TileId tile_id) const {
    auto status = XRFdc_CheckTileEnabled(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id
    );
    return status == XRFDC_SUCCESS;
}

bool RFDC::check_block_enabled(TileType type, TileId tile_id, BlockId block_id) const {
    auto status = XRFdc_CheckBlockEnabled(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        block_id
    );
    return status == XRFDC_SUCCESS;
}

// ===== PLL Operations =====

void RFDC::set_pll_config(TileType type, TileId tile_id, 
                          double ref_clk_freq, double sample_rate) {
    auto status = XRFdc_DynamicPLLConfig(
        &instance_,
        to_underlying(type),
        tile_id,
        XRFDC_EXTERNAL_CLK,
        ref_clk_freq,
        sample_rate
    );
    check_status(status, format_string("SetPLLConfig tile ", tile_id));
}

PLLSettings RFDC::get_pll_config(TileType type, TileId tile_id) const {
    PLLSettings settings;
    auto status = XRFdc_GetPLLConfig(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        settings.get()
    );
    check_status(status, format_string("GetPLLConfig tile ", tile_id));
    return settings;
}

bool RFDC::get_pll_lock_status(TileType type, TileId tile_id) const {
    uint32_t lock_status = 0;
    auto status = XRFdc_GetPLLLockStatus(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        &lock_status
    );
    check_status(status, format_string("GetPLLLockStatus tile ", tile_id));
    return lock_status != 0;
}

// ===== Mixer Operations =====

void RFDC::set_mixer_settings(TileType type, TileId tile_id, BlockId block_id,
                              const MixerSettings& settings) {
    auto status = XRFdc_SetMixerSettings(
        &instance_,
        to_underlying(type),
        tile_id,
        block_id,
        const_cast<XRFdc_Mixer_Settings*>(settings.get())
    );
    check_status(status, format_string("SetMixerSettings tile ", tile_id, " block ", block_id));
}

MixerSettings RFDC::get_mixer_settings(TileType type, TileId tile_id, BlockId block_id) const {
    MixerSettings settings;
    auto status = XRFdc_GetMixerSettings(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        block_id,
        settings.get()
    );
    check_status(status, format_string("GetMixerSettings tile ", tile_id, " block ", block_id));
    return settings;
}

void RFDC::reset_nco_phase(TileType type, TileId tile_id, BlockId block_id) {
    auto status = XRFdc_ResetNCOPhase(
        &instance_,
        to_underlying(type),
        tile_id,
        block_id
    );
    check_status(status, format_string("ResetNCOPhase tile ", tile_id, " block ", block_id));
}

// ===== QMC Operations =====

void RFDC::set_qmc_settings(TileType type, TileId tile_id, BlockId block_id,
                            const QMCSettings& settings) {
    auto status = XRFdc_SetQMCSettings(
        &instance_,
        to_underlying(type),
        tile_id,
        block_id,
        const_cast<XRFdc_QMC_Settings*>(settings.get())
    );
    check_status(status, format_string("SetQMCSettings tile ", tile_id, " block ", block_id));
}

QMCSettings RFDC::get_qmc_settings(TileType type, TileId tile_id, BlockId block_id) const {
    QMCSettings settings;
    auto status = XRFdc_GetQMCSettings(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        block_id,
        settings.get()
    );
    check_status(status, format_string("GetQMCSettings tile ", tile_id, " block ", block_id));
    return settings;
}

// ===== Nyquist Zone Operations =====

void RFDC::set_nyquist_zone(TileType type, TileId tile_id, BlockId block_id, NyquistZone zone) {
    auto status = XRFdc_SetNyquistZone(
        &instance_,
        to_underlying(type),
        tile_id,
        block_id,
        static_cast<uint32_t>(zone)
    );
    check_status(status, format_string("SetNyquistZone tile ", tile_id, " block ", block_id));
}

NyquistZone RFDC::get_nyquist_zone(TileType type, TileId tile_id, BlockId block_id) const {
    uint32_t zone = 0;
    auto status = XRFdc_GetNyquistZone(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        block_id,
        &zone
    );
    check_status(status, format_string("GetNyquistZone tile ", tile_id, " block ", block_id));
    return static_cast<NyquistZone>(zone);
}

// ===== Interpolation/Decimation Operations =====

void RFDC::set_interpolation_factor(TileId tile_id, BlockId block_id, uint32_t factor) {
    auto status = XRFdc_SetInterpolationFactor(
        &instance_,
        tile_id,
        block_id,
        factor
    );
    check_status(status, format_string("SetInterpolationFactor tile ", tile_id, " block ", block_id));
}

uint32_t RFDC::get_interpolation_factor(TileId tile_id, BlockId block_id) const {
    uint32_t factor = 0;
    auto status = XRFdc_GetInterpolationFactor(
        const_cast<XRFdc*>(&instance_),
        tile_id,
        block_id,
        &factor
    );
    check_status(status, format_string("GetInterpolationFactor tile ", tile_id, " block ", block_id));
    return factor;
}

void RFDC::set_decimation_factor(TileId tile_id, BlockId block_id, uint32_t factor) {
    auto status = XRFdc_SetDecimationFactor(
        &instance_,
        tile_id,
        block_id,
        factor
    );
    check_status(status, format_string("SetDecimationFactor tile ", tile_id, " block ", block_id));
}

uint32_t RFDC::get_decimation_factor(TileId tile_id, BlockId block_id) const {
    uint32_t factor = 0;
    auto status = XRFdc_GetDecimationFactor(
        const_cast<XRFdc*>(&instance_),
        tile_id,
        block_id,
        &factor
    );
    check_status(status, format_string("GetDecimationFactor tile ", tile_id, " block ", block_id));
    return factor;
}

// ===== FIFO Operations =====

void RFDC::setup_fifo(TileType type, TileId tile_id, bool enable) {
    auto status = XRFdc_SetupFIFO(
        &instance_,
        to_underlying(type),
        tile_id,
        enable ? 1 : 0
    );
    check_status(status, format_string("SetupFIFO tile ", tile_id));
}

bool RFDC::get_fifo_status(TileType type, TileId tile_id) const {
    uint8_t enabled = 0;
    auto status = XRFdc_GetFIFOStatus(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        &enabled
    );
    check_status(status, format_string("GetFIFOStatus tile ", tile_id));
    return enabled != 0;
}

// ===== Fabric Clock Operations =====

void RFDC::set_fabric_clk_out_div(TileType type, TileId tile_id, uint16_t div) {
    auto status = XRFdc_SetFabClkOutDiv(
        &instance_,
        to_underlying(type),
        tile_id,
        div
    );
    check_status(status, format_string("SetFabClkOutDiv tile ", tile_id));
}

uint16_t RFDC::get_fabric_clk_out_div(TileType type, TileId tile_id) const {
    uint16_t div = 0;
    auto status = XRFdc_GetFabClkOutDiv(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        &div
    );
    check_status(status, format_string("GetFabClkOutDiv tile ", tile_id));
    return div;
}

double RFDC::get_fabric_clk_freq(TileType type, TileId tile_id) const {
    return XRFdc_GetFabClkFreq(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id
    );
}

// ===== ADC-Specific Operations =====

void RFDC::set_threshold_settings(TileId tile_id, BlockId block_id,
                                  const ThresholdSettings& settings) {
    auto status = XRFdc_SetThresholdSettings(
        &instance_,
        tile_id,
        block_id,
        const_cast<XRFdc_Threshold_Settings*>(settings.get())
    );
    check_status(status, format_string("SetThresholdSettings tile ", tile_id, " block ", block_id));
}

ThresholdSettings RFDC::get_threshold_settings(TileId tile_id, BlockId block_id) const {
    ThresholdSettings settings;
    auto status = XRFdc_GetThresholdSettings(
        const_cast<XRFdc*>(&instance_),
        tile_id,
        block_id,
        settings.get()
    );
    check_status(status, format_string("GetThresholdSettings tile ", tile_id, " block ", block_id));
    return settings;
}

void RFDC::set_calibration_mode(TileId tile_id, BlockId block_id, CalibrationMode mode) {
    auto status = XRFdc_SetCalibrationMode(
        &instance_,
        tile_id,
        block_id,
        static_cast<uint8_t>(mode)
    );
    check_status(status, format_string("SetCalibrationMode tile ", tile_id, " block ", block_id));
}

CalibrationMode RFDC::get_calibration_mode(TileId tile_id, BlockId block_id) const {
    uint8_t mode = 0;
    auto status = XRFdc_GetCalibrationMode(
        const_cast<XRFdc*>(&instance_),
        tile_id,
        block_id,
        &mode
    );
    check_status(status, format_string("GetCalibrationMode tile ", tile_id, " block ", block_id));
    return static_cast<CalibrationMode>(mode);
}

// ===== DAC-Specific Operations =====

void RFDC::set_decoder_mode(TileId tile_id, BlockId block_id, uint32_t mode) {
    auto status = XRFdc_SetDecoderMode(
        &instance_,
        tile_id,
        block_id,
        mode
    );
    check_status(status, format_string("SetDecoderMode tile ", tile_id, " block ", block_id));
}

uint32_t RFDC::get_decoder_mode(TileId tile_id, BlockId block_id) const {
    uint32_t mode = 0;
    auto status = XRFdc_GetDecoderMode(
        const_cast<XRFdc*>(&instance_),
        tile_id,
        block_id,
        &mode
    );
    check_status(status, format_string("GetDecoderMode tile ", tile_id, " block ", block_id));
    return mode;
}

void RFDC::set_inverse_sinc_filter(TileId tile_id, BlockId block_id, uint16_t mode) {
    auto status = XRFdc_SetInvSincFIR(
        &instance_,
        tile_id,
        block_id,
        mode
    );
    check_status(status, format_string("SetInvSincFIR tile ", tile_id, " block ", block_id));
}

uint16_t RFDC::get_inverse_sinc_filter(TileId tile_id, BlockId block_id) const {
    uint16_t mode = 0;
    auto status = XRFdc_GetInvSincFIR(
        const_cast<XRFdc*>(&instance_),
        tile_id,
        block_id,
        &mode
    );
    check_status(status, format_string("GetInvSincFIR tile ", tile_id, " block ", block_id));
    return mode;
}

// ===== Interrupt Operations =====

void RFDC::enable_interrupts(TileType type, TileId tile_id, BlockId block_id, uint32_t mask) {
    auto status = XRFdc_IntrEnable(
        &instance_,
        to_underlying(type),
        tile_id,
        block_id,
        mask
    );
    check_status(status, format_string("IntrEnable tile ", tile_id, " block ", block_id));
}

void RFDC::disable_interrupts(TileType type, TileId tile_id, BlockId block_id, uint32_t mask) {
    auto status = XRFdc_IntrDisable(
        &instance_,
        to_underlying(type),
        tile_id,
        block_id,
        mask
    );
    check_status(status, format_string("IntrDisable tile ", tile_id, " block ", block_id));
}

void RFDC::clear_interrupts(TileType type, TileId tile_id, BlockId block_id, uint32_t mask) {
    auto status = XRFdc_IntrClr(
        &instance_,
        to_underlying(type),
        tile_id,
        block_id,
        mask
    );
    check_status(status, format_string("IntrClr tile ", tile_id, " block ", block_id));
}

uint32_t RFDC::get_interrupt_status(TileType type, TileId tile_id, BlockId block_id) const {
    uint32_t status_mask = 0;
    auto status = XRFdc_GetIntrStatus(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        block_id,
        &status_mask
    );
    check_status(status, format_string("GetIntrStatus tile ", tile_id, " block ", block_id));
    return status_mask;
}

// ===== Multi-Tile Sync Operations =====

uint32_t RFDC::multi_converter_sync(TileType type, XRFdc_MultiConverter_Sync_Config* config) {
    return XRFdc_MultiConverter_Sync(
        &instance_,
        to_underlying(type),
        config
    );
}

// ===== Update Event =====

void RFDC::update_event(TileType type, TileId tile_id, BlockId block_id, uint32_t event) {
    auto status = XRFdc_UpdateEvent(
        &instance_,
        to_underlying(type),
        tile_id,
        block_id,
        event
    );
    check_status(status, format_string("UpdateEvent tile ", tile_id, " block ", block_id));
}

// ===== Utility Functions =====

void RFDC::dump_registers(TileType type, int tile_id) const {
    XRFdc_DumpRegs(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id
    );
}

std::string RFDC::get_driver_version() {
    const auto version = XRFdc_GetDriverVersion();
    return std::to_string(version);
}

} // namespace rfdc
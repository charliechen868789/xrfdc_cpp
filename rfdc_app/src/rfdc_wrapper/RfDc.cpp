/******************************************************************************
* Copyright (C) 2024 Charlie
* Modern C++14 wrapper for Xilinx RFDC driver v2024.1
* SPDX-License-Identifier: MIT
******************************************************************************/

#include "RfDc.hpp"
#include <sstream>
#include <fcntl.h>      // For open()
#include <sys/mman.h>   // For mmap()
#include <unistd.h>     // For close()
#include <cstring>      // For memset()

namespace rfdc {

// Constants for memory mapping
constexpr size_t MAP_SIZE = 4096UL;
constexpr size_t MAP_MASK = (MAP_SIZE - 1);
constexpr uint32_t LMEM_INFO = 0x0;  // Offset for memory info register

// Helper function to read from local memory
static inline uint32_t lmem_rd32(void* addr) {
    return *static_cast<volatile uint32_t*>(addr);
}

// Helper function to format strings (replaces std::format for C++14)
template<typename... Args>
std::string format_string(Args&&... args) {
    std::ostringstream oss;
    using expander = int[];
    (void)expander{0, ((oss << std::forward<Args>(args)), 0)...};
    return oss.str();
}

// Constructor
RFDC::RFDC(uint16_t device_id) 
{
    // Initialize libmetal first - C++14 compatible way
    struct metal_init_params init_param;
    init_param.log_handler = nullptr;
    init_param.log_level = METAL_LOG_DEBUG;
    
    if (metal_init(&init_param) != 0) {
        throw RFDCException(
            "metal_init failed - METAL_LOG_INIT_FAILURE"
        );
    }
    
    // Look up configuration first (before RegisterMetal)
    config_ = std::make_unique<XRFdc_Config>();
    
    auto* cfg_ptr = XRFdc_LookupConfig(device_id);
    if (!cfg_ptr) {
        throw RFDCException(
            format_string("Failed to find configuration for device ", device_id)
        );
    }
    *config_ = *cfg_ptr;
    
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
    if (status != XRFDC_SUCCESS) 
    {
        throw RFDCException(
            format_string("XRFdc_CfgInitialize failed with status ", status,
                         " - RFDC_CFG_INIT_FAILURE"),
            status
        );
    }
}

// Destructor
RFDC::~RFDC() {
    // Cleanup memory mapping first
    cleanup_memory_mapping();
    // Cleanup libmetal
    metal_finish();
}

// ===== Startup/Shutdown Operations =====

void RFDC::startup(TileType type, TileId tile_id) 
{
    auto status = XRFdc_StartUp(&instance_, to_underlying(type), tile_id);
    check_status(status, format_string("StartUp tile ", tile_id, " type ", 
                                       to_underlying(type)));
}

void RFDC::shutdown(TileType type, TileId tile_id) 
{
    auto status = XRFdc_Shutdown(&instance_, to_underlying(type), tile_id);
    check_status(status, format_string("Shutdown tile ", tile_id, " type ", 
                                       to_underlying(type)));
}

void RFDC::reset(TileType type, TileId tile_id) 
{
    auto status = XRFdc_Reset(&instance_, to_underlying(type), tile_id);
    check_status(status, format_string("Reset tile ", tile_id, " type ", 
                                       to_underlying(type)));
}

// ===== Status Operations =====

IPStatus RFDC::get_ip_status() const {
    XRFdc_IPStatus st{};
    auto ret = XRFdc_GetIPStatus(
        const_cast<XRFdc*>(&instance_),
        &st
    );
    check_status(ret, "GetIPStatus");
    return IPStatus{st};
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

PLLSettings RFDC::get_pll_config(TileType type, TileId tile_id) const 
{
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

bool RFDC::get_pll_lock_status(TileType type, TileId tile_id) const 
{
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


// ===== DAC-Specific Operations =====

uint32_t RFDC::get_data_path_mode(TileId tile_id, BlockId block_id) const {
    uint32_t mode = 0;
    uint32_t status = XRFdc_GetDataPathMode(
        const_cast<XRFdc*>(&instance_),
        tile_id,
        block_id,
        &mode
    );
    check_status(status, "Get data path mode");
    return mode;
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

// ===== Memory Mapping Operations =====

void RFDC::initialize_memory_mapping(
    uint32_t adc_base_addr,
    uint32_t dac_base_addr,
    const std::array<uint32_t, 4>& adc_clk_wiz_addrs,
    const std::array<uint32_t, 4>& dac_clk_wiz_addrs)
{
    // Open /dev/mem for memory-mapped I/O
    mem_info_.fd = open("/dev/mem", O_RDWR | O_NDELAY);
    if (mem_info_.fd < 0) 
    {
        throw RFDCException("Failed to open /dev/mem - check permissions");
    }
    
    // Map ADC base address
    mem_info_.base_adc = mmap(nullptr, MAP_SIZE * 2, 
                              PROT_READ | PROT_WRITE, MAP_SHARED,
                              mem_info_.fd, adc_base_addr & ~MAP_MASK);
    if (mem_info_.base_adc == MAP_FAILED) {
        close(mem_info_.fd);
        throw RFDCException("Failed to mmap ADC base address");
    }
    mem_info_.vaddr_adc = static_cast<char*>(mem_info_.base_adc) + 
                          (adc_base_addr & MAP_MASK);
    mem_info_.paddr_adc = adc_base_addr;
    
    // Map DAC base address
    mem_info_.base_dac = mmap(nullptr, MAP_SIZE * 2,
                              PROT_READ | PROT_WRITE, MAP_SHARED,
                              mem_info_.fd, dac_base_addr & ~MAP_MASK);
    if (mem_info_.base_dac == MAP_FAILED) {
        munmap(mem_info_.base_adc, MAP_SIZE * 2);
        close(mem_info_.fd);
        throw RFDCException("Failed to mmap DAC base address");
    }
    mem_info_.vaddr_dac = static_cast<char*>(mem_info_.base_dac) + 
                          (dac_base_addr & MAP_MASK);
    mem_info_.paddr_dac = dac_base_addr;
    
    // Map clock wizards for ADC tiles
    for (int i = 0; i < 4; ++i) 
    {
        mem_info_.clk_wiz_adc[i] = mmap(nullptr, MAP_SIZE * 2,
                                        PROT_READ | PROT_WRITE, MAP_SHARED,
                                        mem_info_.fd, 
                                        adc_clk_wiz_addrs[i] & ~MAP_MASK);
        if (mem_info_.clk_wiz_adc[i] == MAP_FAILED) {
            cleanup_memory_mapping();
            throw RFDCException(
                format_string("Failed to mmap ADC clock wizard ", i)
            );
        }
    }
    
    // Map clock wizards for DAC tiles
    for (int i = 0; i < 4; ++i) {
        mem_info_.clk_wiz_dac[i] = mmap(nullptr, MAP_SIZE * 2,
                                        PROT_READ | PROT_WRITE, MAP_SHARED,
                                        mem_info_.fd,
                                        dac_clk_wiz_addrs[i] & ~MAP_MASK);
        if (mem_info_.clk_wiz_dac[i] == MAP_FAILED) {
            cleanup_memory_mapping();
            throw RFDCException(
                format_string("Failed to mmap DAC clock wizard ", i)
            );
        }
    }
    
    // Build channel address maps
    build_channel_maps();
}

void RFDC::cleanup_memory_mapping() {
    // Unmap all memory regions
    if (mem_info_.base_adc && mem_info_.base_adc != MAP_FAILED) {
        munmap(mem_info_.base_adc, MAP_SIZE * 2);
    }
    if (mem_info_.base_dac && mem_info_.base_dac != MAP_FAILED) {
        munmap(mem_info_.base_dac, MAP_SIZE * 2);
    }
    
    // Unmap clock wizards
    for (int i = 0; i < 4; ++i) {
        if (mem_info_.clk_wiz_adc[i] && mem_info_.clk_wiz_adc[i] != MAP_FAILED) {
            munmap(mem_info_.clk_wiz_adc[i], MAP_SIZE * 2);
        }
        if (mem_info_.clk_wiz_dac[i] && mem_info_.clk_wiz_dac[i] != MAP_FAILED) {
            munmap(mem_info_.clk_wiz_dac[i], MAP_SIZE * 2);
        }
    }
    
    // Close /dev/mem
    if (mem_info_.fd >= 0) {
        close(mem_info_.fd);
        mem_info_.fd = -1;
    }
}

void RFDC::build_channel_maps() {
    // Initialize all maps to invalid
    for (size_t i = 0; i < 16; ++i) {
        adc_map_[i] = ChannelMap();  // Uses default constructor (all 0xFFFFFFFF)
        dac_map_[i] = ChannelMap();
        adc_mem_map_[i] = 0xFFFFFFFF;
        dac_mem_map_[i] = 0xFFFFFFFF;
        adc_init_datatype_[i] = 0;
    }
    
    // Read ADC memory info
    uint32_t mem_info = lmem_rd32(
        static_cast<char*>(mem_info_.vaddr_adc) + LMEM_INFO
    );
    uint32_t mem_size = mem_info & 0x00FFFFFF;        // bits [23:0]
    uint32_t num_mem = (mem_info >> 24) & 0x0000007F; // bits [30:24]
    mem_size = 2 * mem_size / (num_mem + 1) / 16;
    
    uint32_t cur_addr = mem_info_.paddr_adc + mem_size;
    uint16_t channel = 0;
    
    // Build ADC channel map
    for (uint32_t tile = 0; tile < 4; ++tile) {
        uint32_t num_blocks = instance_.RFdc_Config.ADCTile_Config[tile].NumSlices;
        
        for (uint32_t block = 0; block < num_blocks; ++block) {
            if (XRFdc_IsADCDigitalPathEnabled(&instance_, tile, block)) {
                uint32_t idx = tile * num_blocks + block;
                
                // Set I channel address
                adc_map_[idx].addr_I = cur_addr;
                adc_map_[idx].Channel_I = channel;
                adc_mem_map_[channel] = idx;
                cur_addr += mem_size;
                channel++;
                
                // Check for IQ mode (complex data)
                bool is_high_speed = XRFdc_IsHighSpeedADC(&instance_, tile);
                bool is_iq_mode = XRFdc_GetDataType(&instance_, static_cast<u32>(TileType::ADC), tile, 
                                                     block << instance_.ADC4GSPS);
                
                if (is_high_speed && is_iq_mode) {
                    // Separate Q channel
                    adc_map_[idx].addr_Q = cur_addr;
                    adc_map_[idx].Channel_Q = channel;
                    adc_mem_map_[channel] = idx;
                    adc_init_datatype_[idx] = 1;  // IQ mode
                    cur_addr += mem_size;
                    channel++;
                } else {
                    // Real mode - Q uses same address as I
                    adc_map_[idx].addr_Q = adc_map_[idx].addr_I;
                    adc_map_[idx].Channel_Q = adc_map_[idx].Channel_I;
                    adc_init_datatype_[idx] = 0;  // Real mode
                }
            }
        }
    }
    
    // Build DAC channel map
    channel = 0;
    mem_info = lmem_rd32(static_cast<char*>(mem_info_.vaddr_dac) + LMEM_INFO);
    mem_size = mem_info & 0x00FFFFFF;
    num_mem = (mem_info >> 24) & 0x0000007F;
    mem_size = 2 * mem_size / (num_mem + 1) / 16;
    cur_addr = mem_info_.paddr_dac + mem_size;
    
    // Calculate number of DAC tiles based on IP type
    uint32_t num_dac_tiles = 4;
    if (instance_.ADC4GSPS && (instance_.RFdc_Config.IPType < XRFDC_GEN3)) {
        num_dac_tiles = 4 >> 1;  // Half the tiles for 4GSPS Gen1/Gen2
    }
    
    for (uint32_t tile = 0; tile < num_dac_tiles; ++tile) {
        for (uint32_t block = 0; block < 4; ++block) {
            if (XRFdc_IsDACDigitalPathEnabled(&instance_, tile, block)) {
                uint32_t idx = tile * 4 + block;
                
                // DACs typically share I and Q addresses
                dac_map_[idx].addr_I = cur_addr;
                dac_map_[idx].addr_Q = cur_addr;
                dac_map_[idx].Channel_I = channel;
                dac_map_[idx].Channel_Q = channel;
                dac_mem_map_[channel] = idx;
                cur_addr += mem_size;
                channel++;
            }
        }
    }
}

void RFDC::initialize_mmcm_adc() 
{
    // Clock wizard register offsets
    constexpr uint32_t CLKREG_OFFSET = 0x200;
    constexpr uint32_t CLKOUT_OFFSET = 0x208;
    
    uint32_t mts_enable = 0;  // Multi-tile sync (typically 0)
    
    for (uint32_t tile_id = 0; tile_id < 4; ++tile_id) {
        // Check if tile is enabled
        if (XRFdc_CheckTileEnabled(&instance_, XRFDC_ADC_TILE, tile_id) != XRFDC_SUCCESS) {
            mmcm_fin_[tile_id] = 0;
            continue;
        }
        
        // Select clock wizard base address for this tile
        void* clk_base_addr = nullptr;
        switch (tile_id & !mts_enable) {
            case 0: clk_base_addr = mem_info_.clk_wiz_adc[0]; break;
            case 1: clk_base_addr = mem_info_.clk_wiz_adc[1]; break;
            case 2: clk_base_addr = mem_info_.clk_wiz_adc[2]; break;
            case 3: clk_base_addr = mem_info_.clk_wiz_adc[3]; break;
        }
        
        if (!clk_base_addr) {
            mmcm_fin_[tile_id] = 0;
            continue;
        }
        
        // Read clock configuration registers
        uint32_t clk_config_reg = lmem_rd32(static_cast<char*>(clk_base_addr) + CLKREG_OFFSET);
        uint32_t mult = (clk_config_reg & 0x0000FF00) >> 8;
        uint32_t div = (clk_config_reg & 0x000000FF);
        uint32_t frac_mult = (clk_config_reg & 0x03FF0000) >> 16;
        mult = (1000 * mult) + frac_mult;
        
        clk_config_reg = lmem_rd32(static_cast<char*>(clk_base_addr) + CLKOUT_OFFSET);
        uint32_t clkout_div = (clk_config_reg & 0x000000FF);
        uint32_t clkout_frac = (clk_config_reg & 0x0003FF00) >> 8;
        clkout_div = (1000 * clkout_div) + clkout_frac;
        
        // Get PLL settings to obtain sample rate
        XRFdc_PLL_Settings pll_settings;
        auto status = XRFdc_GetPLLConfig(&instance_, XRFDC_ADC_TILE, tile_id, &pll_settings);
        if (status != XRFDC_SUCCESS) {
            mmcm_fin_[tile_id] = 0;
            continue;
        }
        
        uint32_t sample_rate = static_cast<uint32_t>(1000000.0 * pll_settings.SampleRate);
        
        // Find first enabled block in this tile
        uint32_t block_id = 0;
        while (XRFdc_CheckDigitalPathEnabled(&instance_, XRFDC_ADC_TILE, tile_id, block_id) != XRFDC_SUCCESS 
               && block_id < 4) {
            block_id++;
        }
        
        if (block_id >= 4) {
            mmcm_fin_[tile_id] = 0;
            continue;
        }
        
        // Get fabric data rate
        uint32_t fabric_data_rate = 0;
        XRFdc_GetFabRdVldWords(&instance_, XRFDC_ADC_TILE, tile_id, block_id, &fabric_data_rate);
        
        // Get mixer settings to determine if data is Real or IQ
        XRFdc_Mixer_Settings mixer_settings;
        status = XRFdc_GetMixerSettings(&instance_, XRFDC_ADC_TILE, tile_id, block_id, &mixer_settings);
        
        uint32_t data_iq = 1;  // 1 = Real, 2 = IQ
        bool is_high_speed = XRFdc_IsHighSpeedADC(&instance_, tile_id);
        
        if (mixer_settings.MixerMode == XRFDC_MIXER_MODE_R2R ||
            is_high_speed ||
            (mixer_settings.MixerType == XRFDC_MIXER_TYPE_COARSE &&
             mixer_settings.CoarseMixFreq == XRFDC_COARSE_MIX_BYPASS)) {
            data_iq = 1;
        } else {
            data_iq = 2;
        }
        
        // Get decimation mode
        uint32_t decimation_mode = 
            instance_.RFdc_Config.ADCTile_Config[tile_id]
                .ADCBlock_Digital_Config[block_id].DecimationMode;
        
        // Calculate MMCM output frequency
        uint32_t mmcm_fout = sample_rate * data_iq / (decimation_mode * fabric_data_rate);
        
        // Calculate MMCM input frequency
        mmcm_fin_[tile_id] = mmcm_fout * div * clkout_div / mult;
    }
}

void RFDC::initialize_mmcm_dac() {
    // Clock wizard register offsets
    constexpr uint32_t CLKREG_OFFSET = 0x200;
    constexpr uint32_t CLKOUT_OFFSET = 0x208;
    
    uint32_t mts_enable = 0;  // Multi-tile sync (typically 0)
    
    // Calculate number of DAC tiles based on IP type
    uint32_t num_dac_tiles = 4;
    if (instance_.ADC4GSPS && (instance_.RFdc_Config.IPType < XRFDC_GEN3)) {
        num_dac_tiles = 4 >> 1;  // Half the tiles for 4GSPS Gen1/Gen2
    }
    
    for (uint32_t tile_id = 0; tile_id < num_dac_tiles; ++tile_id) {
        uint32_t clk_wiz_id = 4 + tile_id;  // DAC uses indices 4-7
        
        // Check if tile is enabled
        if (XRFdc_CheckTileEnabled(&instance_, XRFDC_DAC_TILE, tile_id) != XRFDC_SUCCESS) {
            mmcm_fin_[clk_wiz_id] = 0;
            continue;
        }
        
        // Select clock wizard base address for this tile
        void* clk_base_addr = nullptr;
        switch (tile_id & !mts_enable) {
            case 0: clk_base_addr = mem_info_.clk_wiz_dac[0]; break;
            case 1: clk_base_addr = mem_info_.clk_wiz_dac[1]; break;
            case 2: clk_base_addr = mem_info_.clk_wiz_dac[2]; break;
            case 3: clk_base_addr = mem_info_.clk_wiz_dac[3]; break;
        }
        
        if (!clk_base_addr) {
            mmcm_fin_[clk_wiz_id] = 0;
            continue;
        }
        
        // Read clock configuration registers
        uint32_t clk_config_reg = lmem_rd32(static_cast<char*>(clk_base_addr) + CLKREG_OFFSET);
        uint32_t mult = (clk_config_reg & 0x0000FF00) >> 8;
        uint32_t div = (clk_config_reg & 0x000000FF);
        uint32_t frac_mult = (clk_config_reg & 0x03FF0000) >> 16;
        mult = (1000 * mult) + frac_mult;
        
        clk_config_reg = lmem_rd32(static_cast<char*>(clk_base_addr) + CLKOUT_OFFSET);
        uint32_t clkout_div = (clk_config_reg & 0x000000FF);
        uint32_t clkout_frac = (clk_config_reg & 0x0003FF00) >> 8;
        clkout_div = (1000 * clkout_div) + clkout_frac;
        
        // Get PLL settings to obtain sample rate
        XRFdc_PLL_Settings pll_settings;
        auto status = XRFdc_GetPLLConfig(&instance_, XRFDC_DAC_TILE, tile_id, &pll_settings);
        if (status != XRFDC_SUCCESS) {
            mmcm_fin_[clk_wiz_id] = 0;
            continue;
        }
        
        uint32_t sample_rate = static_cast<uint32_t>(1000000.0 * pll_settings.SampleRate);
        
        // Find first enabled block in this tile
        uint32_t block_id = 0;
        while (XRFdc_CheckDigitalPathEnabled(&instance_, XRFDC_DAC_TILE, tile_id, block_id) != XRFDC_SUCCESS 
               && block_id < 4) {
            block_id++;
        }
        
        if (block_id >= 4) {
            mmcm_fin_[clk_wiz_id] = 0;
            continue;
        }
        
        // Get fabric data rate
        uint32_t fabric_data_rate = 0;
        XRFdc_GetFabWrVldWords(&instance_, XRFDC_DAC_TILE, tile_id, block_id, &fabric_data_rate);
        
        // Get data path mode (Gen3 only)
        uint32_t data_path_mode = 1;
        if (instance_.RFdc_Config.IPType >= XRFDC_GEN3) {
            status = XRFdc_GetDataPathMode(&instance_, tile_id, block_id, &data_path_mode);
        }
        
        // Get mixer settings and interpolation factor
        XRFdc_Mixer_Settings mixer_settings;
        uint32_t inter_decim = 1;
        
        if (data_path_mode != 4) {
            status = XRFdc_GetMixerSettings(&instance_, XRFDC_DAC_TILE, tile_id, block_id, &mixer_settings);
            XRFdc_GetInterpolationFactor(&instance_, tile_id, block_id, &inter_decim);
        }
        
        // Adjust interpolation for certain data path modes
        if (data_path_mode == 2 || data_path_mode == 3) {
            inter_decim = 2 * inter_decim;
        }
        
        // Determine if data is Real or IQ
        uint32_t data_iq = 1;  // 1 = Real, 2 = IQ
        if (data_path_mode == 4 ||
            mixer_settings.MixerMode == XRFDC_MIXER_MODE_R2R ||
            mixer_settings.MixerMode == XRFDC_MIXER_MODE_R2C ||
            (mixer_settings.MixerType == XRFDC_MIXER_TYPE_COARSE &&
             mixer_settings.CoarseMixFreq == XRFDC_COARSE_MIX_BYPASS)) {
            data_iq = 1;
        } else {
            data_iq = 2;
        }
        
        // Calculate MMCM output frequency
        uint32_t mmcm_fout = sample_rate * data_iq / (inter_decim * fabric_data_rate);
        
        // Calculate MMCM input frequency
        mmcm_fin_[clk_wiz_id] = mmcm_fout * div * clkout_div / mult;
    }
    
    // Set unused DAC clock wizard frequencies to 0
    for (uint32_t i = num_dac_tiles; i < 4; ++i) {
        mmcm_fin_[4 + i] = 0;
    }
}


// ===== MMCM Management Operations =====

bool RFDC::check_high_speed_adc(TileId tile_id) const {
    return XRFdc_IsHighSpeedADC(const_cast<XRFdc*>(&instance_), tile_id) != 0;
}

void* RFDC::get_clk_wiz_base(TileType type, TileId tile_id) {
    if (type == TileType::DAC) {
        if (tile_id < 4) {
            return mem_info_.clk_wiz_dac[tile_id];
        }
    } else {
        if (tile_id < 4) {
            return mem_info_.clk_wiz_adc[tile_id];
        }
    }
    return nullptr;
}

#if 0
void RFDC::set_mmcm(TileType type, TileId tile_id)
{
    // Get PLL settings
    XRFdc_PLL_Settings pll_settings;
    auto status = XRFdc_GetPLLConfig(&instance_, to_underlying(type), 
                                     tile_id, &pll_settings);
    check_status(status, format_string("GetPLLConfig for MMCM setup"));
    
    double sample_rate = pll_settings.SampleRate * 1000.0;  // Convert GSPS to MHz
    
    // Get fabric clock divider
    uint16_t fab_clk_div = 0;
    status = XRFdc_GetFabClkOutDiv(&instance_, to_underlying(type), 
                                   tile_id, &fab_clk_div);
    check_status(status, format_string("GetFabClkOutDiv"));
    
    // Find first enabled block
    uint32_t block_id = 0;
    while (block_id < 4) {
        if (XRFdc_CheckDigitalPathEnabled(&instance_, to_underlying(type),
                                         tile_id, block_id) == XRFDC_SUCCESS) {
            break;
        }
        block_id++;
    }
    
    if (block_id >= 4) {
        throw RFDCException("No enabled blocks found for MMCM setup");
    }
    
    // Get fabric data rate and interpolation/decimation settings
    uint32_t fabric_words = 0;
    uint32_t inter_decim = 1;
    uint32_t data_iq = 1;  // 1 = Real, 2 = IQ
    
    if (type == TileType::DAC) {
        // Get fabric write words
        status = XRFdc_GetFabWrVldWords(&instance_, to_underlying(type),
                                       tile_id, block_id, &fabric_words);
        check_status(status, format_string("GetFabWrVldWords"));
        
        // Get interpolation factor
        status = XRFdc_GetInterpolationFactor(&instance_, tile_id, 
                                             block_id, &inter_decim);
        check_status(status, format_string("GetInterpolationFactor"));
        
        // Get mixer settings to determine Real vs IQ
        XRFdc_Mixer_Settings mixer;
        status = XRFdc_GetMixerSettings(&instance_, to_underlying(type),
                                       tile_id, block_id, &mixer);
        check_status(status, format_string("GetMixerSettings"));
        
        // Determine data type
        if (mixer.MixerMode == XRFDC_MIXER_MODE_R2R ||
            mixer.MixerMode == XRFDC_MIXER_MODE_R2C ||
            (mixer.MixerType == XRFDC_MIXER_TYPE_COARSE &&
             mixer.CoarseMixFreq == XRFDC_COARSE_MIX_BYPASS)) {
            data_iq = 1;
        } else {
            data_iq = 2;
        }
        
        // Check for datapath mode (Gen3+ only)
        if (instance_.RFdc_Config.IPType >= XRFDC_GEN3) {
            uint32_t datapath_mode = 1;
            status = XRFdc_GetDataPathMode(&instance_, tile_id, 
                                          block_id, &datapath_mode);
            if (status == XRFDC_SUCCESS) {
                if (datapath_mode == 2 || datapath_mode == 3) {
                    inter_decim = 2 * inter_decim;
                }
                if (datapath_mode == 4) {
                    data_iq = 1;
                }
            }
        }
        
    } else {
        // ADC
        // Get fabric read words
        status = XRFdc_GetFabRdVldWords(&instance_, to_underlying(type),
                                       tile_id, block_id, &fabric_words);
        check_status(status, format_string("GetFabRdVldWords"));
        
        // Get decimation factor
        status = XRFdc_GetDecimationFactor(&instance_, tile_id, 
                                          block_id, &inter_decim);
        check_status(status, format_string("GetDecimationFactor"));
        
        // Get mixer settings
        XRFdc_Mixer_Settings mixer;
        status = XRFdc_GetMixerSettings(&instance_, to_underlying(type),
                                       tile_id, block_id, &mixer);
        check_status(status, format_string("GetMixerSettings"));
        
        // Determine data type
        if (mixer.MixerMode == XRFDC_MIXER_MODE_R2R ||
            XRFdc_IsHighSpeedADC(&instance_, tile_id) ||
            (mixer.MixerType == XRFDC_MIXER_TYPE_COARSE &&
             mixer.CoarseMixFreq == XRFDC_COARSE_MIX_BYPASS)) {
            data_iq = 1;
        } else {
            data_iq = 2;
        }
    }
    
    // Calculate MMCM parameters
    uint32_t const_divider = 8;  // Typically 8 for most configurations
    
    double fdc_out = sample_rate / (const_divider * (1 << (fab_clk_div - 1)));
    double fplin = fdc_out;
    
    // Store the calculated MMCM input frequency
    mmcm_fin_[4 * to_underlying(type) + tile_id] = static_cast<uint32_t>(1000.0 * fdc_out);
    
    // Calculate required frequency ratio
    uint32_t fratio_n = data_iq * const_divider * (1 << (fab_clk_div - 1));
    uint32_t fratio_d = inter_decim * fabric_words;
    
    // MMCM constraints
    int fpdmax = 450;     // MHz - Phase detector max
    int fpdmin = 70;      // MHz - Phase detector min
    int fvco_max = 1500;  // MHz - VCO max
    int fvco_min = 800;   // MHz - VCO min
    
    uint32_t div_min = (fplin > fpdmax) ? 2 : 1;
    uint32_t div_max = static_cast<uint32_t>(fplin / fpdmin);
    
    if (div_max == 0) {
        fpdmin = 10;
        div_max = static_cast<uint32_t>(fplin / fpdmin);
    }
    
    if (div_max == 0) {
        throw RFDCException(
            format_string("MMCM spec violation: Fin=", fplin, 
                         " is below minimum ", fpdmin)
        );
    }
    
    // Find valid MMCM parameters
    bool found = false;
    uint32_t best_mult = 0, best_div = 0, best_clkout_div = 0;
    
    for (uint32_t div = div_min; div <= div_max && !found; div++) {
        uint32_t mult_min = static_cast<uint32_t>(
            std::ceil(static_cast<double>(fvco_min) * div / fplin)
        );
        uint32_t mult_max = static_cast<uint32_t>(fvco_max * div / fplin);
        
        if (mult_min < 2) mult_min = 2;
        if (mult_max > 128) mult_max = 128;
        
        for (uint32_t mult = mult_max; mult >= mult_min && !found; mult--) {
            for (uint32_t clkout = 1; clkout <= 128; clkout++) {
                // Check if ratio matches: fratio_n/fratio_d = mult/(div*clkout)
                if (div * fratio_n * clkout == mult * fratio_d) {
                    best_mult = mult;
                    best_div = div;
                    best_clkout_div = clkout;
                    found = true;
                    break;
                }
            }
        }
    }
    
    if (!found) {
        throw RFDCException(
            format_string("Could not find valid MMCM ratio for Fin=", 
                         fplin, " ratio=", fratio_n, "/", fratio_d)
        );
    }
    
    // Validate parameters
    double vco_freq = fplin * best_mult / best_div;
    double fpd = fplin / best_div;
    double fout = fplin * best_mult / (best_div * best_clkout_div);
    
    // Check all MMCM constraints
    if (best_mult < 2 || best_mult > 128) {
        throw RFDCException(
            format_string("MMCM Mult=", best_mult, " outside range 2-128")
        );
    }
    
    if (vco_freq < fvco_min || vco_freq > fvco_max) {
        throw RFDCException(
            format_string("MMCM VCO=", vco_freq, "MHz outside range ",
                         fvco_min, "-", fvco_max, "MHz")
        );
    }
    
    if (fpd < fpdmin || fpd > fpdmax) {
        throw RFDCException(
            format_string("MMCM Fpd=", fpd, "MHz outside range ",
                         fpdmin, "-", fpdmax, "MHz")
        );
    }
    
    if (fout < 6.25) {
        throw RFDCException(
            format_string("MMCM Fout=", fout, "MHz below minimum 6.25MHz")
        );
    }
    
    // Program the MMCM
    usleep(200);
    int ret = mmcm_reprogram(type, tile_id, best_mult, 0, best_div,
                            best_clkout_div, 0);
    if (ret != 0) {
        throw RFDCException("Failed to reprogram MMCM");
    }
    usleep(200);
    
    // Reset and check lock
    uint16_t locked = mmcm_reset(type, tile_id);
    
    if (!locked) {
        throw RFDCException(
            format_string("MMCM failed to lock after reset with M=",
                         best_mult, " D=", best_div, 
                         " CLKOUT=", best_clkout_div)
        );
    }
}

#endif
void RFDC::set_mmcm(TileType type, TileId tile_id)
{
    const u32 Type    = to_underlying(type);   // must match XRFDC_ADC_TILE / XRFDC_DAC_TILE
    const u32 Tile_Id = tile_id;

    XRFdc_PLL_Settings PLLSettings{};
    XRFdc_Mixer_Settings Mixer_Settings{};
    u32 DataIQ = 1;              // 1 real, 2 IQ
    u16 FabClkDiv = 0;
    u32 DataPathMode = 1;
    double SampleRate = 0.0;     // MHz
    double FDCout = 0.0;         // MHz
    double Fplin  = 0.0;         // MHz
    u32 InterDecim = 1;
    u32 RdWidth = 0;
    u32 WrWidth = 0;
    u32 Wpl = 0;
    u32 Block_Id = 0;

    // ---------- Find a usable block ----------
    // Safer: find first block where digital path is enabled.
    bool found_block = false;
    for (u32 b = 0; b < 4; ++b) {
        int r = XRFdc_CheckDigitalPathEnabled(&instance_, Type, Tile_Id, b);
        if (r == XRFDC_SUCCESS) {   // treat SUCCESS as enabled
            Block_Id = b;
            found_block = true;
            break;
        }
    }
    if (!found_block) {
        throw RFDCException("No enabled blocks found for MMCM setup");
    }

    // ---------- ConstDivider logic (match reference) ----------
    // Reference chooses 8 for DAC always, 4 for some ADC cases.
    const bool is_hs_adc = (Type == XRFDC_ADC_TILE) ? (XRFdc_IsHighSpeedADC(&instance_, Tile_Id) != 0) : false;

    const u32 ConstDivider =
        ((Type == XRFDC_ADC_TILE &&
          instance_.RFdc_Config.IPType < XRFDC_GEN3 &&
          is_hs_adc) ||
         (Type == XRFDC_DAC_TILE) ||
         (Type == XRFDC_ADC_TILE &&
          instance_.RFdc_Config.IPType >= XRFDC_GEN3 &&
          is_hs_adc))
        ? 8u : 4u;

    // ---------- Read tile/block parameters ----------
    auto st = XRFdc_GetPLLConfig(&instance_, Type, Tile_Id, &PLLSettings);
    check_status(st, "GetPLLConfig");

    st = XRFdc_GetFabClkOutDiv(&instance_, Type, Tile_Id, &FabClkDiv);
    check_status(st, "GetFabClkOutDiv");

    st = XRFdc_GetFabRdVldWords(&instance_, Type, Tile_Id, Block_Id, &RdWidth);
    check_status(st, "GetFabRdVldWords");

    st = XRFdc_GetFabWrVldWords(&instance_, Type, Tile_Id, Block_Id, &WrWidth);
    check_status(st, "GetFabWrVldWords");

    if (Type == XRFDC_DAC_TILE) {
        Wpl = WrWidth;

        if (instance_.RFdc_Config.IPType >= XRFDC_GEN3) {
            st = XRFdc_GetDataPathMode(&instance_, Tile_Id, Block_Id, &DataPathMode);
            // ref code ORs status; we’ll accept failure as “unknown” only if you want.
            check_status(st, "GetDataPathMode");
        }

        if (DataPathMode != 4) {
            st = XRFdc_GetMixerSettings(&instance_, Type, Tile_Id, Block_Id, &Mixer_Settings);
            check_status(st, "GetMixerSettings");

            st = XRFdc_GetInterpolationFactor(&instance_, Tile_Id, Block_Id, &InterDecim);
            check_status(st, "GetInterpolationFactor");
        }

        if (DataPathMode == 2 || DataPathMode == 3) {
            InterDecim = 2 * InterDecim;
        }

        if ((DataPathMode == 4) ||
            (Mixer_Settings.MixerMode == XRFDC_MIXER_MODE_R2R) ||
            (Mixer_Settings.MixerMode == XRFDC_MIXER_MODE_R2C) ||
            (Mixer_Settings.MixerType == XRFDC_MIXER_TYPE_COARSE &&
             Mixer_Settings.CoarseMixFreq == XRFDC_COARSE_MIX_BYPASS))
        {
            DataIQ = 1;
        } else {
            DataIQ = 2;
        }

    } else {
        // ADC
        Wpl = RdWidth;

        st = XRFdc_GetMixerSettings(&instance_, Type, Tile_Id, Block_Id, &Mixer_Settings);
        check_status(st, "GetMixerSettings");

        st = XRFdc_GetDecimationFactor(&instance_, Tile_Id, Block_Id, &InterDecim);
        check_status(st, "GetDecimationFactor");

        if ((Mixer_Settings.MixerMode == XRFDC_MIXER_MODE_R2R) ||
            is_hs_adc ||
            (Mixer_Settings.MixerType == XRFDC_MIXER_TYPE_COARSE &&
             Mixer_Settings.CoarseMixFreq == XRFDC_COARSE_MIX_BYPASS))
        {
            DataIQ = 1;
        } else {
            DataIQ = 2;
        }
    }

    // ---------- Compute Fin (match reference math) ----------
    SampleRate = 1000.0 * PLLSettings.SampleRate;  // GSPS -> MHz
    FDCout = SampleRate / static_cast<double>(ConstDivider * (1u << (FabClkDiv - 1)));
    Fplin = FDCout;

    mmcm_fin_[4 * Type + Tile_Id] = static_cast<u32>(1000.0 * FDCout); // kHz

    // ---------- Ratio search (match reference) ----------
    const int Fpdmax = 450;
    int Fpdmin = 70;
    const int FvcoMax = 1500;
    const int FvcoMin = 800;
    const int FoutMin_kHz = 6250;

    u32 Div_Min = (Fplin > Fpdmax) ? 2u : 1u;
    u32 Div_Max = static_cast<u32>(Fplin / Fpdmin);

    if (Div_Max == 0) {
        Fpdmin = 10;
        Div_Max = static_cast<u32>(Fplin / Fpdmin);
    }
    if (Div_Max == 0) {
        throw RFDCException(format_string("MMCM spec violation: Fin=", Fplin, "MHz is below ", Fpdmin, "MHz"));
    }

    const u32 Fratio_N = DataIQ * ConstDivider * (1u << (FabClkDiv - 1));
    const u32 Fratio_D = InterDecim * Wpl;

    bool found_ratio = false;
    u32 bestMult = 0, bestDiv = 0, bestClkout0 = 1;

    for (u32 Div = Div_Min; Div <= Div_Max && !found_ratio; ++Div) {

        u32 Mult_Min = static_cast<u32>(std::ceil(FvcoMin * Div / Fplin));
        u32 Mult_Max = static_cast<u32>(FvcoMax * Div / Fplin);

        if (Mult_Min < 2) Mult_Min = 2;
        if (Mult_Max > 128) Mult_Max = 128;

        for (u32 Mult = Mult_Max; Mult >= Mult_Min && !found_ratio; --Mult) {
            for (u32 i = 1; i <= 128; ++i) {
                if (Div * Fratio_N * i == Mult * Fratio_D) {
                    bestDiv = Div;
                    bestMult = Mult;
                    bestClkout0 = i;
                    found_ratio = true;
                    break;
                }
            }
            if (Mult == Mult_Min) break; // prevent u32 underflow
        }
    }

    if (!found_ratio) {
        throw RFDCException(format_string("Could not find MMCM ratio for Type=", Type,
                                          " Tile=", Tile_Id, " Fin=", Fplin,
                                          " N=", Fratio_N, " D=", Fratio_D));
    }

    // ---------- Spec checks (match reference intent) ----------
    const double VCO = Fplin * bestMult / bestDiv;
    const double Fpd = Fplin / bestDiv;
    const double Fout_MHz = Fplin * bestMult / (bestDiv * bestClkout0);
    const double Fout_kHz = 1000.0 * Fout_MHz;

    if (bestMult < 2 || bestMult > 128) {
        throw RFDCException(format_string("MMCM spec violation: Mult=", bestMult));
    }
    if (Fout_MHz < 6.25) {
        throw RFDCException(format_string("MMCM output too low: Fout=", Fout_MHz, "MHz (<6.25MHz). Interp/Decim too high?"));
    }
    if (Fplin < 10.0) {
        throw RFDCException(format_string("MMCM spec violation: Fin=", Fplin, "MHz < 10MHz"));
    }
    if (VCO < FvcoMin || VCO > FvcoMax) {
        throw RFDCException(format_string("MMCM VCO out of range: VCO=", VCO, "MHz"));
    }
    if (Fpd < Fpdmin || Fpd > Fpdmax) {
        throw RFDCException(format_string("MMCM Fpd out of range: Fpd=", Fpd, "MHz"));
    }
    if (Fout_kHz < FoutMin_kHz) {
        throw RFDCException(format_string("MMCM Fout out of range: Fout=", Fout_kHz, "kHz < ", FoutMin_kHz, "kHz"));
    }

    // ---------- Program + reset + lock ----------
    usleep(200);
    int ret = mmcm_reprogram(type, tile_id,
                             bestMult, /*ClkFbOutFrac=*/0,
                             bestDiv,
                             bestClkout0, /*Clk0DivFrac=*/0);
    if (ret != 0) {
        throw RFDCException("Failed to reprogram MMCM");
    }
    usleep(200);

    const uint16_t locked = mmcm_reset(type, tile_id);
    if (!locked) {
        throw RFDCException(format_string("MMCM not locked after reset: Fin=", Fplin,
                                          " M=", bestMult, " D=", bestDiv, " clkout0=", bestClkout0,
                                          " (try shutdown/startup tile; likely need re-config)"));
    }
}

int RFDC::mmcm_reprogram(TileType type, TileId tile_id,
                        uint32_t mult, uint32_t frac_mult,
                        uint32_t div, uint32_t clkout_div,
                        uint32_t clk0_div_frac)
{
    void* clk_base = get_clk_wiz_base(type, tile_id);
    if (!clk_base) {
        return -1;
    }
    
    // Clock Wizard register offsets
    constexpr uint32_t CLKREG_OFFSET = 0x200;   // Clock config register
    constexpr uint32_t CLKOUT_OFFSET = 0x208;   // Clock output config
    
    // Prepare clock config: [25:16]=frac_mult, [15:8]=mult, [7:0]=div
    uint32_t clk_config = (frac_mult << 16) | (mult << 8) | div;
    
    // Prepare clkout config: [17:8]=frac, [7:0]=div
    uint32_t clkout_config = (clk0_div_frac << 8) | clkout_div;
    
    // Write registers using reinterpret_cast
    volatile uint32_t* clk_reg = reinterpret_cast<volatile uint32_t*>(
        static_cast<char*>(clk_base) + CLKREG_OFFSET
    );
    volatile uint32_t* clkout_reg = reinterpret_cast<volatile uint32_t*>(
        static_cast<char*>(clk_base) + CLKOUT_OFFSET
    );
    
    *clk_reg = clk_config;
    *clkout_reg = clkout_config;
    
    return 0;
}

uint16_t RFDC::mmcm_reset(TileType type, TileId tile_id)
{
    void* clk_base = get_clk_wiz_base(type, tile_id);
    if (!clk_base) {
        return 0;
    }
    
    // MMCM control register offsets
    constexpr uint32_t MMCM_RESET_OFFSET = 0x25C;
    constexpr uint32_t MMCM_LOCK_OFFSET = 0x004;
    
    // Use reinterpret_cast for pointer conversions
    volatile uint32_t* reset_reg = reinterpret_cast<volatile uint32_t*>(
        static_cast<char*>(clk_base) + MMCM_RESET_OFFSET
    );
    volatile uint32_t* lock_reg = reinterpret_cast<volatile uint32_t*>(
        static_cast<char*>(clk_base) + MMCM_LOCK_OFFSET
    );
    
    // Assert reset
    *reset_reg = 0xA;
    usleep(10);
    
    // Deassert reset
    *reset_reg = 0x0;
    usleep(1000);  // Wait for lock
    
    // Read lock status
    uint32_t lock_status = *lock_reg;
    
    return (lock_status & 0x1) ? 1 : 0;
}

// ===== Fabric Interface Operations =====

void RFDC::get_fab_rd_vld_words(TileType type, TileId tile_id, BlockId block_id, uint32_t& words) const {
    uint32_t status = XRFdc_GetFabRdVldWords(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        block_id,
        &words
    );
    check_status(status, "Get fabric read valid words");
}

void RFDC::get_fab_wr_vld_words(TileType type, TileId tile_id, BlockId block_id, uint32_t& words) const {
    uint32_t status = XRFdc_GetFabWrVldWords(
        const_cast<XRFdc*>(&instance_),
        to_underlying(type),
        tile_id,
        block_id,
        &words
    );
    check_status(status, "Get fabric write valid words");
}

} // namespace rfdc

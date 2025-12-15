#pragma once

#include <cstdint>
#include <memory>
#include "rfdc_wrapper/RfDc.hpp"

namespace clock_wizard {

/**
 * @brief Clock Wizard (MMCM) controller for RFDC fabric clocking
 * 
 * Manages the Xilinx Clocking Wizard IP that generates the AXI-Stream
 * fabric clock from the RFDC output clock.
 */
class ClockWizard {
public:
    struct MMCMConfig {
        uint32_t mult;          // Multiplier (M)
        uint32_t mult_frac;     // Fractional multiplier
        uint32_t div;           // Input divider (D)
        uint32_t clkout0_div;   // Output divider (O)
        uint32_t clkout0_frac;  // Fractional output divider
        uint32_t clkout1_div;   // Output divider for second clock
        
        double fin_mhz;         // Input frequency (MHz)
        double fvco_mhz;        // VCO frequency (MHz)
        double fout_mhz;        // Output frequency (MHz)
        bool locked;            // MMCM lock status
    };
    
    // MMCM register offsets
    static constexpr uint32_t MMCM_STATUS_REG = 0x04;
    static constexpr uint32_t MMCM_CLK_CONFIG0_REG = 0x200;
    static constexpr uint32_t MMCM_CLKOUT0_REG = 0x208;
    static constexpr uint32_t MMCM_CLKOUT1_REG = 0x214;
    static constexpr uint32_t MMCM_LOAD_REG = 0x25C;
    static constexpr uint32_t MMCM_RESET_REG = 0x00;
    
    // MMCM specification limits (MHz)
    static constexpr double FPD_MAX = 450.0;
    static constexpr double FPD_MIN = 70.0;
    static constexpr double FVCO_MAX = 1500.0;
    static constexpr double FVCO_MIN = 800.0;
    static constexpr double FOUT_MIN = 6.25;
    
    /**
     * @brief Construct ClockWizard controller
     * @param rfdc Pointer to RFDC instance
     */
    explicit ClockWizard(rfdc::RFDC* rfdc);
    
    /**
     * @brief Calculate and program MMCM for a tile
     * @param type DAC or ADC
     * @param tile_id Tile ID (0-3)
     * @return true on success
     */
    bool program_mmcm(rfdc::TileType type, uint32_t tile_id);
    
    /**
     * @brief Reset MMCM
     * @param type DAC or ADC
     * @param tile_id Tile ID (0-3)
     * @return true if locked after reset
     */
    bool reset_mmcm(rfdc::TileType type, uint32_t tile_id);
    
    /**
     * @brief Get current MMCM configuration
     * @param type DAC or ADC
     * @param tile_id Tile ID (0-3)
     * @return MMCMConfig structure
     */
    MMCMConfig get_mmcm_config(rfdc::TileType type, uint32_t tile_id);
    
    /**
     * @brief Get MMCM input frequency (calculated)
     * @param type DAC or ADC
     * @param tile_id Tile ID (0-3)
     * @return Frequency in kHz
     */
    uint32_t get_mmcm_fin_khz(rfdc::TileType type, uint32_t tile_id) const;

private:
    rfdc::RFDC* rfdc_;
    
    // Calculated MMCM input frequencies [0-3: ADC, 4-7: DAC] in kHz
    uint32_t mmcm_fin_[8];
    
    /**
     * @brief Calculate MMCM parameters for required clock ratio
     * @param fplin Input frequency (MHz)
     * @param ratio_n Numerator of frequency ratio
     * @param ratio_d Denominator of frequency ratio
     * @param mult Output: Multiplier value
     * @param div Output: Divider value
     * @param clkout0_div Output: Output divider
     * @return true if valid parameters found
     */
    bool calculate_mmcm_params(double fplin, uint32_t ratio_n, uint32_t ratio_d,
                               uint32_t& mult, uint32_t& div, uint32_t& clkout0_div);
    
    /**
     * @brief Reprogram MMCM hardware registers
     * @param type DAC or ADC
     * @param tile_id Tile ID
     * @param mult Multiplier
     * @param mult_frac Fractional multiplier
     * @param div Divider
     * @param clkout0_div Output divider
     * @param clkout0_frac Fractional output divider
     * @return true if locked after programming
     */
    bool reprogram_hw(rfdc::TileType type, uint32_t tile_id,
                     uint32_t mult, uint32_t mult_frac, uint32_t div,
                     uint32_t clkout0_div, uint32_t clkout0_frac);
    
    /**
     * @brief Reset MMCM hardware
     * @param type DAC or ADC
     * @param tile_id Tile ID
     * @return true if locked after reset
     */
    bool reset_hw(rfdc::TileType type, uint32_t tile_id);
    
    /**
     * @brief Get clock wizard base address
     * @param type DAC or ADC
     * @param tile_id Tile ID
     * @return Base address pointer
     */
    void* get_clk_wiz_base(rfdc::TileType type, uint32_t tile_id);
    
    /**
     * @brief Write to MMCM register
     */
    void write_reg(void* base, uint32_t offset, uint32_t value);
    
    /**
     * @brief Read from MMCM register
     */
    uint32_t read_reg(void* base, uint32_t offset);
};

} // namespace clock_wizard
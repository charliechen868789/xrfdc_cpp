#include "ClockWizard.hpp"
#include <iostream>
#include <cmath>
#include <unistd.h>

namespace clock_wizard {

ClockWizard::ClockWizard(rfdc::RFDC* rfdc)
    : rfdc_(rfdc)
{
    // Initialize MMCM input frequencies
    for (int i = 0; i < 8; i++) {
        mmcm_fin_[i] = 0;
    }
}

void* ClockWizard::get_clk_wiz_base(rfdc::TileType type, uint32_t tile_id) {
    // This calls into your RFDC wrapper which has the memory-mapped clock wizard addresses
    return rfdc_->get_clk_wiz_base(type, tile_id);
}

void ClockWizard::write_reg(void* base, uint32_t offset, uint32_t value) {
    // Fix: use reinterpret_cast for pointer arithmetic, then cast to volatile
    char* byte_ptr = static_cast<char*>(base) + offset;
    volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(byte_ptr);
    *reg = value;
}

uint32_t ClockWizard::read_reg(void* base, uint32_t offset) 
{
    // Fix: use reinterpret_cast for pointer arithmetic, then cast to volatile
    char* byte_ptr = static_cast<char*>(base) + offset;
    volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(byte_ptr);
    return *reg;
}

bool ClockWizard::program_mmcm(rfdc::TileType type, uint32_t tile_id) {
    std::cout << "  Programming MMCM for " 
              << (type == rfdc::TileType::DAC ? "DAC" : "ADC")
              << " Tile " << tile_id << "...\n";
    
    // Find first DISABLED block (while enabled, increment)
    uint32_t block_id = 0;
    while (block_id < 4 && rfdc_->check_block_enabled(type, tile_id, block_id)) {
        block_id++;
    }
    if (block_id >= 4) block_id = 0;  // Fallback to block 0
    
    // Get IP type for Gen3 checks
    uint32_t ip_type = rfdc_->get_ip_type();
    bool is_gen3_plus = (ip_type >= XRFDC_GEN3);
    
    // Calculate ConstDivider (matches RFTool exactly - complex ternary)
    bool is_high_speed = rfdc_->check_high_speed_adc(tile_id);
    uint32_t const_divider = 
        ((type == rfdc::TileType::ADC && 
          ip_type < XRFDC_GEN3 && 
          is_high_speed) ||
         type == rfdc::TileType::DAC ||
         (type == rfdc::TileType::ADC && 
          ip_type >= XRFDC_GEN3 && 
          is_high_speed)) ? 8 : 4;
    
    // Get PLL settings
    auto pll = rfdc_->get_pll_config(type, tile_id);
    double sample_rate_gsps = pll.sample_rate();  // In GSPS (e.g., 7.86432)
    
    // Get fabric clock divider
    uint16_t fab_clk_div = rfdc_->get_fabric_clk_out_div(type, tile_id);
    
    // Get fabric interface width
    uint32_t rd_width = 0, wr_width = 0;
    rfdc_->get_fab_rd_vld_words(type, tile_id, block_id, rd_width);
    rfdc_->get_fab_wr_vld_words(type, tile_id, block_id, wr_width);
    
    uint32_t wpl;
    uint32_t data_path_mode = 1;
    uint32_t inter_decim = 1;
    
    if (type == rfdc::TileType::DAC) {
        wpl = wr_width;
        
        // Get data path mode for Gen3+
        if (is_gen3_plus) {
            try {
                data_path_mode = rfdc_->get_data_path_mode(tile_id, block_id);
            } catch (...) {
                data_path_mode = 1;  // Default
            }
        }
        
        // Get mixer settings and interpolation (only if not DUC DDR mode)
        if (data_path_mode != 4) {
            inter_decim = rfdc_->get_interpolation_factor(tile_id, block_id);
        }
        
        // Double interpolation for ALT_BOND modes
        if (data_path_mode == 2 || data_path_mode == 3) {
            inter_decim = 2 * inter_decim;
        }
    } else {
        // ADC
        wpl = rd_width;
        inter_decim = rfdc_->get_decimation_factor(tile_id, block_id);
    }
    
    // Determine if Real or IQ mode
    auto mixer = rfdc_->get_mixer_settings(type, tile_id, block_id);
    uint32_t data_iq = 1;  // 1=Real, 2=IQ
    
    if (type == rfdc::TileType::DAC) {
        if ((data_path_mode == 4) ||
            (mixer.mode() == rfdc::MixerMode::R2R) ||
            (mixer.mode() == rfdc::MixerMode::R2C) ||
            (mixer.type() == rfdc::MixerType::Coarse &&
             mixer.coarse_mix_freq() == XRFDC_COARSE_MIX_BYPASS)) {
            data_iq = 1;
        } else {
            data_iq = 2;
        }
    } else {
        if ((mixer.mode() == rfdc::MixerMode::R2R) ||
            is_high_speed ||
            (mixer.type() == rfdc::MixerType::Coarse &&
             mixer.coarse_mix_freq() == XRFDC_COARSE_MIX_BYPASS)) {
            data_iq = 1;
        } else {
            data_iq = 2;
        }
    }
    
    // Calculate frequencies (RFTool: SampleRate = 1000 * PLLSettings.SampleRate)
    double sample_rate_mhz = 1000.0 * sample_rate_gsps;  // GSPS → MHz
    double fdc_out = sample_rate_mhz / (const_divider * (1 << (fab_clk_div - 1)));
    double fplin = fdc_out;
    
    // Store for later retrieval (in kHz)
    uint32_t idx = (type == rfdc::TileType::DAC) ? (4 + tile_id) : tile_id;
    mmcm_fin_[idx] = static_cast<uint32_t>(1000.0 * fdc_out);  // Store in kHz
    
    // Calculate required frequency ratio
    uint32_t fratio_n = data_iq * const_divider * (1 << (fab_clk_div - 1));
    uint32_t fratio_d = inter_decim * wpl;
    double fratio = fratio_n / static_cast<double>(fratio_d);
    
    std::cout << "    Sample Rate: " << sample_rate_gsps << " GSPS (" 
              << sample_rate_mhz << " MHz)\n";
    std::cout << "    ConstDivider: " << const_divider << "\n";
    std::cout << "    FabClkDiv: " << fab_clk_div << " (shift=" << (1 << (fab_clk_div - 1)) << ")\n";
    if (type == rfdc::TileType::DAC) {
        std::cout << "    DataPathMode: " << data_path_mode << "\n";
    }
    std::cout << "    FDCout/Fplin: " << fplin << " MHz\n";
    std::cout << "    Ratio: " << fratio_n << "/" << fratio_d << " = " << fratio << "\n";
    std::cout << "    DataIQ: " << data_iq << " (1=Real, 2=IQ)\n";
    std::cout << "    Interp/Decim: " << inter_decim << "\n";
    std::cout << "    Words per clock: " << wpl << "\n";
    
    // Calculate divider limits
    uint32_t div_min = (fplin > 450.0) ? 2 : 1;
    uint32_t div_max = static_cast<uint32_t>(fplin / 70.0);
    
    if (div_max == 0) {
        div_max = static_cast<uint32_t>(fplin / 10.0);
    }
    
    if (div_max == 0) {
        std::cerr << "    ✗ MMCM spec violation: Fin=" << fplin << " MHz is below 10 MHz\n";
        return false;
    }
    
    // Try to find valid M/D/O combination
    uint32_t mult = 1, div = 1, clkout0_div = 1;
    bool found_ratio = false;
    
    for (div = div_min; div <= div_max; div++) {
        uint32_t mult_min;
        if (static_cast<uint32_t>(800.0 * div / fplin) == 800.0 * div / fplin) {
            mult_min = static_cast<uint32_t>(800.0 * div / fplin);
        } else {
            mult_min = static_cast<uint32_t>(1 + 800.0 * div / fplin);
        }
        
        uint32_t mult_max = static_cast<uint32_t>(1500.0 * div / fplin);
        
        for (mult = mult_max; mult >= mult_min; mult--) {
            for (uint32_t i = 1; i <= 128; i++) {
                if (div * fratio_n * i == mult * fratio_d) {
                    clkout0_div = i;
                    found_ratio = true;
                    goto end_search;
                }
            }
        }
    }
    
end_search:
    
    if (!found_ratio) {
        std::cerr << "    ✗ Could not find MMCM/PLL ratio for Fin=" << fplin 
                  << " MHz, Fout=" << (fratio * fplin) << " MHz\n";
        return false;
    }
    
    double fvco = (fplin * mult) / div;
    double fout = (fplin * mult) / (div * clkout0_div);
    
    std::cout << "    Found: M=" << mult << " D=" << div << " O=" << clkout0_div << "\n";
    std::cout << "    VCO: " << fvco << " MHz\n";
    std::cout << "    Fout: " << fout << " MHz\n";
    
    // Validate parameters (matches RFTool order exactly)
    bool validation_failed = false;
    
    if (mult < 2 || mult > 128) {
        std::cerr << "    ✗ MMCM spec violation: Mult=" << mult << " outside range 2-128\n";
        validation_failed = true;
    }
    
    if (fratio * fplin < 6.25) {
        std::cerr << "    ✗ MMCM spec violation: Output frequency " << (fratio * fplin) 
                  << " MHz < 6.25 MHz (may be due to interpolation/decimation too high)\n";
        validation_failed = true;
    }
    
    if (fplin < 10.0) {
        std::cerr << "    ✗ MMCM spec violation: Fin=" << fplin << " MHz < 10 MHz\n";
        std::cerr << "       M=" << mult << ", D=" << div << ", O=" << clkout0_div << "\n";
        validation_failed = true;
    }
    
    if ((800.0 * div > fplin * mult) || (div * 1500.0 < fplin * mult)) {
        std::cerr << "    ✗ MMCM spec violation: VCO=" << fvco << " MHz outside 800-1500 MHz\n";
        std::cerr << "       M=" << mult << ", D=" << div << ", O=" << clkout0_div << "\n";
        validation_failed = true;
    }
    
    double fpd = fplin / div;
    if ((70.0 > fpd) || (450.0 < fpd)) {
        std::cerr << "    ✗ MMCM spec violation: Fph=" << fpd << " MHz outside 70-450 MHz\n";
        std::cerr << "       M=" << mult << ", D=" << div << ", O=" << clkout0_div << "\n";
        validation_failed = true;
    }
    
    double fout_khz = 1000.0 * fplin * mult / (div * clkout0_div);
    if (fout_khz < 6250.0) {
        std::cerr << "    ✗ MMCM spec violation: Fout=" << fout_khz << " kHz < 6250 kHz\n";
        std::cerr << "       M=" << mult << ", D=" << div << ", O=" << clkout0_div << "\n";
        validation_failed = true;
    }
    
    if (validation_failed) {
        return false;
    }
    
    // Program the MMCM
    std::cout << "    Programming MMCM registers...\n";
    usleep(200);
    if (!reprogram_hw(type, tile_id, mult, 0, div, clkout0_div, 0)) {
        std::cerr << "    ✗ MMCM reprogramming failed\n";
        return false;
    }
    
    // Reset to lock
    std::cout << "    Resetting MMCM...\n";
    usleep(200);
    if (!reset_hw(type, tile_id)) {
        std::cerr << "    ✗ MMCM is not locked after reset\n";
        std::cerr << "       Fin=" << fplin << " MHz, M=" << mult 
                  << ", D=" << div << ", O=" << clkout0_div << "\n";
        std::cerr << "       Try shutdown/startup the tile or re-configure device\n";
        return false;
    }
    
    std::cout << "    ✓ MMCM programmed and locked\n";
    return true;
}

bool ClockWizard::calculate_mmcm_params(double fplin, uint32_t ratio_n, uint32_t ratio_d,
                                        uint32_t& mult, uint32_t& div, uint32_t& clkout0_div)
{
    // Calculate divider range
    uint32_t div_min = (fplin > FPD_MAX) ? 2 : 1;
    uint32_t div_max = static_cast<uint32_t>(fplin / FPD_MIN);
    if (div_max == 0) div_max = 1;
    
    // Try to find valid M/D/O combination
    for (div = div_min; div <= div_max; div++) {
        uint32_t mult_min = static_cast<uint32_t>(std::ceil(FVCO_MIN * div / fplin));
        uint32_t mult_max = static_cast<uint32_t>(FVCO_MAX * div / fplin);
        
        if (mult_min < 2) mult_min = 2;
        if (mult_max > 128) mult_max = 128;
        
        for (mult = mult_max; mult >= mult_min; mult--) {
            // Try to find clkout0_div that satisfies the ratio
            for (uint32_t i = 1; i <= 128; i++) {
                if (div * ratio_n * i == mult * ratio_d) {
                    clkout0_div = i;
                    
                    // Validate
                    double fvco = (fplin * mult) / div;
                    double fout = (fplin * mult) / (div * i);
                    
                    if (fvco >= FVCO_MIN && fvco <= FVCO_MAX &&
                        fout >= FOUT_MIN) {
                        return true;
                    }
                }
            }
        }
    }
    
    return false;
}

bool ClockWizard::reprogram_hw(rfdc::TileType type, uint32_t tile_id,
                               uint32_t mult, uint32_t mult_frac, uint32_t div,
                               uint32_t clkout0_div, uint32_t clkout0_frac)
{
    void* base = get_clk_wiz_base(type, tile_id);
    if (!base) {
        std::cerr << "Invalid clock wizard base address\n";
        return false;
    }
    
    usleep(100);
    
    // Write CLK_CONFIG0: [31:16]=mult_frac, [15:8]=mult, [7:0]=div
    uint32_t clk_conf0 = (mult_frac << 16) | (mult << 8) | div;
    write_reg(base, MMCM_CLK_CONFIG0_REG, clk_conf0);
    
    usleep(100);
    
    // Write CLKOUT0: [15:8]=clkout0_frac, [7:0]=clkout0_div
    uint32_t clkout0 = (clkout0_frac << 8) | clkout0_div;
    write_reg(base, MMCM_CLKOUT0_REG, clkout0);
    
    // Write CLKOUT1: always half of CLKOUT0
    write_reg(base, MMCM_CLKOUT1_REG, clkout0_div << 1);
    
    usleep(100);
    
    // Load new configuration
    write_reg(base, MMCM_LOAD_REG, 0x03);
    
    usleep(100);
    
    // Check lock
    uint32_t status = read_reg(base, MMCM_STATUS_REG);
    return (status & 0x00000001) != 0;
}

bool ClockWizard::reset_hw(rfdc::TileType type, uint32_t tile_id) {
    void* base = get_clk_wiz_base(type, tile_id);
    if (!base) {
        return false;
    }
    
    // Read current config to preserve settings
    uint32_t clk_conf0 = read_reg(base, MMCM_CLK_CONFIG0_REG);
    uint32_t clkout0 = read_reg(base, MMCM_CLKOUT0_REG);
    
    // Reset
    write_reg(base, MMCM_RESET_REG, 0x0A);
    usleep(100);
    
    // Restore config
    write_reg(base, MMCM_CLK_CONFIG0_REG, clk_conf0);
    write_reg(base, MMCM_CLKOUT0_REG, clkout0);
    
    // Check lock
    uint32_t status = read_reg(base, MMCM_STATUS_REG);
    return (status & 0x00000001) != 0;
}

bool ClockWizard::reset_mmcm(rfdc::TileType type, uint32_t tile_id) {
    return reset_hw(type, tile_id);
}

ClockWizard::MMCMConfig ClockWizard::get_mmcm_config(rfdc::TileType type, uint32_t tile_id) {
    MMCMConfig config = {};
    
    void* base = get_clk_wiz_base(type, tile_id);
    if (!base) {
        return config;
    }
    
    // Read lock status
    uint32_t status = read_reg(base, MMCM_STATUS_REG);
    config.locked = (status & 0x00000001) != 0;
    
    // Read CLK_CONFIG0
    uint32_t clk_conf0 = read_reg(base, MMCM_CLK_CONFIG0_REG);
    config.mult_frac = (clk_conf0 >> 16) & 0x03FF;
    config.mult = (clk_conf0 >> 8) & 0x00FF;
    config.div = clk_conf0 & 0x00FF;
    
    // Read CLKOUT0
    uint32_t clkout0 = read_reg(base, MMCM_CLKOUT0_REG);
    config.clkout0_div = clkout0 & 0x00FF;
    config.clkout0_frac = (clkout0 >> 8) & 0x03FF;
    
    // Read CLKOUT1
    uint32_t clkout1 = read_reg(base, MMCM_CLKOUT1_REG);
    config.clkout1_div = clkout1 & 0x00FF;
    
    // Calculate frequencies
    uint32_t idx = (type == rfdc::TileType::DAC) ? (4 + tile_id) : tile_id;
    config.fin_mhz = mmcm_fin_[idx] / 1000.0;
    
    if (config.div > 0) {
        config.fvco_mhz = (config.fin_mhz * config.mult) / config.div;
        if (config.clkout0_div > 0) {
            config.fout_mhz = config.fvco_mhz / config.clkout0_div;
        }
    }
    
    return config;
}

uint32_t ClockWizard::get_mmcm_fin_khz(rfdc::TileType type, uint32_t tile_id) const {
    uint32_t idx = (type == rfdc::TileType::DAC) ? (4 + tile_id) : tile_id;
    return mmcm_fin_[idx];
}

} // namespace clock_wizard
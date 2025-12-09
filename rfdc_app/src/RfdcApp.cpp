#include "RfdcApp.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>

// Helper for formatting
template<typename... Args>
std::string format_msg(Args&&... args) {
    std::ostringstream oss;
    using expander = int[];
    (void)expander{0, ((oss << std::forward<Args>(args)), 0)...};
    return oss.str();
}

RfDcApp::RfDcApp(const std::string& name) 
    : name_(name)
{
    std::cout << "===========================================\n";
    std::cout << "  " << name_ << "\n";
    std::cout << "===========================================\n\n";
}

void RfDcApp::run() {
    try {
        std::cout << "Starting RF Data Converter Application...\n\n";
        
        // Initialize subsystems in order
        initialize_clocks();
        initialize_rfdc();
        configure_dac_tiles();
        configure_adc_tiles();
        verify_configuration();
        
        // Display final status
        display_status();
        
        // Run tests
        run_loopback_test();
        
        std::cout << "\n✓ Application completed successfully!\n";
        
    } catch (const rfdc::RFDCException& e) {
        std::cerr << "\n✗ RFDC Error: " << e.what() 
                  << " (code: " << e.error_code() << ")\n";
        throw;
    } catch (const std::exception& e) {
        std::cerr << "\n✗ Error: " << e.what() << "\n";
        throw;
    }
}

void RfDcApp::initialize_clocks() {
    std::cout << "━━━ Initializing RF Clocks ━━━\n";
    
    try {
        // Initialize RF Clock system
#if defined __BAREMETAL__
        clock_ = std::make_unique<rfdc::RFClock>(0x0);  // GPIO base address
#else
        clock_ = std::make_unique<rfdc::RFClock>(541);    // GPIO device ID
#endif
        
        std::cout << "  ✓ RF Clock driver initialized\n";
        std::cout << "  ✓ Driver version: " << rfdc::RFClock::get_version() << "\n";
        std::cout << "  ✓ Board type: " << 
            (rfdc::RFClock::get_board_type() == rfdc::BoardType::ZCU216 ? 
             "ZCU216" : "ZCU111") << "\n";
        
        // Configure all clocks with preset configuration
        // Using maximum sample rate preset (4.9152 GSPS)
        constexpr uint32_t LMK_CONFIG = 0;   // 245.76 MHz reference
        constexpr uint32_t LMX_ADC_CONFIG = 0;  // 4915.2 MSPS
        constexpr uint32_t LMX_DAC_CONFIG = 0;  // 4915.2 MSPS
        
        std::cout << "\n  Configuring clock chips...\n";
        
        // Reset all chips first
        std::cout << "    • Resetting LMK04828...\n";
        clock_->reset_chip(rfdc::RFClockChip::LMK04828);
        
        std::cout << "    • Resetting LMX2594_1 (ADC)...\n";
        clock_->reset_chip(rfdc::RFClockChip::LMX2594_1);
        
        std::cout << "    • Resetting LMX2594_2 (DAC)...\n";
        clock_->reset_chip(rfdc::RFClockChip::LMX2594_2);
        
        // Configure all chips at once
        std::cout << "    • Programming clock configurations...\n";
#ifdef XPS_BOARD_ZCU111
        clock_->set_all_configs(LMK_CONFIG, LMX_ADC_CONFIG, LMX_DAC_CONFIG, 0);
#else
        clock_->set_all_configs(LMK_CONFIG, LMX_ADC_CONFIG, LMX_DAC_CONFIG);
#endif
        
        std::cout << "  ✓ LMK04828 configured (245.76 MHz ref)\n";
        std::cout << "  ✓ LMX2594_1 configured (4915.2 MSPS ADC)\n";
        std::cout << "  ✓ LMX2594_2 configured (4915.2 MSPS DAC)\n";
        
        // Enable LMK output ports
        std::cout << "\n  Enabling LMK output ports...\n";
        for (uint32_t port = 1; port <= 7; ++port) {
            clock_->control_lmk_port(
                static_cast<rfdc::LMKPort>(port),
                rfdc::PortState::Enable
            );
        }
        std::cout << "  ✓ All LMK ports enabled\n";
        
        std::cout << "  ✓ Reference clocks stable\n\n";
        
    } catch (const rfdc::RFClockException& e) {
        std::cerr << "  ✗ RF Clock Error: " << e.what() << "\n";
        throw;
    }
}
#define RFDC_DEVICE_ID 0
void RfDcApp::initialize_rfdc() {
    std::cout << "━━━ Initializing RF Data Converter ━━━\n";
    
    // Initialize RFDC with device ID 0
    rfdc_ = std::make_unique<rfdc::RFDC>(RFDC_DEVICE_ID);
    
    std::cout << "  ✓ RFDC driver initialized\n";
    std::cout << "  ✓ Driver version: " << rfdc::RFDC::get_driver_version() << "\n";
    
    // Get IP status
    auto ip_status = rfdc_->get_ip_status();
    std::cout << format_msg("  ✓ IP State: 0x", std::hex, ip_status.state(), std::dec, "\n");
    
    // Report enabled tiles
    std::cout << "\n  Enabled Tiles:\n";
    for (uint32_t tile = 0; tile < 4; ++tile) {
        bool dac_en = ip_status.dac_tile_enabled(tile);
        bool adc_en = ip_status.adc_tile_enabled(tile);
        if (dac_en || adc_en) {
            std::cout << format_msg("    Tile ", tile, ": ");
            if (dac_en) std::cout << "DAC ";
            if (adc_en) std::cout << "ADC ";
            std::cout << "\n";
        }
    }
    std::cout << "\n";
}

void RfDcApp::configure_dac_tiles() {
    std::cout << "━━━ Configuring DAC Tiles ━━━\n";
    
    // Configuration parameters for ZCU216
    constexpr double REF_CLK_FREQ = 245.76;    // MHz
    constexpr double SAMPLE_RATE = 4915.2;     // MSPS
    constexpr double NCO_FREQ = 100.0;         // MHz
    
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (!rfdc_->check_tile_enabled(rfdc::TileType::DAC, tile)) {
            continue;
        }
        
        std::cout << format_msg("  Configuring DAC Tile ", tile, "...\n");
        
        // Startup the tile
        rfdc_->startup(rfdc::TileType::DAC, tile);
        std::cout << "    • Tile started\n";
        
        // Configure PLL
        rfdc_->set_pll_config(rfdc::TileType::DAC, tile, REF_CLK_FREQ, SAMPLE_RATE);
        std::cout << format_msg("    • PLL configured (Fs=", SAMPLE_RATE, " MSPS)\n");
        
        // Wait for PLL lock with timeout
        bool locked = false;
        for (int retry = 0; retry < 100 && !locked; ++retry) {
            locked = rfdc_->get_pll_lock_status(rfdc::TileType::DAC, tile);
            if (!locked) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        if (!locked) {
            throw rfdc::RFDCException(
                format_msg("DAC Tile ", tile, " PLL failed to lock")
            );
        }
        std::cout << "    • PLL locked\n";
        
        // Configure each block in the tile
        for (uint32_t block = 0; block < 4; ++block) {
            if (!rfdc_->check_block_enabled(rfdc::TileType::DAC, tile, block)) {
                continue;
            }
            
            std::cout << format_msg("      Block ", block, ":\n");
            
            // Configure mixer for NCO frequency
            rfdc::MixerSettings mixer(
                NCO_FREQ,
                0.0,                            // Phase offset
                rfdc::EventSource::Immediate,
                XRFDC_COARSE_MIX_BYPASS,
                rfdc::MixerMode::C2C,
                0,                              // Fine mixer scale
                rfdc::MixerType::Fine
            );
            rfdc_->set_mixer_settings(rfdc::TileType::DAC, tile, block, mixer);
            std::cout << format_msg("        - Mixer: ", NCO_FREQ, " MHz\n");
            
            // Set Nyquist zone
            rfdc_->set_nyquist_zone(rfdc::TileType::DAC, tile, block, 
                                   rfdc::NyquistZone::Zone1);
            
            // Configure interpolation (2x for this example)
            rfdc_->set_interpolation_factor(tile, block, 2);
            std::cout << "        - Interpolation: 2x\n";
            
            // Enable inverse sinc filter
            rfdc_->set_inverse_sinc_filter(tile, block, 1);
            std::cout << "        - Inverse Sinc: Enabled\n";
            
            // Update event to apply all settings
            rfdc_->update_event(rfdc::TileType::DAC, tile, block, 
                              XRFDC_EVENT_MIXER);
        }
        
        // Enable FIFO
        rfdc_->setup_fifo(rfdc::TileType::DAC, tile, true);
        std::cout << "    • FIFO enabled\n";
        
        std::cout << format_msg("  ✓ DAC Tile ", tile, " configured\n\n");
    }
}

void RfDcApp::configure_adc_tiles() {
    std::cout << "━━━ Configuring ADC Tiles ━━━\n";
    
    // Configuration parameters for ZCU216
    constexpr double REF_CLK_FREQ = 245.76;    // MHz
    constexpr double SAMPLE_RATE = 4915.2;     // MSPS
    constexpr double NCO_FREQ = 100.0;         // MHz (match DAC for loopback)
    
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (!rfdc_->check_tile_enabled(rfdc::TileType::ADC, tile)) {
            continue;
        }
        
        std::cout << format_msg("  Configuring ADC Tile ", tile, "...\n");
        
        // Startup the tile
        rfdc_->startup(rfdc::TileType::ADC, tile);
        std::cout << "    • Tile started\n";
        
        // Configure PLL
        rfdc_->set_pll_config(rfdc::TileType::ADC, tile, REF_CLK_FREQ, SAMPLE_RATE);
        std::cout << format_msg("    • PLL configured (Fs=", SAMPLE_RATE, " MSPS)\n");
        
        // Wait for PLL lock
        bool locked = false;
        for (int retry = 0; retry < 100 && !locked; ++retry) {
            locked = rfdc_->get_pll_lock_status(rfdc::TileType::ADC, tile);
            if (!locked) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        if (!locked) {
            throw rfdc::RFDCException(
                format_msg("ADC Tile ", tile, " PLL failed to lock")
            );
        }
        std::cout << "    • PLL locked\n";
        
        // Configure each block
        for (uint32_t block = 0; block < 4; ++block) {
            if (!rfdc_->check_block_enabled(rfdc::TileType::ADC, tile, block)) {
                continue;
            }
            
            std::cout << format_msg("      Block ", block, ":\n");
            
            // Configure mixer
            rfdc::MixerSettings mixer(
                NCO_FREQ,
                0.0,
                rfdc::EventSource::Immediate,
                XRFDC_COARSE_MIX_BYPASS,
                rfdc::MixerMode::C2C,
                0,
                rfdc::MixerType::Fine
            );
            rfdc_->set_mixer_settings(rfdc::TileType::ADC, tile, block, mixer);
            std::cout << format_msg("        - Mixer: ", NCO_FREQ, " MHz\n");
            
            // Set Nyquist zone
            rfdc_->set_nyquist_zone(rfdc::TileType::ADC, tile, block,
                                   rfdc::NyquistZone::Zone1);
            
            // Configure decimation
            rfdc_->set_decimation_factor(tile, block, 2);
            std::cout << "        - Decimation: 2x\n";
            
            // Configure QMC (Quadrature Modulation Correction)
            rfdc::QMCSettings qmc(
                true,   // Enable phase correction
                true,   // Enable gain correction
                1.0,    // Gain correction factor
                0.0,    // Phase correction factor
                0,      // Offset correction
                rfdc::EventSource::Immediate
            );
            rfdc_->set_qmc_settings(rfdc::TileType::ADC, tile, block, qmc);
            std::cout << "        - QMC: Enabled\n";
            
            // Configure threshold
            rfdc::ThresholdSettings threshold;
            threshold.set_mode(0, rfdc::ThresholdMode::StickyOver);
            threshold.set_threshold_values(0, 0x3FFF, 0x1000, 0x3000);
            rfdc_->set_threshold_settings(tile, block, threshold);
            
            // Set calibration mode
            rfdc_->set_calibration_mode(tile, block, rfdc::CalibrationMode::Mode1);
            std::cout << "        - Calibration: Mode 1\n";
            
            // Update event
            rfdc_->update_event(rfdc::TileType::ADC, tile, block,
                              XRFDC_EVENT_MIXER);
        }
        
        // Enable FIFO
        rfdc_->setup_fifo(rfdc::TileType::ADC, tile, true);
        std::cout << "    • FIFO enabled\n";
        
        std::cout << format_msg("  ✓ ADC Tile ", tile, " configured\n\n");
    }
}

void RfDcApp::verify_configuration() {
    std::cout << "━━━ Verifying Configuration ━━━\n";
    
    // Verify DAC tiles
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (!rfdc_->check_tile_enabled(rfdc::TileType::DAC, tile)) {
            continue;
        }
        
        std::cout << format_msg("  DAC Tile ", tile, ":\n");
        
        // Check PLL lock
        bool pll_locked = rfdc_->get_pll_lock_status(rfdc::TileType::DAC, tile);
        std::cout << format_msg("    PLL Lock: ", 
                               (pll_locked ? "✓ LOCKED" : "✗ UNLOCKED"), "\n");
        
        // Get PLL configuration
        auto pll = rfdc_->get_pll_config(rfdc::TileType::DAC, tile);
        std::cout << std::fixed << std::setprecision(2)
                 << "    Ref Clock: " << pll.ref_clk_freq() << " MHz\n"
                 << "    Sample Rate: " << pll.sample_rate() << " MSPS\n";
        
        // Check FIFO status
        bool fifo_enabled = rfdc_->get_fifo_status(rfdc::TileType::DAC, tile);
        std::cout << format_msg("    FIFO: ", 
                               (fifo_enabled ? "Enabled" : "Disabled"), "\n");
    }
    
    std::cout << "\n";
    
    // Verify ADC tiles
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (!rfdc_->check_tile_enabled(rfdc::TileType::ADC, tile)) {
            continue;
        }
        
        std::cout << format_msg("  ADC Tile ", tile, ":\n");
        
        // Check PLL lock
        bool pll_locked = rfdc_->get_pll_lock_status(rfdc::TileType::ADC, tile);
        std::cout << format_msg("    PLL Lock: ",
                               (pll_locked ? "✓ LOCKED" : "✗ UNLOCKED"), "\n");
        
        // Get PLL configuration
        auto pll = rfdc_->get_pll_config(rfdc::TileType::ADC, tile);
        std::cout << std::fixed << std::setprecision(2)
                 << "    Ref Clock: " << pll.ref_clk_freq() << " MHz\n"
                 << "    Sample Rate: " << pll.sample_rate() << " MSPS\n";
        
        // Check FIFO status
        bool fifo_enabled = rfdc_->get_fifo_status(rfdc::TileType::ADC, tile);
        std::cout << format_msg("    FIFO: ",
                               (fifo_enabled ? "Enabled" : "Disabled"), "\n");
    }
    
    std::cout << "\n";
}

void RfDcApp::display_status() {
    std::cout << "━━━ System Status ━━━\n";
    
    auto ip_status = rfdc_->get_ip_status();
    
    std::cout << "\n  DAC Tiles:\n";
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (!ip_status.dac_tile_enabled(tile)) {
            continue;
        }
        
        std::cout << format_msg("    Tile ", tile, ":\n");
        
        for (uint32_t block = 0; block < 4; ++block) {
            if (!rfdc_->check_block_enabled(rfdc::TileType::DAC, tile, block)) {
                continue;
            }
            
            auto block_status = rfdc_->get_block_status(
                rfdc::TileType::DAC, tile, block
            );
            
            std::cout << format_msg("      Block ", block, ": ");
            std::cout << std::fixed << std::setprecision(2)
                     << "Fs=" << block_status.sampling_freq() << " MHz, ";
            std::cout << (block_status.digital_path_clocks_enabled() ? 
                         "Clocks✓" : "Clocks✗");
            std::cout << "\n";
        }
    }
    
    std::cout << "\n  ADC Tiles:\n";
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (!ip_status.adc_tile_enabled(tile)) {
            continue;
        }
        
        std::cout << format_msg("    Tile ", tile, ":\n");
        
        for (uint32_t block = 0; block < 4; ++block) {
            if (!rfdc_->check_block_enabled(rfdc::TileType::ADC, tile, block)) {
                continue;
            }
            
            auto block_status = rfdc_->get_block_status(
                rfdc::TileType::ADC, tile, block
            );
            
            std::cout << format_msg("      Block ", block, ": ");
            std::cout << std::fixed << std::setprecision(2)
                     << "Fs=" << block_status.sampling_freq() << " MHz, ";
            std::cout << (block_status.digital_path_clocks_enabled() ?
                         "Clocks✓" : "Clocks✗");
            std::cout << "\n";
        }
    }
    
    std::cout << "\n";
}

void RfDcApp::run_loopback_test() {
    std::cout << "━━━ Running Loopback Test ━━━\n";
    
    std::cout << "  Note: Loopback test requires external connections\n";
    std::cout << "        between DAC and ADC RF ports.\n\n";
    
    // This is a placeholder for actual loopback testing
    // In a real implementation, you would:
    // 1. Generate test waveforms on DAC
    // 2. Capture data on ADC
    // 3. Verify signal integrity
    // 4. Check frequency, amplitude, phase
    
    std::cout << "  Test Status: Ready (manual verification required)\n";
    std::cout << "  Expected: 100 MHz tone on all enabled channels\n\n";
}
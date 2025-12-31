#include "RfdcApp.hpp"
#include <cstdint>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cstring>

#define _USE_MATH_DEFINES
#include <cmath>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// Fallback: Define M_PI if still not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define static const members
constexpr size_t RfDcApp::DAC_MAP_SZ;
constexpr size_t RfDcApp::ADC_MAP_SZ;
constexpr size_t RfDcApp::FIFO_SIZE;
constexpr size_t RfDcApp::ADC_DAC_SZ_ALIGNMENT;
constexpr uint16_t RfDcApp::RFDC_DEVICE_ID;

// Define static const GPIO arrays
const int RfDcApp::dac_userselect_gpio[RfDcApp::MAX_DAC_PER_TILE * RfDcApp::MAX_DAC_TILE] = {
    517, 518, 519, 520, 521, 522, 523, 524,
    525, 526, 527, 528, 529, 530, 531, 532,
};

const int RfDcApp::dac_mts_clk_en[RfDcApp::MAX_DAC_TILE] = {
    533, 534, 535, 536,
};

const int RfDcApp::adc_mts_clk_en[RfDcApp::MAX_ADC_TILE] = {
    537, 538, 539, 540,
};

const int RfDcApp::adc_axiswitchrst = 516;

// Paths for UIO devices (matching RFTool exactly)
static const char* mem_path_dac[16] = {
    "/dev/plmem0",  "/dev/plmem1",  "/dev/plmem2",  "/dev/plmem3",
    "/dev/plmem4",  "/dev/plmem5",  "/dev/plmem6",  "/dev/plmem7",
    "/dev/plmem8",  "/dev/plmem9",  "/dev/plmem10", "/dev/plmem11",
    "/dev/plmem12", "/dev/plmem13", "/dev/plmem14", "/dev/plmem15",
};

static const char* mem_path_adc[16] = {
    "/dev/plmem16", "/dev/plmem17", "/dev/plmem18", "/dev/plmem19",
    "/dev/plmem20", "/dev/plmem21", "/dev/plmem22", "/dev/plmem23",
    "/dev/plmem24", "/dev/plmem25", "/dev/plmem26", "/dev/plmem27",
    "/dev/plmem28", "/dev/plmem29", "/dev/plmem30", "/dev/plmem31",
};

static const char* mem_type_path_dac[16] = {
    "/sys/class/plmem/plmem0/device/select_mem",
    "/sys/class/plmem/plmem1/device/select_mem",
    "/sys/class/plmem/plmem2/device/select_mem",
    "/sys/class/plmem/plmem3/device/select_mem",
    "/sys/class/plmem/plmem4/device/select_mem",
    "/sys/class/plmem/plmem5/device/select_mem",
    "/sys/class/plmem/plmem6/device/select_mem",
    "/sys/class/plmem/plmem7/device/select_mem",
    "/sys/class/plmem/plmem8/device/select_mem",
    "/sys/class/plmem/plmem9/device/select_mem",
    "/sys/class/plmem/plmem10/device/select_mem",
    "/sys/class/plmem/plmem11/device/select_mem",
    "/sys/class/plmem/plmem12/device/select_mem",
    "/sys/class/plmem/plmem13/device/select_mem",
    "/sys/class/plmem/plmem14/device/select_mem",
    "/sys/class/plmem/plmem15/device/select_mem",
};

static const char* mem_type_path_adc[16] = {
    "/sys/class/plmem/plmem16/device/select_mem",
    "/sys/class/plmem/plmem17/device/select_mem",
    "/sys/class/plmem/plmem18/device/select_mem",
    "/sys/class/plmem/plmem19/device/select_mem",
    "/sys/class/plmem/plmem20/device/select_mem",
    "/sys/class/plmem/plmem21/device/select_mem",
    "/sys/class/plmem/plmem22/device/select_mem",
    "/sys/class/plmem/plmem23/device/select_mem",
    "/sys/class/plmem/plmem24/device/select_mem",
    "/sys/class/plmem/plmem25/device/select_mem",
    "/sys/class/plmem/plmem26/device/select_mem",
    "/sys/class/plmem/plmem27/device/select_mem",
    "/sys/class/plmem/plmem28/device/select_mem",
    "/sys/class/plmem/plmem29/device/select_mem",
    "/sys/class/plmem/plmem30/device/select_mem",
    "/sys/class/plmem/plmem31/device/select_mem",
};

static const char* bram_ddr_path_dac[16] = {
    "/sys/class/plmem/plmem0/device/mem_type",
    "/sys/class/plmem/plmem1/device/mem_type",
    "/sys/class/plmem/plmem2/device/mem_type",
    "/sys/class/plmem/plmem3/device/mem_type",
    "/sys/class/plmem/plmem4/device/mem_type",
    "/sys/class/plmem/plmem5/device/mem_type",
    "/sys/class/plmem/plmem6/device/mem_type",
    "/sys/class/plmem/plmem7/device/mem_type",
    "/sys/class/plmem/plmem8/device/mem_type",
    "/sys/class/plmem/plmem9/device/mem_type",
    "/sys/class/plmem/plmem10/device/mem_type",
    "/sys/class/plmem/plmem11/device/mem_type",
    "/sys/class/plmem/plmem12/device/mem_type",
    "/sys/class/plmem/plmem13/device/mem_type",
    "/sys/class/plmem/plmem14/device/mem_type",
    "/sys/class/plmem/plmem15/device/mem_type",
};

static const char* bram_ddr_path_adc[16] = {
    "/sys/class/plmem/plmem16/device/mem_type",
    "/sys/class/plmem/plmem17/device/mem_type",
    "/sys/class/plmem/plmem18/device/mem_type",
    "/sys/class/plmem/plmem19/device/mem_type",
    "/sys/class/plmem/plmem20/device/mem_type",
    "/sys/class/plmem/plmem21/device/mem_type",
    "/sys/class/plmem/plmem22/device/mem_type",
    "/sys/class/plmem/plmem23/device/mem_type",
    "/sys/class/plmem/plmem24/device/mem_type",
    "/sys/class/plmem/plmem25/device/mem_type",
    "/sys/class/plmem/plmem26/device/mem_type",
    "/sys/class/plmem/plmem27/device/mem_type",
    "/sys/class/plmem/plmem28/device/mem_type",
    "/sys/class/plmem/plmem29/device/mem_type",
    "/sys/class/plmem/plmem30/device/mem_type",
    "/sys/class/plmem/plmem31/device/mem_type",
};

// Constants matching RFTool
constexpr int SUCCESS = 0;
constexpr int FAIL = -1;
constexpr const char* PL_MEM = "0x1";
constexpr const char* NO_MEM = "0x0";
constexpr const char* BRAM = "0";

// Helper for formatting
template<typename... Args>
std::string format_msg(Args&&... args) {
    std::ostringstream oss;
    using expander = int[];
    (void)expander{0, ((oss << std::forward<Args>(args)), 0)...};
    return oss.str();
}

// Helper function to deinitialize a single path (matching RFTool)
static int deinit_path(int* fd, signed char* map, size_t sz) {
    if (*fd != 0) {
        fsync(*fd);
    }
    
    if (map != nullptr && map != MAP_FAILED) {
        if (munmap(map, sz) == -1) {
            std::cerr << "    ⚠ Warning: unmap failed\n";
        }
    }
    
    if (*fd != 0 && *fd != -1) {
        close(*fd);
        *fd = 0;
    }
    
    return SUCCESS;
}

RfDcApp::RfDcApp(const std::string& name) 
    : name_(name)
{
    std::cout << "===========================================\n";
    std::cout << "  " << name_ << "\n";
    std::cout << "===========================================\n\n";
    
    // Initialize info structure
    memset(&info_, 0, sizeof(info_));
    for (int i = 0; i < 16; i++) {
        info_.fd_dac[i] = -1;
        info_.fd_adc[i] = -1;
        info_.map_dac[i] = nullptr;
        info_.map_adc[i] = nullptr;
    }
    info_.fd = -1;
}

void RfDcApp::run() 
{
    try 
    {
        std::cout << "Starting RF Data Converter Application...\n\n";
        
        // Initialize GPIO first
        init_gpio();
        // Initialize clocks
        initialize_clocks();
        // Initialize RFDC
        initialize_rfdc();
        // Initialize LocalMem controller
        local_mem_ = std::make_unique<local_mem::LocalMem>(rfdc_.get());
        // Initialize the Clock Wizard 
        clock_wiz_ = std::make_unique<clock_wizard::ClockWizard>(rfdc_.get()); 
        // Initialize memory mapping (for clock wizards)
        initialize_memory_mapping();
        if (init_mem() != SUCCESS)
        {
            throw std::runtime_error("Failed to initialize UIO memory");
        }
        // Configure tiles
        configure_dac_tiles();
        configure_adc_tiles();
        initialize_mmcm_adc();
        initialize_mmcm_dac();

        verify_configuration();
        display_status();
        // Run tests
        //run_loopback_test();
        run_iq_loopback_test();
        //run_codec_diagnostic_test();
        //run_string_loopback_test();
        std::cout << "\n✓ Application completed successfully!\n";
        
        // Cleanup
        deinit_mem();
        deinit_gpio();
        
    } 
    catch (const rfdc::RFDCException& e) 
    {
        std::cerr << "\n✗ RFDC Error: " << e.what() 
                  << " (code: " << e.error_code() << ")\n";
        deinit_mem();
        deinit_gpio();
        throw;
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "\n✗ Error: " << e.what() << "\n";
        deinit_mem();
        deinit_gpio();
        throw;
    }
}

void RfDcApp::run_codec_diagnostic_test()
{
    std::cout << "━━━ Codec Diagnostic Test ━━━\n";
    std::cout << "  Testing encode/decode chain WITHOUT RF hardware\n\n";
    
    const std::string test_msg = "HELLO";
    
    // Show what "HELLO" looks like in hex
    std::cout << "Reference: \"HELLO\" in hex:\n  ";
    for (char c : test_msg) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(static_cast<uint8_t>(c)) << " ";
    }
    std::cout << std::dec << "\n\n";
    
    // Test configuration
    codec::StringCodec::Config config;
    config.modulation = codec::StringCodec::ModulationType::BPSK;
    config.samples_per_symbol = 64;
    config.amplitude = 3000;
    config.use_crc = true;
    config.use_preamble = true;
    
    codec::StringCodec codec(config);
    
    // ===== TEST 1: Perfect loopback (no noise) =====
    std::cout << "Test 1: Perfect loopback (no noise)\n";
    
    std::vector<int16_t> tx_samples = codec.encode_real(test_msg);
    std::cout << "  TX: Encoded \"" << test_msg << "\" to " 
              << tx_samples.size() << " samples\n\n";
    
    std::string decoded;
    bool success = codec.decode_real(tx_samples, decoded);
    
    if (success && decoded == test_msg) {
        std::cout << "  ✓✓✓ PASS: Decoded \"" << decoded << "\"\n\n";
    } else {
        std::cout << "  ✗ FAIL: Expected \"" << test_msg 
                 << "\", got \"" << decoded << "\"\n";
        std::cout << "  ⚠️ CODEC BUG - software issue, not RF!\n\n";
        
        // Show bit-level comparison
        if (!decoded.empty()) {
            std::cout << "  Bit comparison:\n";
            std::cout << "    Expected (hex): ";
            for (char c : test_msg) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') 
                          << static_cast<int>(static_cast<uint8_t>(c)) << " ";
            }
            std::cout << "\n    Got (hex):      ";
            for (char c : decoded) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') 
                          << static_cast<int>(static_cast<uint8_t>(c)) << " ";
            }
            std::cout << std::dec << "\n\n";
        }
        return;
    }
    
    // ===== TEST 2: With added noise =====
    std::cout << "Test 2: With 1% noise\n";
    
    std::vector<int16_t> noisy_samples = tx_samples;
    std::default_random_engine rng(42);
    std::normal_distribution<double> noise(0.0, config.amplitude * 0.01);
    
    for (auto& s : noisy_samples) {
        s += static_cast<int16_t>(noise(rng));
        // Clamp
        if (s > 32767) s = 32767;
        if (s < -32768) s = -32768;
    }
    
    success = codec.decode_real(noisy_samples, decoded);
    
    if (success && decoded == test_msg) {
        std::cout << "  ✓✓✓ PASS: Still decoded correctly\n\n";
    } else {
        std::cout << "  ✗ FAIL: Noise broke decoder\n";
        std::cout << "  Expected: \"" << test_msg << "\"\n";
        std::cout << "  Got:      \"" << decoded << "\"\n\n";
    }
    
    // ===== TEST 3: With amplitude scaling =====
    std::cout << "Test 3: With 50% amplitude reduction\n";
    
    std::vector<int16_t> scaled_samples;
    for (auto s : tx_samples) {
        scaled_samples.push_back(s / 2);
    }
    
    success = codec.decode_real(scaled_samples, decoded);
    
    if (success && decoded == test_msg) {
        std::cout << "  ✓✓✓ PASS: Handles amplitude variation\n\n";
    } else {
        std::cout << "  ✗ FAIL: Can't handle amplitude changes\n";
        std::cout << "  This suggests RF path has gain/attenuation issues\n\n";
    }
    
    // ===== TEST 4: With DC offset =====
    std::cout << "Test 4: With DC offset (+1000)\n";
    
    std::vector<int16_t> dc_offset_samples;
    for (auto s : tx_samples) {
        dc_offset_samples.push_back(s + 1000);
    }
    
    success = codec.decode_real(dc_offset_samples, decoded);
    
    if (success && decoded == test_msg) {
        std::cout << "  ✓✓✓ PASS: Handles DC offset\n\n";
    } else {
        std::cout << "  ⚠ FAIL: DC offset breaks decoder\n";
        std::cout << "  Check ADC DC offset in RF path\n\n";
    }
    
    // ===== TEST 5: Timing offset =====
    std::cout << "Test 5: With timing offset (5 samples delay)\n";
    
    std::vector<int16_t> delayed_samples(5, 0);
    delayed_samples.insert(delayed_samples.end(), 
                          tx_samples.begin(), 
                          tx_samples.end());
    
    success = codec.decode_real(delayed_samples, decoded);
    
    if (success && decoded == test_msg) {
        std::cout << "  ✓✓✓ PASS: Handles timing offset\n\n";
    } else {
        std::cout << "  ✗ FAIL: Timing offset breaks decoder\n";
        std::cout << "  Preamble detection may need tuning\n\n";
    }
    
    // ===== SUMMARY =====
    std::cout << "━━━ Diagnostic Summary ━━━\n";
    std::cout << "If ALL tests pass:\n";
    std::cout << "  → Codec is working correctly\n";
    std::cout << "  → Problem is in RF path (cables, gains, timing)\n\n";
    
    std::cout << "If tests FAIL:\n";
    std::cout << "  → Fix codec implementation first\n";
    std::cout << "  → Don't proceed to RF testing until software works\n\n";
}

// ===== Helper Functions =====

int RfDcApp::write_to_file(const std::string& path, const std::string& value) {
    std::ofstream file(path);
    if (!file.is_open()) {
        return FAIL;
    }
    file << value;
    file.close();
    return SUCCESS;
}

// ===== Memory Initialization (RFTool Compatible API) =====

int RfDcApp::init_mem() {
    std::cout << "━━━ Initializing Memory (UIO) ━━━\n";
    
    int i, ret;
    
    // Open /dev/mem for BRAM access
    info_.fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (info_.fd < 0) {
        std::cerr << "  ✗ Failed to open /dev/mem\n";
        return FAIL;
    }
    std::cout << "  ✓ Opened /dev/mem\n";
    
    // Initialize DAC memory paths
    std::cout << "  Initializing DAC UIO devices...\n";
    for (i = 0; i < 16; i++) {
        // Set memory type to PL_MEM
        ret = write_to_file(mem_type_path_dac[i], PL_MEM);
        if (ret != SUCCESS) {
            std::cerr << "  ✗ Error configuring memory for " << mem_path_dac[i] << "\n";
            return FAIL;
        }
        
        // Open memory file
        info_.fd_dac[i] = open(mem_path_dac[i], O_RDWR);
        if (info_.fd_dac[i] < 0) {
            std::cerr << "  ✗ File " << mem_path_dac[i] << " open failed\n";
            return FAIL;
        }
        
        // Map size of memory
        info_.map_dac[i] = (signed char*)mmap(0, DAC_MAP_SZ,
                                              PROT_READ | PROT_WRITE,
                                              MAP_SHARED,
                                              info_.fd_dac[i], 0);
        if (info_.map_dac[i] == MAP_FAILED) {
            std::cerr << "  ✗ Error mmapping the file " << i << "\n";
            return FAIL;
        }
        
        std::cout << "    ✓ Mapped " << mem_path_dac[i] << "\n";
    }
    
    // Initialize ADC memory paths
    std::cout << "  Initializing ADC UIO devices...\n";
    for (i = 0; i < 16; i++) {
        // Set memory type to PL_MEM
        ret = write_to_file(mem_type_path_adc[i], PL_MEM);
        if (ret != SUCCESS) {
            std::cerr << "  ✗ Error configuring memory for " << mem_path_adc[i] << "\n";
            return FAIL;
        }
        
        // Open memory file
        info_.fd_adc[i] = open(mem_path_adc[i], O_RDWR);
        if (info_.fd_adc[i] < 0) {
            std::cerr << "  ✗ File " << mem_path_adc[i] << " open failed\n";
            return FAIL;
        }
        
        // Map size of memory
        info_.map_adc[i] = (signed char*)mmap(0, ADC_MAP_SZ,
                                              PROT_READ | PROT_WRITE,
                                              MAP_SHARED,
                                              info_.fd_adc[i], 0);
        if (info_.map_adc[i] == MAP_FAILED) {
            std::cerr << "  ✗ Error mmapping the file " << i << "\n";
            return FAIL;
        }
        
        std::cout << "    ✓ Mapped " << mem_path_adc[i] << "\n";
    }
    
    // Set all 4 tiles in BRAM mode
    info_.mem_type_dac = 0xF;
    info_.mem_type_adc = 0xF;
    
    std::cout << "  ✓ Memory initialization complete\n";
    std::cout << "    - Memory type DAC: 0x" << std::hex << info_.mem_type_dac << std::dec << " (BRAM)\n";
    std::cout << "    - Memory type ADC: 0x" << std::hex << info_.mem_type_adc << std::dec << " (BRAM)\n\n";
    
    return SUCCESS;
}

int RfDcApp::deinit_mem() {
    std::cout << "━━━ De-initializing Memory (UIO) ━━━\n";
    
    int i, ret;
    
    // Cleanup DAC memory paths
    std::cout << "  Cleaning up DAC UIO devices...\n";
    for (i = 0; i < 16; i++) {
        // Set memory type back to NO_MEM
        ret = write_to_file(mem_type_path_dac[i], NO_MEM);
        if (ret != SUCCESS) {
            std::cerr << "    ⚠ Error configuring dac memory: DAC mem index: " << i << "\n";
        }
        
        // Set mem_type back to BRAM
        ret = write_to_file(bram_ddr_path_dac[i], BRAM);
        if (ret != SUCCESS) {
            std::cerr << "    ⚠ Error configuring dac memory: DAC mem index: " << i << "\n";
        }
        
        // Deinitialize path
        deinit_path(&info_.fd_dac[i], info_.map_dac[i], DAC_MAP_SZ);
        std::cout << "    ✓ Cleaned up " << mem_path_dac[i] << "\n";
    }
    
    // Cleanup ADC memory paths
    std::cout << "  Cleaning up ADC UIO devices...\n";
    for (i = 0; i < 16; i++) {
        // Set memory type back to NO_MEM
        ret = write_to_file(mem_type_path_adc[i], NO_MEM);
        if (ret != SUCCESS) {
            std::cerr << "    ⚠ Error configuring ADC memory: ADC mem index: " << i << "\n";
        }
        
        // Set mem_type back to BRAM
        ret = write_to_file(bram_ddr_path_adc[i], BRAM);
        if (ret != SUCCESS) {
            std::cerr << "    ⚠ Error configuring dac mem type: mem index: " << i << "\n";
        }
        
        // Deinitialize path
        deinit_path(&info_.fd_adc[i], info_.map_adc[i], ADC_MAP_SZ);
        std::cout << "    ✓ Cleaned up " << mem_path_adc[i] << "\n";
    }
    
    // Close /dev/mem
    if (info_.fd >= 0) {
        close(info_.fd);
        info_.fd = -1;
        std::cout << "  ✓ Closed /dev/mem\n";
    }
    
    std::cout << "  ✓ Memory cleanup complete\n";
    
    return SUCCESS;
}

// ===== UIO Data Transfer Functions =====

int RfDcApp::change_fifo_stat(int fifo_id, int tile_id, int stat) {
    auto type = (fifo_id == 0) ? rfdc::TileType::ADC : rfdc::TileType::DAC;
    try {
        rfdc_->setup_fifo(type, tile_id, stat != 0);
        return SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "  ✗ FIFO control failed: " << e.what() << "\n";
        return FAIL;
    }
}

int RfDcApp::write_data_to_memory_bram(uint32_t block_id, int tile_id,
                                       uint32_t size, const std::vector<int16_t>& samples)
{
    std::cout << "  Writing to DAC[" << tile_id << "][" << block_id << "]...\n";
    
    // Validate size
    if (size > FIFO_SIZE) {
        std::cerr << "    ✗ Size " << size << " exceeds FIFO size " << FIFO_SIZE << "\n";
        return FAIL;
    }
    
    if ((size == 0) || (size % ADC_DAC_SZ_ALIGNMENT) != 0) {
        std::cerr << "    ✗ Size must be multiple of " << ADC_DAC_SZ_ALIGNMENT << " bytes\n";
        return FAIL;
    }
    
    // Get channel index from tile and block
    uint32_t channel = tile_id * 4 + block_id;
    if (channel >= 16) {
        std::cerr << "    ✗ Invalid channel " << channel << "\n";
        return FAIL;
    }
    
    // Get DAC map for this channel
    const auto& dac_map = rfdc_->get_dac_map();
    uint32_t paddr_dac = dac_map[channel].addr_I;
    
    if (paddr_dac == 0xFFFFFFFF) {
        std::cerr << "    ✗ Channel not available\n";
        return FAIL;
    }
    
    // Map BRAM at physical address
    void* bram_base_dac = mmap(nullptr, size, PROT_READ | PROT_WRITE,
                               MAP_SHARED, info_.fd, paddr_dac);
    if (bram_base_dac == MAP_FAILED) {
        std::cerr << "    ✗ Failed to mmap BRAM at 0x" << std::hex << paddr_dac << std::dec << "\n";
        return FAIL;
    }
    
    signed int* vaddr_dac = (signed int*)bram_base_dac;
    
    // Convert int16_t samples to 32-bit format and write
    size_t num_samples = samples.size();
    for (size_t i = 0; i < num_samples / 2; i++) {
        // Pack two 16-bit samples into one 32-bit word
        int16_t sample1 = samples[i * 2];
        int16_t sample2 = (i * 2 + 1 < num_samples) ? samples[i * 2 + 1] : 0;
        vaddr_dac[i] = (sample1 & 0xFFFF) | ((sample2 & 0xFFFF) << 16);
    }
    
    munmap(bram_base_dac, size);
    
    std::cout << "    ✓ Wrote " << num_samples << " samples (" << size << " bytes)\n";
    
    // Enable FIFO for this tile
    change_fifo_stat(1, tile_id, 1);  // 1 = DAC
    
    return SUCCESS;
}

int RfDcApp::read_data_from_memory_bram(
    uint32_t block_id,
    int tile_id,
    uint32_t size_bytes,
    std::vector<int16_t>& samples
)
{
    std::cout << "  Reading ADC[" << tile_id << "][" << block_id << "] (REAL)...\n";

    // ------------------------------------------------------------
    // 1. 参数检查
    // ------------------------------------------------------------
    if (size_bytes == 0 || size_bytes % sizeof(uint32_t) != 0) {
        std::cerr << "    ✗ Size must be multiple of 32-bit words\n";
        return FAIL;
    }

    if (size_bytes > FIFO_SIZE) {
        std::cerr << "    ✗ Size exceeds FIFO size\n";
        return FAIL;
    }

    // ------------------------------------------------------------
    // 2. 启用 ADC FIFO
    // ------------------------------------------------------------
    change_fifo_stat(XRFDC_ADC_TILE, tile_id, 1);

    // ------------------------------------------------------------
    // 3. 计算 BRAM 物理地址
    // ------------------------------------------------------------
    const auto& adc_map = rfdc_->get_adc_map();

    constexpr uint32_t blocks_per_tile = 4;
    uint32_t idx = tile_id * blocks_per_tile + block_id;

    uint32_t paddr_adc = adc_map[idx].addr_I;
    if (paddr_adc == 0xFFFFFFFF) {
        std::cerr << "    ✗ Invalid ADC BRAM address\n";
        return FAIL;
    }

    // ------------------------------------------------------------
    // 4. mmap BRAM
    // ------------------------------------------------------------
    void* bram_base = mmap(
        nullptr,
        size_bytes,
        PROT_READ,
        MAP_SHARED,
        info_.fd,
        paddr_adc
    );

    if (bram_base == MAP_FAILED) {
        perror("mmap");
        return FAIL;
    }

    volatile uint32_t* bram_words =
        reinterpret_cast<volatile uint32_t*>(bram_base);

    // ------------------------------------------------------------
    // 5. REAL ADC 数据读取
    //
    // 关键点：
    //   - 1 word = 1 sample
    //   - 只用低 16 bit
    // ------------------------------------------------------------
    uint32_t num_words = size_bytes / sizeof(uint32_t);
    samples.resize(num_words);

    for (uint32_t i = 0; i < num_words; ++i) {
        samples[i] = static_cast<int16_t>(bram_words[i] & 0xFFFF);
    }

    // ------------------------------------------------------------
    // 6. 清理
    // ------------------------------------------------------------
    munmap(bram_base, size_bytes);

    std::cout << "    ✓ Read " << samples.size()
              << " REAL samples (" << size_bytes << " bytes)\n";

    return SUCCESS;
}

int RfDcApp::write_dac_bram_rftool_style(
    uint32_t tile,
    uint32_t block,
    uint32_t size_bytes,
    const std::vector<uint8_t>& data
)
{
    std::cout << "[DAC] Write BRAM (RFTOOL style) "
              << "tile=" << tile
              << " block=" << block
              << " bytes=" << size_bytes << "\n";

    if (size_bytes == 0 || data.size() < size_bytes) {
        std::cerr << "✗ Invalid DAC write size\n";
        return FAIL;
    }

    if ((size_bytes % ADC_DAC_SZ_ALIGNMENT) != 0) {
        std::cerr << "✗ size_bytes must be multiple of "
                  << ADC_DAC_SZ_ALIGNMENT << "\n";
        return FAIL;
    }

    if (size_bytes > FIFO_SIZE) {
        std::cerr << "✗ size_bytes exceeds FIFO_SIZE\n";
        return FAIL;
    }

    const auto& dac_map = rfdc_->get_dac_map();
    uint32_t idx = tile * 4 + block;
    uint32_t paddr = dac_map[idx].addr_I;

    if (paddr == 0xFFFFFFFF) {
        std::cerr << "✗ Invalid DAC BRAM address\n";
        return FAIL;
    }

    void* bram = mmap(nullptr,
                      size_bytes,
                      PROT_READ | PROT_WRITE,
                      MAP_SHARED,
                      info_.fd,
                      paddr);
    if (bram == MAP_FAILED) {
        perror("mmap DAC");
        return FAIL;
    }

    // RFTOOL behavior: raw byte copy
    std::memcpy(bram, data.data(), size_bytes);

    munmap(bram, size_bytes);

    // Enable FIFO (same as RFTOOL)
    if (change_fifo_stat(XRFDC_DAC_TILE, tile, 1) != SUCCESS) {
        std::cerr << "✗ Failed to enable DAC FIFO\n";
        return FAIL;
    }

    std::cout << "✓ DAC BRAM write done\n";
    return SUCCESS;
}

/////////////////////////////////////////////////////////

int RfDcApp::read_adc_bram_rftool_style(
    uint32_t tile,
    uint32_t block,
    uint32_t size_bytes,
    std::vector<uint8_t>& out
)
{
    std::cout << "[ADC] Read BRAM (RFTOOL style) "
              << "tile=" << tile
              << " block=" << block
              << " bytes=" << size_bytes << "\n";

    out.clear();

    if (size_bytes == 0) {
        std::cerr << "✗ Invalid ADC read size\n";
        return FAIL;
    }

    if ((size_bytes % ADC_DAC_SZ_ALIGNMENT) != 0) {
        std::cerr << "✗ size_bytes must be multiple of "
                  << ADC_DAC_SZ_ALIGNMENT << "\n";
        return FAIL;
    }

    if (size_bytes > FIFO_SIZE) {
        std::cerr << "✗ size_bytes exceeds FIFO_SIZE\n";
        return FAIL;
    }

    // Enable FIFO (same as RFTOOL)
    if (change_fifo_stat(XRFDC_ADC_TILE, tile, 1) != SUCCESS) {
        std::cerr << "✗ Failed to enable ADC FIFO\n";
        return FAIL;
    }

    const auto& adc_map = rfdc_->get_adc_map();
    uint32_t idx = tile * 4 + block;
    
    uint32_t paddr = adc_map[idx].addr_I;
    if (paddr == 0xFFFFFFFF) {
        std::cerr << "✗ Invalid ADC BRAM address\n";
        return FAIL;
    }

    void* bram = mmap(nullptr,
                      size_bytes,
                      PROT_READ | PROT_WRITE,
                      MAP_SHARED,
                      info_.fd,
                      paddr);
    if (bram == MAP_FAILED) {
        perror("mmap ADC");
        return FAIL;
    }

    out.resize(size_bytes);
    std::memcpy(out.data(), bram, size_bytes);

    munmap(bram, size_bytes);

    std::cout << "✓ ADC BRAM read done\n";
    return SUCCESS;
}
/////////////////////////////////////////////////////////


int RfDcApp::read_adc_bram_rftool_style(
    uint32_t tile,
    uint32_t block,
    uint32_t size_bytes,
    AdcSamples& out)
{
    out.I.clear();
    out.Q.clear();
    out.is_iq = false;

    if (size_bytes == 0 || (size_bytes % ADC_DAC_SZ_ALIGNMENT) != 0) {
        std::cerr << "Invalid ADC read size\n";
        return FAIL;
    }

    // Enable FIFO (same as RFTOOL)
    if (change_fifo_stat(0, tile, 1) != SUCCESS) {
        std::cerr << "Failed to enable ADC FIFO\n";
        return FAIL;
    }

    // ------------------------------------------------------------
    // 1) Query mixer settings via wrapper
    // ------------------------------------------------------------
    auto mixer = rfdc_->get_mixer_settings(
        rfdc::TileType::ADC, tile, block);

    bool is_high_speed =
        rfdc_->check_high_speed_adc(tile);

    // ------------------------------------------------------------
    // 2) RFTOOL-equivalent REAL / IQ decision
    // ------------------------------------------------------------
    bool is_real =
        (mixer.mode() == rfdc::MixerMode::R2R) ||
        (mixer.type() == rfdc::MixerType::Coarse &&
         mixer.frequency() == XRFDC_COARSE_MIX_BYPASS) ||
        (!is_high_speed);

    out.is_iq = !is_real;
    std::cout << "--------------------------------------------------\n";
    std::cout << "ADC Mixer Status (Tile " << tile
            << ", Block " << block << ")\n";

    std::cout << "  High-speed ADC : "
            << (is_high_speed ? "YES" : "NO") << "\n";

    std::cout << "  Mixer Type     : "
            << rfdc_->to_string(mixer.type()) << "\n";

    std::cout << "  Mixer Mode     : "
            << rfdc_->to_string(mixer.mode()) << "\n";

    if (mixer.type() == rfdc::MixerType::Fine) {
        std::cout << "  NCO Frequency  : "
                << mixer.frequency() << " MHz\n";
    }
    else if (mixer.type() == rfdc::MixerType::Coarse) {
        std::cout << "  Coarse Mixer   : "
                << (mixer.frequency() == XRFDC_COARSE_MIX_BYPASS
                    ? "BYPASS"
                    : "ACTIVE")
                << "\n";
    }
    else {
        std::cout << "  Mixer Path     : OFF / DISABLED\n";
    }

    std::cout << "  Data Type      : "
            << (out.is_iq ? "IQ" : "REAL") << "\n";
    std::cout << "--------------------------------------------------\n";

    // ------------------------------------------------------------
    // 3) Get BRAM addresses from wrapper map
    // ------------------------------------------------------------
    const auto& adc_map = rfdc_->get_adc_map();
    constexpr uint32_t blocks_per_tile = 4;
    uint32_t idx = tile * blocks_per_tile + block;

    uint32_t addr_I = adc_map[idx].addr_I;
    uint32_t addr_Q = adc_map[idx].addr_Q;

    if (addr_I == 0xFFFFFFFF) {
        std::cerr << "Invalid ADC BRAM address\n";
        return FAIL;
    }

    // ------------------------------------------------------------
    // 4) REAL mode read (I only)
    // ------------------------------------------------------------
    if (is_real) {
        void* base = mmap(nullptr, size_bytes,
                          PROT_READ | PROT_WRITE,
                          MAP_SHARED, info_.fd, addr_I);
        if (base == MAP_FAILED) {
            perror("mmap ADC I");
            return FAIL;
        }

        int32_t* src = static_cast<int32_t*>(base);
        size_t samples = size_bytes / sizeof(uint32_t);
        out.I.resize(samples);

        for (size_t i = 0; i < samples; ++i) {
            out.I[i] = int16_t(src[i] & 0xFFFF);
        }

        munmap(base, size_bytes);
        return SUCCESS;
    }

    // ------------------------------------------------------------
    // 5) IQ mode read (I + Q)
    // ------------------------------------------------------------
    if (addr_Q == 0xFFFFFFFF) {
        std::cerr << "IQ expected but Q BRAM missing\n";
        return FAIL;
    }

    uint32_t half_bytes = size_bytes / 2;
    size_t samples_lane = half_bytes / sizeof(int16_t);

    out.I.resize(samples_lane);
    out.Q.resize(samples_lane);

    // ---- I lane ----
    {
        void* base = mmap(nullptr, half_bytes,
                          PROT_READ | PROT_WRITE,
                          MAP_SHARED, info_.fd, addr_I);
        if (base == MAP_FAILED) {
            perror("mmap ADC I");
            return FAIL;
        }

        int32_t* src = static_cast<int32_t*>(base);
        for (size_t i = 0; i < samples_lane / 2; ++i) {
            uint32_t w = src[i];
            out.I[i * 2]     = int16_t(w & 0xFFFF);
            out.I[i * 2 + 1] = int16_t((w >> 16) & 0xFFFF);
        }
        munmap(base, half_bytes);
    }

    // ---- Q lane ----
    {
        void* base = mmap(nullptr, half_bytes,
                          PROT_READ | PROT_WRITE,
                          MAP_SHARED, info_.fd, addr_Q);
        if (base == MAP_FAILED) {
            perror("mmap ADC Q");
            return FAIL;
        }

        int32_t* src = static_cast<int32_t*>(base);
        for (size_t i = 0; i < samples_lane / 2; ++i) {
            uint32_t w = src[i];
            out.Q[i * 2]     = int16_t(w & 0xFFFF);
            out.Q[i * 2 + 1] = int16_t((w >> 16) & 0xFFFF);
        }
        munmap(base, half_bytes);
    }

    return SUCCESS;
}

void RfDcApp::init_gpio() 
{
    std::cout << "━━━ Initializing GPIO Pins ━━━\n";
    
    uint32_t max_dac = MAX_DAC_PER_TILE * MAX_DAC_TILE;
    
    // Initialize DAC user select GPIOs
    std::cout << "  Configuring DAC user select GPIOs...\n";
    for (uint32_t i = 0; i < max_dac; i++) 
    {
        auto gpio = std::make_unique<gpio::Gpio>(dac_userselect_gpio[i]);
        
        if (!gpio->enable()) 
        {
            throw std::runtime_error("Unable to enable DAC user select GPIO " + std::to_string(i));
        }
        
        if (!gpio->set_direction(gpio::Gpio::Direction::Output)) 
        {
            throw std::runtime_error("Unable to set direction for DAC user select GPIO " + std::to_string(i));
        }
        
        dac_userselect_gpios_.push_back(std::move(gpio));
    }
    std::cout << "  ✓ DAC user select GPIOs initialized (" << max_dac << " pins)\n";
    
    // Initialize DAC MTS clock enable GPIOs
    std::cout << "  Configuring DAC MTS clock GPIOs...\n";
    for (uint32_t i = 0; i < MAX_DAC_TILE; i++) 
    {
        auto gpio = std::make_unique<gpio::Gpio>(dac_mts_clk_en[i]);
        
        if (!gpio->enable()) 
        {
            throw std::runtime_error("Unable to enable DAC MTS clock GPIO " + std::to_string(i));
        }
        
        if (!gpio->set_direction(gpio::Gpio::Direction::Output)) 
        {
            throw std::runtime_error("Unable to set direction for DAC MTS clock GPIO " + std::to_string(i));
        }
        
        if (!gpio->set_value(gpio::Gpio::Value::High)) 
        {
            throw std::runtime_error("Unable to set value for DAC MTS clock GPIO " + std::to_string(i));
        }
        
        dac_mts_clk_gpios_.push_back(std::move(gpio));
    }
    std::cout << "  ✓ DAC MTS clock GPIOs initialized and enabled\n";
    
    // Initialize ADC MTS clock enable GPIOs
    std::cout << "  Configuring ADC MTS clock GPIOs...\n";
    for (uint32_t i = 0; i < MAX_ADC_TILE; i++) 
    {
        auto gpio = std::make_unique<gpio::Gpio>(adc_mts_clk_en[i]);
        
        if (!gpio->enable()) 
        {
            throw std::runtime_error("Unable to enable ADC MTS clock GPIO " + std::to_string(i));
        }
        
        if (!gpio->set_direction(gpio::Gpio::Direction::Output)) 
        {
            throw std::runtime_error("Unable to set direction for ADC MTS clock GPIO " + std::to_string(i));
        }
        
        if (!gpio->set_value(gpio::Gpio::Value::High)) 
        {
            throw std::runtime_error("Unable to set value for ADC MTS clock GPIO " + std::to_string(i));
        }
        
        adc_mts_clk_gpios_.push_back(std::move(gpio));
    }
    std::cout << "  ✓ ADC MTS clock GPIOs initialized and enabled\n";
    
    // Initialize ADC AXI switch reset GPIO
    std::cout << "  Configuring ADC AXI switch reset GPIO...\n";
    adc_axiswitch_reset_gpio_ = std::make_unique<gpio::Gpio>(adc_axiswitchrst);
    
    if (!adc_axiswitch_reset_gpio_->enable())
    {
        throw std::runtime_error("Unable to enable ADC AXI switch reset GPIO");
    }
    
    if (!adc_axiswitch_reset_gpio_->set_direction(gpio::Gpio::Direction::Output)) 
    {
        throw std::runtime_error("Unable to set direction for ADC AXI switch reset GPIO");
    }
    
    if (!adc_axiswitch_reset_gpio_->set_value(gpio::Gpio::Value::High)) 
    {
        throw std::runtime_error("Unable to set value for ADC AXI switch reset GPIO");
    }
    std::cout << "  ✓ ADC AXI switch reset GPIO initialized\n";
    std::cout << "  ✓ All GPIO pins configured successfully\n\n";
}

void RfDcApp::deinit_gpio() 
{
    std::cout << "\n━━━ Cleaning up GPIO Pins ━━━\n";
    
    // Clear all GPIO vectors - destructors will handle cleanup
    dac_userselect_gpios_.clear();
    dac_mts_clk_gpios_.clear();
    adc_mts_clk_gpios_.clear();
    adc_axiswitch_reset_gpio_.reset();
    
    std::cout << "  ✓ GPIO cleanup complete\n";
}

// ===== Clock Initialization =====

void RfDcApp::initialize_clocks() 
{
    std::cout << "━━━ Initializing RF Clocks ━━━\n";
    
    try {
        // Initialize RF Clock system
        clock_ = std::make_unique<rfdc::RFClock>(541);    // GPIO device ID
        std::cout << "  ✓ RF Clock driver initialized\n";
        std::cout << "  ✓ Driver version: " << rfdc::RFClock::get_version() << "\n";
        std::cout << "  ✓ Board type: " << 
            (rfdc::RFClock::get_board_type() == rfdc::BoardType::ZCU216 ? 
             "ZCU216" : "ZCU111") << "\n";
        
        // Configure all clocks with preset configuration
        constexpr uint32_t LMK_CONFIG = 0;
        constexpr uint32_t LMX_ADC_CONFIG = 0;
        constexpr uint32_t LMX_DAC_CONFIG = 0;
        
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
        clock_->set_all_configs(LMK_CONFIG, LMX_ADC_CONFIG, LMX_DAC_CONFIG);
        
        std::cout << "  ✓ LMK04828 configured\n";
        std::cout << "  ✓ LMX2594_1 configured\n";
        std::cout << "  ✓ LMX2594_2 configured\n";
        
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

void RfDcApp::initialize_rfdc() 
{
    std::cout << "━━━ Initializing RF Data Converter ━━━\n";
    
    // Initialize RFDC with device ID 0
    rfdc_ = std::make_unique<rfdc::RFDC>(RFDC_DEVICE_ID);
    
    std::cout << "  ✓ RFDC driver initialized\n";
    std::cout << "  ✓ Driver version: " << rfdc::RFDC::get_driver_version() << "\n";
    
    // Get IP status
    auto ip_status = rfdc_->get_ip_status();
    std::cout << format_msg("  ✓ IP State: 0x", std::hex, ip_status.state(), std::dec, "\n");
    
    // Report enabled tiles
    std::cout << "\n  Configured Tiles (Enabled in Vivado):\n";
    for (uint32_t tile = 0; tile < 4; ++tile) 
    {
        bool dac_en = ip_status.dac_tile_enabled(tile);
        bool adc_en = ip_status.adc_tile_enabled(tile);
        if (dac_en || adc_en) 
        {
            std::cout << format_msg("    Tile ", tile, ": ");
            if (dac_en) std::cout << "DAC ";
            if (adc_en) std::cout << "ADC ";
            std::cout << "\n";
        }
    }
    std::cout << "\n";
}

void RfDcApp::initialize_memory_mapping()
{
    adc_clk_wiz_addrs_ = {
        CLK_WIZ_ADC0_BASEADDR,
        CLK_WIZ_ADC1_BASEADDR,
        CLK_WIZ_ADC2_BASEADDR,
        CLK_WIZ_ADC3_BASEADDR
    };

    dac_clk_wiz_addrs_ = {
        CLK_WIZ_DAC0_BASEADDR,
        CLK_WIZ_DAC1_BASEADDR,
        CLK_WIZ_DAC2_BASEADDR,
        CLK_WIZ_DAC3_BASEADDR
    };

    // Pass to the RFDC wrapper
    rfdc_->initialize_memory_mapping(
        ADC_SINK_I_BASEADDR,
        DAC_SOURCE_I_BASEADDR,
        adc_clk_wiz_addrs_,
        dac_clk_wiz_addrs_
    );
}

void RfDcApp::initialize_mmcm_adc() {
    std::cout << "\n━━━ Initializing MMCM for ADC tiles ━━━\n";
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (rfdc_->check_tile_enabled(rfdc::TileType::ADC, tile)) {
            clock_wiz_->program_mmcm(rfdc::TileType::ADC, tile);
        }
    }
}

void RfDcApp::initialize_mmcm_dac() {
    std::cout << "\n━━━ Initializing MMCM for DAC tiles ━━━\n";
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (rfdc_->check_tile_enabled(rfdc::TileType::DAC, tile)) {
            clock_wiz_->program_mmcm(rfdc::TileType::DAC, tile);
        }
    }
}

void RfDcApp::configure_dac_tiles() 
{
    std::cout << "━━━ Configuring DAC Tiles (I/Q Mode) ━━━\n";
    
    constexpr double NCO_FREQ = 0.0;  // MHz
    
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (!rfdc_->check_tile_enabled(rfdc::TileType::DAC, tile)) {
            continue;
        }
        
        std::cout << format_msg("  Configuring DAC Tile ", tile, "...\n");
        #if 1
        constexpr double DAC_SAMPLE_RATE_MHZ = 5898.24;
        update_pll_sample_rate(
            rfdc::TileType::DAC,
            tile,
            DAC_SAMPLE_RATE_MHZ
        );
        #endif
        // Startup the tile
        rfdc_->startup(rfdc::TileType::DAC, tile);
        std::cout << "    • Tile started\n";
        
        // Wait for PLL lock
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
        
        // Configure each block in the tile (I/Q pairs)
        for (uint32_t block = 0; block < 4; block += 2) {  // Process I/Q pairs
            if (!rfdc_->check_block_enabled(rfdc::TileType::DAC, tile, block)) {
                continue;
            }
            
            std::cout << format_msg("      Block ", block, " (I) & ", block+1, " (Q):\n");
            
            // Configure I channel
            rfdc_->set_datapath_mode(
                tile,                    
                block,
                rfdc::DataPathMode::FullNyquistDucPass
            );   
            
            // Configure Q channel
            rfdc_->set_datapath_mode(
                tile,                    
                block + 1,
                rfdc::DataPathMode::FullNyquistDucPass
            );
            
        #if 1
            // Configure mixer for I channel (Real-to-Complex)
            rfdc::MixerSettings mixer_i(
                NCO_FREQ,
                0.0,
                rfdc::EventSource::Tile,
                XRFDC_COARSE_MIX_BYPASS,
                rfdc::MixerMode::C2C,  // Real-to-Complex for I/Q
                0.0,
                rfdc::MixerType::Fine
            );
            rfdc_->set_mixer_settings(rfdc::TileType::DAC, tile, block, mixer_i);
            
            // Configure mixer for Q channel (Real-to-Complex)
            rfdc::MixerSettings mixer_q(
                NCO_FREQ,
                0.0,
                rfdc::EventSource::Tile,
                XRFDC_COARSE_MIX_BYPASS,
                rfdc::MixerMode::C2C,  // Real-to-Complex for I/Q
                0.0,
                rfdc::MixerType::Fine
            );
            rfdc_->set_mixer_settings(rfdc::TileType::DAC, tile, block + 1, mixer_q);
            
            std::cout << format_msg("        - Mixer (I/Q): ", NCO_FREQ, " MHz, R2C mode\n");
            
            // Set Nyquist zone for both I and Q
            rfdc_->set_nyquist_zone(rfdc::TileType::DAC, tile, block, 
                                   rfdc::NyquistZone::Zone1);
            rfdc_->set_nyquist_zone(rfdc::TileType::DAC, tile, block + 1, 
                                   rfdc::NyquistZone::Zone1);
            
            // Configure interpolation for both channels
            rfdc_->set_interpolation_factor(tile, block, 2);
            rfdc_->set_interpolation_factor(tile, block + 1, 2);
            std::cout << "        - Interpolation: 2x (I/Q)\n";
           
            // Disable inverse sinc filter for both channels
            rfdc_->set_inverse_sinc_filter(tile, block, 0);
            rfdc_->set_inverse_sinc_filter(tile, block + 1, 0);
            std::cout << "        - Inverse Sinc: Disabled (I/Q)\n";
            
            // Reset NCO phase for both channels
            rfdc_->reset_nco_phase(rfdc::TileType::DAC, tile, block);
            rfdc_->reset_nco_phase(rfdc::TileType::DAC, tile, block + 1);
            std::cout << "        - NCO Phase: reset (I/Q)\n";
            
            // Update event for both channels
            rfdc_->update_event(rfdc::TileType::DAC, tile, block, 
                              XRFDC_EVENT_MIXER);
            rfdc_->update_event(rfdc::TileType::DAC, tile, block + 1, 
                              XRFDC_EVENT_MIXER);
            #endif
        }
        
        // **CRITICAL: Reprogram MMCM after configuration changes**
        std::cout << "    • Reprogramming MMCM...\n";
        if (!clock_wiz_->program_mmcm(rfdc::TileType::DAC, tile)) {
            std::cerr << "    ✗ MMCM programming failed\n";
        }
        
        // Enable FIFO
        rfdc_->setup_fifo(rfdc::TileType::DAC, tile, true);
        std::cout << "    • FIFO enabled\n";
        
        std::cout << format_msg("  ✓ DAC Tile ", tile, " configured (I/Q)\n\n");
    }
}


void RfDcApp::configure_adc_tiles() {
    std::cout << "━━━ Configuring ADC Tiles (I/Q Mode - RF Eval Tool Style) ━━━\n";
    
    constexpr double NCO_FREQ = 0.0;  // MHz
    
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (!rfdc_->check_tile_enabled(rfdc::TileType::ADC, tile)) {
            continue;
        }
        
        std::cout << format_msg("  Configuring ADC Tile ", tile, "...\n");
        
        // Startup the tile
        rfdc_->startup(rfdc::TileType::ADC, tile);
        std::cout << "    • Tile started\n";
        
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
        
        // Configure each block individually (RF Eval Tool style)
        // Each block is "Real" but outputs I/Q via Fine mixer
        for (uint32_t block = 0; block < 4; ++block) {
            if (!rfdc_->check_block_enabled(rfdc::TileType::ADC, tile, block)) {
                continue;
            }
            
            std::cout << format_msg("      Block ", block, ":\n");
            
            // Configure decimation
            rfdc_->set_decimation_factor(tile, block, 1);  // RF Eval shows 1x
            std::cout << "        - Decimation: 1x\n";      
            
            // CRITICAL: Use Fine mixer with R2C (Real to I/Q) mode
            // This matches RF Eval Tool's "Real to I/Q" setting
            rfdc::MixerSettings mixer(
                NCO_FREQ,                  // 0.0 MHz frequency
                0.0,                       // Phase
                rfdc::EventSource::Tile,
                XRFDC_COARSE_MIX_BYPASS,   // Bypass coarse mixer
                rfdc::MixerMode::R2C,      // Real-to-Complex (Real to I/Q)
                0,
                rfdc::MixerType::Fine      // Fine mixer (as shown in GUI)
            );
            rfdc_->set_mixer_settings(rfdc::TileType::ADC, tile, block, mixer);
            
            std::cout << format_msg("        - Mixer: Fine, Real to I/Q, ", NCO_FREQ, " MHz\n");
            
            // Set Nyquist zone
            rfdc_->set_nyquist_zone(rfdc::TileType::ADC, tile, block,
                                   rfdc::NyquistZone::Zone1);
            
            // Configure QMC (disabled in RF Eval Tool)
            rfdc::QMCSettings qmc(
                false, false, 0.0, 0.0, 0,
                rfdc::EventSource::Tile
            );
            
            rfdc_->set_qmc_settings(rfdc::TileType::ADC, tile, block, qmc);
            std::cout << "        - QMC: Disabled\n";
            
            // Set calibration mode (AutoCal in RF Eval Tool)
            rfdc_->set_calibration_mode(tile, block, rfdc::CalibrationMode::Mode1);
            std::cout << "        - Calibration: Mode 1 (AutoCal)\n";
            
            // Update event
            rfdc_->update_event(rfdc::TileType::ADC, tile, block,
                              XRFDC_EVENT_MIXER);
        }
        
        // **CRITICAL: Reprogram MMCM after configuration changes**
        std::cout << "    • Reprogramming MMCM...\n";
        if (!clock_wiz_->program_mmcm(rfdc::TileType::ADC, tile)) {
            std::cerr << "    ✗ MMCM programming failed\n";
        }
        
        // Enable FIFO
        rfdc_->setup_fifo(rfdc::TileType::ADC, tile, true);
        std::cout << "    • FIFO enabled\n";
        
        std::cout << format_msg("  ✓ ADC Tile ", tile, " configured (Real to I/Q mode)\n\n");
    }
}

#if 0 //this is real mode 
void RfDcApp::configure_dac_tiles() 
{
    std::cout << "━━━ Configuring DAC Tiles ━━━\n";
    
    constexpr double NCO_FREQ = 0.0;  // MHz
    
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (!rfdc_->check_tile_enabled(rfdc::TileType::DAC, tile)) {
            continue;
        }
        
        std::cout << format_msg("  Configuring DAC Tile ", tile, "...\n");
        #if 1
        constexpr double DAC_SAMPLE_RATE_MHZ = 5898.24;
        update_pll_sample_rate(
            rfdc::TileType::DAC,
            tile,
            DAC_SAMPLE_RATE_MHZ
        );
        #endif
        // Startup the tile
        rfdc_->startup(rfdc::TileType::DAC, tile);
        std::cout << "    • Tile started\n";
        
        // Wait for PLL lock
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
            rfdc_->set_datapath_mode(
                tile,                    
                block,
                rfdc::DataPathMode::FullNyquistDucPass
            );   
        #if 1
            // Configure mixer
            rfdc::MixerSettings mixer(
                NCO_FREQ,
                0.0,
                rfdc::EventSource::Tile,
                XRFDC_COARSE_MIX_BYPASS,
                rfdc::MixerMode::R2R,
                0.0,
                rfdc::MixerType::Coarse
            );
            rfdc_->set_mixer_settings(rfdc::TileType::DAC, tile, block, mixer);
            std::cout << format_msg("        - Mixer: ", NCO_FREQ, " MHz\n");
            
            // Set Nyquist zone
            rfdc_->set_nyquist_zone(rfdc::TileType::DAC, tile, block, 
                                   rfdc::NyquistZone::Zone1);
            // Configure interpolation
            rfdc_->set_interpolation_factor(tile, block, 2);
            std::cout << "        - Interpolation: 2x\n";
           
            // Disable inverse sinc filter
            rfdc_->set_inverse_sinc_filter(tile, block, 0);
            std::cout << "        - Inverse Sinc: Disabled\n";
            
            rfdc_->reset_nco_phase(rfdc::TileType::DAC, tile, block);
            std::cout << "        - NCO Phase: reset\n";
            // Update event
            rfdc_->update_event(rfdc::TileType::DAC, tile, block, 
                              XRFDC_EVENT_MIXER);
            #endif
        }
        
        // **CRITICAL: Reprogram MMCM after configuration changes**
        std::cout << "    • Reprogramming MMCM...\n";
        if (!clock_wiz_->program_mmcm(rfdc::TileType::DAC, tile)) {
            std::cerr << "    ✗ MMCM programming failed\n";
        }
        
        // Enable FIFO
        rfdc_->setup_fifo(rfdc::TileType::DAC, tile, true);
        std::cout << "    • FIFO enabled\n";
        
        std::cout << format_msg("  ✓ DAC Tile ", tile, " configured\n\n");
    }
}

void RfDcApp::configure_adc_tiles() {
    std::cout << "━━━ Configuring ADC Tiles ━━━\n";
    
    constexpr double NCO_FREQ = 0;  // MHz
    
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (!rfdc_->check_tile_enabled(rfdc::TileType::ADC, tile)) {
            continue;
        }
        
        std::cout << format_msg("  Configuring ADC Tile ", tile, "...\n");
        
        // Startup the tile
        rfdc_->startup(rfdc::TileType::ADC, tile);
        std::cout << "    • Tile started\n";
        
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
                  // Configure decimation
            rfdc_->set_decimation_factor(tile, block, 2);
            std::cout << "        - Decimation: 2x\n";      
            // Configure mixer
            rfdc::MixerSettings mixer(
                0.0,
                0.0,
                rfdc::EventSource::Tile,
                XRFDC_COARSE_MIX_BYPASS,
                rfdc::MixerMode::R2R,
                0,
                rfdc::MixerType::Coarse
            );
            rfdc_->set_mixer_settings(rfdc::TileType::ADC, tile, block, mixer);
            std::cout << format_msg("        - Mixer: ", NCO_FREQ, " MHz\n");
            
            // Set Nyquist zone
            rfdc_->set_nyquist_zone(rfdc::TileType::ADC, tile, block,
                                   rfdc::NyquistZone::Zone1);
            

            
            // Configure QMC
            rfdc::QMCSettings qmc(
                false, false, 0.0, 0.0, 0,
                rfdc::EventSource::Tile
            );

            //rfdc::QMCSettings qmc(
            //    true,   // EnableGain
            //    false,  // EnablePhase
            //    4.0,    // GainCorrectionFactor (start here)
            //    0.0,    // PhaseCorrectionFactor
            //    0,      // OffsetCorrectionFactor
            //    rfdc::EventSource::Tile
            //);    

            rfdc_->set_qmc_settings(rfdc::TileType::ADC, tile, block, qmc);
            std::cout << "        - QMC: Disable\n";
            
            // Set calibration mode
            rfdc_->set_calibration_mode(tile, block, rfdc::CalibrationMode::Mode1);
            std::cout << "        - Calibration: Mode 1\n";
            
            // Update event
            rfdc_->update_event(rfdc::TileType::ADC, tile, block,
                              XRFDC_EVENT_MIXER);
        }
        
        // **CRITICAL: Reprogram MMCM after configuration changes**
        std::cout << "    • Reprogramming MMCM...\n";
        if (!clock_wiz_->program_mmcm(rfdc::TileType::ADC, tile)) {
            std::cerr << "    ✗ MMCM programming failed\n";
        }
        
        // Enable FIFO
        rfdc_->setup_fifo(rfdc::TileType::ADC, tile, true);
        std::cout << "    • FIFO enabled\n";
        
        std::cout << format_msg("  ✓ ADC Tile ", tile, " configured\n\n");
    }
}

#endif
void RfDcApp::verify_configuration() 
{
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
        std::cout << std::fixed << std::setprecision(4)
                 << "    Ref Clock: " << pll.ref_clk_freq() << " MHz\n"
                 << "    Sample Rate: " << pll.sample_rate() << " MSPS\n";
        
        // Check FIFO status
        bool fifo_enabled = rfdc_->get_fifo_status(rfdc::TileType::DAC, tile);
        std::cout << format_msg("    FIFO: ", 
                               (fifo_enabled ? "Enabled" : "Disabled"), "\n");
    }
    
    std::cout << "\n";
    
    // Verify ADC tiles
    for (uint32_t tile = 0; tile < 4; ++tile)
    {
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
        std::cout << std::fixed << std::setprecision(4)
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
            std::cout
                << "DDC: "
                << (block_status.digital_datapath_enabled() ? "ENABLED" : "DISABLED")
                << " | Clocks: "
                << (block_status.digital_path_clocks_enabled() ? "ON" : "OFF")
                << std::endl;
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
            std::cout
                << "DDC: "
                << (block_status.digital_datapath_enabled() ? "ENABLED" : "DISABLED")
                << " | Clocks: "
                << (block_status.digital_path_clocks_enabled() ? "ON" : "OFF")
                << std::endl;
            std::cout << "\n";
        }
    }
    
    std::cout << "\n";
}

// ===== Data Transfer Wrapper Functions =====

void RfDcApp::write_dac_samples(uint32_t tile, uint32_t block,
                                const std::vector<int16_t>& samples)
{
    #if 0
    uint32_t size_bytes = samples.size() * sizeof(int16_t);
    int ret = write_data_to_memory_bram(block, tile, size_bytes, samples);
    if (ret != SUCCESS) {
        throw std::runtime_error("Failed to write DAC samples");
    }
    #endif

    const uint32_t size_bytes =
        samples.size() * sizeof(int16_t);

    std::vector<uint8_t> raw(size_bytes);
    std::memcpy(raw.data(), samples.data(), size_bytes);

    int ret = write_dac_bram_rftool_style(
        tile,
        block,
        size_bytes,
        raw
    );

    if (ret != SUCCESS) {
        throw std::runtime_error("Failed to write DAC samples");
    }
}


std::vector<int16_t> RfDcApp::read_adc_samples_pure_real(uint32_t tile, uint32_t block,
                                               size_t num_samples)
{
    std::vector<int16_t> samples;
    uint32_t size_bytes = num_samples * sizeof(int16_t);
    
    int ret = read_data_from_memory_bram(block, tile, size_bytes, samples);
    if (ret != SUCCESS) {
        throw std::runtime_error("Failed to read ADC samples");
    }
    
    return samples;
}


#if 0
RfDcApp::AdcSamples RfDcApp::read_adc_samples_i_q(
    uint32_t tile,
    uint32_t block,
    size_t num_samples
)
{
#if 0
    AdcSamples captured;

    const uint32_t size_bytes =
        num_samples * sizeof(int16_t);

    int ret = read_adc_bram_rftool_style(
        tile,
        block,
        size_bytes,
        captured
    );

    if (ret != SUCCESS) {
        throw std::runtime_error("Failed to read ADC samples");
    }

    captured.is_iq = !captured.Q.empty();

    return captured;
#endif
    AdcSamples captured;
    captured.I.clear();
    captured.Q.clear();
    captured.is_iq = false;

    // ------------------------------------------------------------
    // RFDC BRAM stores 2× int16 per 32-bit word
    // We must read WHOLE WORDS
    // ------------------------------------------------------------
    const size_t samples_per_word = 2;
    const size_t words =
        (num_samples + samples_per_word - 1) / samples_per_word;

    const uint32_t size_bytes =
        words * sizeof(uint32_t);   // <-- CRITICAL FIX

    // ------------------------------------------------------------
    // Raw BRAM read (RFTOOL style)
    // ------------------------------------------------------------
    std::vector<uint8_t> raw;
    int ret = read_adc_bram_rftool_style(
        tile,
        block,
        size_bytes,
        raw
    );

    if (ret != SUCCESS) {
        throw std::runtime_error("Failed to read ADC BRAM");
    }

    // ------------------------------------------------------------
    // Decode AFTER read (little-endian int16 stream)
    // ------------------------------------------------------------
    const int16_t* s = reinterpret_cast<const int16_t*>(raw.data());
    const size_t total_samples = raw.size() / sizeof(int16_t);

    captured.I.reserve(num_samples);

    for (size_t i = 0; i < total_samples && captured.I.size() < num_samples; ++i) {
        captured.I.push_back(s[i]);
    }

    captured.is_iq = false;   // raw REAL stream

    return captured;
}
// ===== Test Functions =====
#endif

void RfDcApp::run_loopback_test() 
{
    std::cout << "━━━ Running Loopback Test ━━━\n";
    std::cout << "  Testing: DAC Tile 0 Block 0 → ADC Tile 0 Block 0\n";
    std::cout << "  (Loopback cable connected)\n\n";
    
    try {
        const uint32_t tile = 0;
        const uint32_t block = 0;
        const uint32_t channel_mask = 0x0001;
        const size_t num_samples = 16384;
        const double test_frequency = 50e6;  // 100 MHz RF output desired
        
        // Get DAC configuration
        auto dac_pll = rfdc_->get_pll_config(rfdc::TileType::DAC, tile);
        double dac_pll_rate_hz = dac_pll.sample_rate() * 1e9;
        uint32_t dac_interpolation = rfdc_->get_interpolation_factor(tile, block);
        
        // Get ADC configuration
        auto adc_pll = rfdc_->get_pll_config(rfdc::TileType::ADC, tile);
        double adc_pll_rate_hz = adc_pll.sample_rate() * 1e9;
        uint32_t adc_decimation = rfdc_->get_decimation_factor(tile, block);
        
        // Get IP type for Gen3 checks
        uint32_t ip_type = rfdc_->get_ip_type();
        bool is_gen3_plus = (ip_type >= XRFDC_GEN3);
        
        std::cout << "Configuration:\n";
        std::cout << "  IP Type: " << (is_gen3_plus ? "Gen3+" : "Pre-Gen3") << " (value=" << ip_type << ")\n";
        std::cout << "  DAC PLL Rate: " << dac_pll.sample_rate() << " GSPS\n";
        std::cout << "  DAC Interpolation: " << dac_interpolation << "x\n";
        std::cout << "  ADC PLL Rate: " << adc_pll.sample_rate() << " GSPS\n";
        std::cout << "  ADC Decimation: " << adc_decimation << "x\n";
        
        // ===== DEBUG: Data Path Configuration =====
        std::cout << "\n━━━ DEBUG: Data Path Configuration ━━━\n";
        
        // DAC data path mode
        uint32_t dac_data_path_mode = 1;  // Default
        if (is_gen3_plus) {
            try {
                dac_data_path_mode = rfdc_->get_data_path_mode(tile, block);
                std::cout << "  1. DAC Data Path Mode: " << dac_data_path_mode;
                switch(dac_data_path_mode) {
                    case 1: std::cout << " (XRFDC_DAC_MODE_7G_NQ1 - 1st Nyquist, 7 GSPS)\n"; break;
                    case 2: std::cout << " (XRFDC_DAC_MODE_7G_NQ2 - 2nd Nyquist, 7 GSPS)\n"; break;
                    case 3: std::cout << " (XRFDC_DAC_MODE_10G_IMR - 1st Nyquist, 10 GSPS IMR)\n"; break;
                    case 4: std::cout << " (XRFDC_DAC_MODE_10G_BYPASS - Full BW, Bypass)\n"; break;
                    default: std::cout << " (Unknown)\n"; break;
                }
            } catch (const std::exception& e) {
                std::cout << "  1. DAC Data Path Mode: Not available (" << e.what() << ")\n";
            }
        }

        if (is_gen3_plus) {
            try {
                uint32_t dac_imr_mode = rfdc_->get_imr_pass_mode(tile, block);
                std::cout << "  2. DAC IMR Pass Mode: " << dac_imr_mode;
                if (dac_imr_mode == 0) {
                    std::cout << " (LOWPASS - passes 1st Nyquist)\n";
                } else if (dac_imr_mode == 1) {
                    std::cout << " (HIGHPASS - passes 2nd Nyquist)\n";
                } else {
                    std::cout << " (Unknown)\n";
                }
            } catch (const std::exception& e) {
                std::cout << "  2. DAC IMR Pass Mode: Not available (" << e.what() << ")\n";
            }
        }

        // Get fabric interface widths
        uint32_t dac_wr_width = 0, adc_rd_width = 0;
        rfdc_->get_fab_wr_vld_words(rfdc::TileType::DAC, tile, block, dac_wr_width);
        rfdc_->get_fab_rd_vld_words(rfdc::TileType::ADC, tile, block, adc_rd_width);
        std::cout << "  DAC Fabric Write Width: " << dac_wr_width << " words\n";
        std::cout << "  ADC Fabric Read Width: " << adc_rd_width << " words\n";
        
        // Get mixer settings
        auto dac_mixer = rfdc_->get_mixer_settings(rfdc::TileType::DAC, tile, block);
        auto adc_mixer = rfdc_->get_mixer_settings(rfdc::TileType::ADC, tile, block);
        std::cout << "  DAC Mixer Mode: " << static_cast<int>(dac_mixer.mode()) 
                  << " (1=C2C, 1=C2R, 3=R2C, 4=R2R)\n";
        std::cout << "  ADC Mixer Mode: " << static_cast<int>(adc_mixer.mode())
                  << " (1=C2C, 2=C2R, 3=R2C, 4=R2R)\n";
        
        // Calculate actual decimation factor including data path mode
        uint32_t adc_total_decimation = adc_decimation;
        // ===== DAC PLAYBACK SEQUENCE =====
        std::cout << "━━━ DAC Tile 0 Block 0 Setup ━━━\n";
        
        if (!rfdc_->get_pll_lock_status(rfdc::TileType::DAC, tile)) {
            throw std::runtime_error("DAC Tile 0 PLL not locked!");
        }
        std::cout << "  ✓ DAC PLL locked\n";
        
        std::cout << "\n  Step 1: Reset DAC memory\n";
        local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, 0x0000);
        
        std::cout << "  Step 2: Configure sample count\n";
        set_local_mem_sample(rfdc::TileType::DAC, tile, block, num_samples);
        
        std::cout << "  Step 3: Generate sine wave\n";
        bool imr_lowpass_enabled  = false;
        uint32_t dac_datapath = rfdc_->get_data_path_mode(tile, block);

        if (dac_datapath == static_cast<uint32_t>(rfdc::DataPathMode::IMRLowPass))
        {
            imr_lowpass_enabled = true;
        }
        auto samples = generate_sine_wave(test_frequency, dac_pll_rate_hz, 
                                         dac_interpolation, num_samples, 30000, -50.0,imr_lowpass_enabled);
        //unsigned int dac_datapath = rfdc_->get_data_path_mode(tile, block);
        //auto samples = generate_sine_wave_rf(test_frequency, dac_pll_rate_hz, 
        //                                 dac_interpolation, num_samples, 30000, -50.0,static_cast<rfdc::DataPathMode>(dac_datapath));
        std::cout << "  Step 4: Write samples to DAC BRAM\n";
        write_dac_samples(tile, block, samples);
        
        
        std::cout << "  DAC Datapath mode: " << dac_datapath
                  << " (1=First Nyquist zone FS 7 GSPS, 2=Second Nyquist zone FS 7 GSPS, 3=First Nyquist zone FS 10 GSPS IMR, 4=Full Bandwidth, Bypass datapath)\n";
        // Create DAC metadata
        std::stringstream dac_meta;
        dac_meta << "# RFDC DAC Configuration\n";
        dac_meta << "# type: DAC\n";
        dac_meta << "# tile: " << tile << "\n";
        dac_meta << "# block: " << block << "\n";
        dac_meta << "# datapath_mode: " << dac_datapath << "\n";
        dac_meta << "# pll_rate_mhz: " << (dac_pll_rate_hz / 1e6) << "\n";
        dac_meta << "# interpolation: " << dac_interpolation << "\n";
        dac_meta << "# signal_frequency_mhz: " << test_frequency << "\n";  // Baseband in BRAM
        dac_meta << "# rf_output_frequency_mhz: " << (test_frequency / 1e6) << "\n";  // RF output
        dac_meta << "# num_samples: " << num_samples << "\n";
        dac_meta << "# amplitude: 30000\n";
        
        save_samples_to_csv(samples, dac_pll_rate_hz, "dac_t0_b0_100MHz.csv", dac_meta.str());
        std::cout << "    ✓ Saved: dac_t0_b0_100MHz.csv (with metadata)\n";
        
        std::cout << "  Step 5: **TRIGGER DAC playback**\n";
        local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, channel_mask);
        
        std::cout << "    Waiting 200ms for DAC to stabilize...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "  ✓ DAC is now outputting RF signal\n\n";
        
        // ===== ADC CAPTURE SEQUENCE =====
        std::cout << "━━━ ADC Tile 0 Block 0 Capture ━━━\n";
        
        if (!rfdc_->get_pll_lock_status(rfdc::TileType::ADC, tile)) {
            throw std::runtime_error("ADC Tile 0 PLL not locked!");
        }
        std::cout << "  ✓ ADC PLL locked\n";
        
        std::cout << "\n  Step 1: Configure ADC capture size\n";
        set_local_mem_sample(rfdc::TileType::ADC, tile, block, num_samples);
        
        std::cout << "  Step 2: **TRIGGER ADC capture**\n";
        local_mem_trigger(rfdc::TileType::ADC, tile, num_samples, channel_mask);
        
        std::cout << "    Waiting 200ms for capture to complete...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "  Step 3: Reading captured data from ADC BRAM\n";
        auto captured = read_adc_samples_i_q(tile, block, num_samples);
        
        // Create ADC metadata with TOTAL decimation
        std::stringstream adc_meta;
        adc_meta << "# RFDC ADC Configuration\n";
        adc_meta << "# type: ADC\n";
        adc_meta << "# tile: " << tile << "\n";
        adc_meta << "# block: " << block << "\n";
        adc_meta << "# pll_rate_mhz: " << (adc_pll_rate_hz / 1e6) << "\n";
        adc_meta << "# decimation: " << adc_total_decimation << "\n";  // Use TOTAL decimation!
        adc_meta << "# decimation_factor: " << adc_decimation << "\n";  // Original factor
        adc_meta << "# rf_input_frequency_mhz: " << (test_frequency / 1e6) << "\n";  // RF input
        adc_meta << "# signal_frequency_mhz: " << test_frequency << "\n";  // Baseband in BRAM
        adc_meta << "# num_samples: " << num_samples << "\n";
        
        save_samples_to_csv(captured.I, adc_pll_rate_hz, "adc_t0_b0_capture.csv", adc_meta.str());
        std::cout << "    ✓ Saved: adc_t0_b0_capture.csv (with metadata)\n";
        
        // ===== ANALYSIS =====
        std::cout << "\n━━━ Analysis ━━━\n";
        
        std::cout << "  First 32 captured samples:\n    ";
        for (size_t i = 0; i < std::min(size_t(32), captured.size()); ++i) {
            std::cout << captured[i] << " ";
            if ((i + 1) % 16 == 0 && i < 31) std::cout << "\n    ";
        }
        std::cout << "\n\n";
        
        auto max_it = std::max_element(captured.begin(), captured.end());
        auto min_it = std::min_element(captured.begin(), captured.end());
        
        int16_t max_val = (max_it != captured.end()) ? *max_it : 0;
        int16_t min_val = (min_it != captured.end()) ? *min_it : 0;
        int16_t peak_to_peak = max_val - min_val;
        
        double sum_squares = 0.0;
        for (const auto& sample : captured) {
            sum_squares += sample * sample;
        }
        double rms = std::sqrt(sum_squares / captured.size());
        
        std::cout << "  Signal Statistics:\n";
        std::cout << "    Max value:      " << max_val << " counts\n";
        std::cout << "    Min value:      " << min_val << " counts\n";
        std::cout << "    Peak-to-Peak:   " << peak_to_peak << " counts\n";
        std::cout << "    RMS:            " << static_cast<int>(rms) << " counts\n";
        
        std::cout << "\n━━━ Result ━━━\n";
        if (peak_to_peak > 1000) {
            std::cout << "  ✓✓✓ LOOPBACK SUCCESS! ✓✓✓\n";
            std::cout << "  Strong signal detected!\n";
            std::cout << "  Expected: ~60000 counts P-P for 30000 amplitude sine wave\n";
            std::cout << "  Measured: " << peak_to_peak << " counts P-P\n";
            std::cout << "\n  System is working correctly!\n";
            std::cout << "\n  To analyze results (metadata auto-detected):\n";
            std::cout << "    python3 plot_loopback.py dac_t0_b0_100MHz.csv adc_t0_b0_capture.csv\n";
        } else {
            std::cout << "  ✗ NO/WEAK SIGNAL DETECTED\n";
            std::cout << "  Measured: " << peak_to_peak << " counts P-P\n";
        }
        
    } catch (const std::exception& e) {
        std::cerr << "\n✗ Error: " << e.what() << "\n";
    }
}

void RfDcApp::save_samples_to_csv(const std::vector<int16_t>& samples,
                                  double sample_rate_hz,
                                  const std::string& filename,
                                  const std::string& metadata)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + filename);
    }
    
    // Write metadata as comments if provided
    if (!metadata.empty()) {
        file << metadata;
    }
    
    // Write CSV header
    file << "sample_index,time_ns,value\n";
    
    // Calculate time step
    double time_step_ns = (1.0 / sample_rate_hz) * 1e9;
    
    // Write samples
    for (size_t i = 0; i < samples.size(); ++i) {
        double time_ns = i * time_step_ns;
        file << i << "," << time_ns << "," << samples[i] << "\n";
    }
    
    file.close();
}

void RfDcApp::save_samples_to_csv(
    const std::vector<int16_t>& I,
    const std::vector<int16_t>& Q,
    double fabric_sample_rate_hz,
    const std::string& filename,
    const std::string& metadata
)
{
    if (I.size() != Q.size()) {
        throw std::runtime_error("I/Q size mismatch");
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    if (!metadata.empty()) {
        file << metadata;
    }

    file << "sample_index,time_ns,I,Q\n";

    const double time_step_ns = 1e9 / fabric_sample_rate_hz;

    for (size_t i = 0; i < I.size(); ++i) {
        file << i << ","
             << (i * time_step_ns) << ","
             << I[i] << ","
             << Q[i] << "\n";
    }
}
// ===== Waveform Generation Functions =====

std::vector<int16_t> RfDcApp::generate_sine_wave(
    double frequency_hz,          // Digital frequency (CF in RF Eval Tool)
    double dac_pll_rate_hz,        // PLL rate
    uint32_t dac_interpolation,
    size_t num_samples,
    int16_t amplitude,
    double noise_dbfs,
    bool imr_lowpass_enabled
)
{
    std::vector<int16_t> samples(num_samples);
    const uint32_t eff_interp =
        dac_interpolation * (imr_lowpass_enabled ? 2 : 1);
    // RF Eval Tool behavior:
    // Samples are generated at the FABRIC rate
    const double sample_rate_hz = dac_pll_rate_hz / eff_interp;

    const double two_pi = 2.0 * M_PI;
    const double phase_increment = two_pi * frequency_hz / sample_rate_hz;

    // Optional noise
    const double noise_rms = amplitude * std::pow(10.0, noise_dbfs / 20.0);
    std::default_random_engine rng;
    std::normal_distribution<double> gaussian(0.0, 1.0);

    for (size_t i = 0; i < num_samples; ++i) {
        const double phase = phase_increment * static_cast<double>(i);
        const double sine  = amplitude * std::sin(phase);
        const double noise = (noise_dbfs < 0.0) ? noise_rms * gaussian(rng) : 0.0;

        double sample = sine + noise;

        // Clamp (C++14)
        if (sample > 32767.0)
            sample = 32767.0;
        else if (sample < -32768.0)
            sample = -32768.0;

        samples[i] = static_cast<int16_t>(std::lrint(sample));
    }

    std::cout << "  ✓ Generated " << num_samples << " DAC samples\n";
    std::cout << "      Desired RF Freq:        " << frequency_hz / 1e6 << " MHz\n";
    std::cout << "      DAC PLL Rate:           " << dac_pll_rate_hz / 1e9 << " GSPS\n";
    std::cout << "      Fabric Rate:            " << sample_rate_hz / 1e6 << " MSPS\n";
    std::cout << "      Interp (User):          " << dac_interpolation << "x\n";
    std::cout << "      Interp (Effective):     " << eff_interp << "x\n";
    std::cout << "      IMR Low-Pass:           "
              << (imr_lowpass_enabled ? "ON" : "OFF") << "\n";

    return samples;
}

std::vector<int16_t> RfDcApp::generate_sine_wave_rf(
    double frequency_mhz,      // RF-equivalent frequency (what user expects)
    double dac_pll_rate_hz,
    uint32_t dac_interpolation,
    size_t num_samples,
    int16_t amplitude,
    double noise_dbfs,
    rfdc::DataPathMode datapath
)
{
    double rf_freq_mhz = frequency_mhz;
    std::vector<int16_t> samples(num_samples);
    // RF Eval Tool behavior:
    // Samples are generated at the FABRIC rate
    const double sample_rate_hz = dac_pll_rate_hz / dac_interpolation;

    const double two_pi = 2.0 * M_PI;
    const double phase_increment = two_pi * rf_freq_mhz / sample_rate_hz;

    // Optional noise
    const double noise_rms = amplitude * std::pow(10.0, noise_dbfs / 20.0);
    std::default_random_engine rng;
    std::normal_distribution<double> gaussian(0.0, 1.0);

    for (size_t i = 0; i < num_samples; ++i) {
        const double phase = phase_increment * static_cast<double>(i);
        const double sine  = amplitude * std::sin(phase);
        const double noise = (noise_dbfs < 0.0) ? noise_rms * gaussian(rng) : 0.0;

        double sample = sine + noise;

        // Clamp (C++14)
        if (sample > 32767.0)
            sample = 32767.0;
        else if (sample < -32768.0)
            sample = -32768.0;

        samples[i] = static_cast<int16_t>(std::lrint(sample));
    }

    std::cout << "  ✓ Generated " << num_samples << " DAC samples\n";
    std::cout << "      Digital Frequency (CF-RF): " << rf_freq_mhz / 1e6 << " MHz\n";
    std::cout << "      DAC PLL Rate:           " << dac_pll_rate_hz / 1e9 << " GSPS\n";
    std::cout << "      Fabric Rate:            " << sample_rate_hz / 1e6 << " MSPS\n";
    std::cout << "      Interpolation:          " << dac_interpolation << "x\n";
    std::cout << "      Amplitude:              " << amplitude << " LSB\n";
    std::cout << "      Noise:                  " << noise_dbfs << " dBFS\n";

    return samples;
}


std::vector<int16_t> RfDcApp::generate_dc_offset(int16_t value, size_t num_samples)
{
    std::vector<int16_t> samples(num_samples, value);
    
    std::cout << "  ✓ Generated " << num_samples 
              << " sample DC offset: " << value << "\n";
    
    return samples;
}

// Simplify set_local_mem_sample to use LocalMem class
void RfDcApp::set_local_mem_sample(rfdc::TileType type, uint32_t tile_id,
                                   uint32_t block_id, uint32_t num_samples)
{
    void* mem_base_addr = (type == rfdc::TileType::DAC) ? 
                          rfdc_->get_dac_vaddr() : 
                          rfdc_->get_adc_vaddr();
    
    const void* channel_map = (type == rfdc::TileType::DAC) ?
                              static_cast<const void*>(rfdc_->get_dac_map().data()) :
                              static_cast<const void*>(rfdc_->get_adc_map().data());
    
    local_mem_->set_sample_count(type, tile_id, block_id, num_samples, 
                                 mem_base_addr, channel_map);
}

// Simplify local_mem_trigger to use LocalMem class
void RfDcApp::local_mem_trigger(rfdc::TileType type, uint32_t tile_id,
                               uint32_t num_samples, uint32_t channel_mask)
{
    if (channel_mask == 0) {
        std::cout << "  • Reset " << (type == rfdc::TileType::DAC ? "DAC" : "ADC")
                  << " Tile " << tile_id << "\n";
        
        // Reset via LocalMem
        void* mem_base_addr = (type == rfdc::TileType::DAC) ? 
                              rfdc_->get_dac_vaddr() : 
                              rfdc_->get_adc_vaddr();
        
        const void* channel_map = (type == rfdc::TileType::DAC) ?
                                  static_cast<const void*>(rfdc_->get_dac_map().data()) :
                                  static_cast<const void*>(rfdc_->get_adc_map().data());
        
        local_mem_->trigger(type, mem_base_addr, 0, channel_map);
        return;
    }
    
    std::cout << "  • **TRIGGER** " << (type == rfdc::TileType::DAC ? "DAC" : "ADC")
              << " Tile " << tile_id << " mask=0x" << std::hex << channel_mask << std::dec
              << " samples=" << num_samples << "\n";
    
    // Use LocalMem hardware trigger
    void* mem_base_addr = (type == rfdc::TileType::DAC) ? 
                          rfdc_->get_dac_vaddr() : 
                          rfdc_->get_adc_vaddr();
    
    const void* channel_map = (type == rfdc::TileType::DAC) ?
                              static_cast<const void*>(rfdc_->get_dac_map().data()) :
                              static_cast<const void*>(rfdc_->get_adc_map().data());
    
    local_mem_->trigger(type, mem_base_addr, channel_mask, channel_map);
    
    // Also trigger via UIO for redundancy (both methods used in RFTool)
    for (uint32_t block = 0; block < 4; ++block) {
        if (!(channel_mask & (1 << block))) {
            continue;
        }
        
        uint32_t channel = tile_id * 4 + block;
        
        if (type == rfdc::TileType::DAC) {
            if (channel < 16 && info_.fd_dac[channel] >= 0) {
                uint64_t trigger = 1;
                write(info_.fd_dac[channel], &trigger, sizeof(trigger));
                std::cout << "    ✓ Triggered DAC channel " << channel << " (UIO)\n";
            }
        } else {
            if (channel < 16 && info_.fd_adc[channel] >= 0) {
                uint64_t trigger = 1;
                write(info_.fd_adc[channel], &trigger, sizeof(trigger));
                std::cout << "    ✓ Triggered ADC channel " << channel << " (UIO)\n";
            }
        }
    }
}

void RfDcApp::update_pll_sample_rate(
    rfdc::TileType type,
    uint32_t tile,
    double desired_sample_rate_mhz
)
{
    // Read current PLL configuration
    auto pll = rfdc_->get_pll_config(type, tile);

    std::cout << format_msg(
        "    • Updating PLL Fs ",
        pll.sample_rate_mhz(),
        " MHz → ",
        desired_sample_rate_mhz,
        " MHz\n"
    );

    // Reuse existing ref clock & source
    rfdc_->set_pll_config(
        type,
        tile,
        rfdc::ClockSource::Internal,
        pll.ref_clk_freq(),                // MHz
        desired_sample_rate_mhz   // MHz
    );

    // Optional: wait for lock
    for (int retry = 0; retry < 100; ++retry) {
        if (rfdc_->get_pll_lock_status(type, tile)) {
            std::cout << "    • PLL locked\n";
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    throw std::runtime_error(
        format_msg("PLL failed to lock on tile ", tile)
    );
}


// ===== CRITICAL FIX: Reduce amplitude to avoid clipping =====
RfDcApp::AdcSamples RfDcApp::read_adc_samples_i_q(
    uint32_t tile,
    uint32_t block,
    size_t num_samples
)
{
    AdcSamples captured;
    captured.I.clear();
    captured.Q.clear();
    captured.is_iq = false;

    // ------------------------------------------------------------
    // 1) Check mixer settings to determine if we're in I/Q mode
    // ------------------------------------------------------------
    auto mixer = rfdc_->get_mixer_settings(rfdc::TileType::ADC, tile, block);
    bool is_high_speed = rfdc_->check_high_speed_adc(tile);

    // Determine if this is REAL or IQ mode based on mixer
    bool is_real = 
        (mixer.mode() == rfdc::MixerMode::R2R) ||
        (mixer.type() == rfdc::MixerType::Coarse &&
         mixer.frequency() == XRFDC_COARSE_MIX_BYPASS);

    bool is_iq_mode = !is_real;

    std::cout << "--------------------------------------------------\n";
    std::cout << "ADC Read (Tile " << tile << ", Block " << block << ")\n";
    std::cout << "  Mixer Mode     : " << rfdc_->to_string(mixer.mode()) << "\n";
    std::cout << "  Mixer Type     : " << rfdc_->to_string(mixer.type()) << "\n";
    std::cout << "  High-speed ADC : " << (is_high_speed ? "YES" : "NO") << "\n";
    std::cout << "  Detected Mode  : " << (is_iq_mode ? "I/Q" : "REAL") << "\n";
    std::cout << "--------------------------------------------------\n";

    // ------------------------------------------------------------
    // 2) Calculate read size (must be word-aligned)
    //    RFDC BRAM stores 2× int16 per 32-bit word
    // ------------------------------------------------------------
    const size_t samples_per_word = 2;
    const size_t words = (num_samples + samples_per_word - 1) / samples_per_word;
    const uint32_t size_bytes = words * sizeof(uint32_t);

    // ------------------------------------------------------------
    // 3) REAL mode - read from single block (I channel only)
    // ------------------------------------------------------------
    if (!is_iq_mode) {
        std::cout << "  Reading REAL mode from block " << block << "\n";
        
        std::vector<uint8_t> raw;
        int ret = read_adc_bram_rftool_style(tile, block,size_bytes, raw);
        
        if (ret != SUCCESS) {
            throw std::runtime_error("Failed to read ADC BRAM (REAL mode)");
        }

        // Decode int16 stream
        const int16_t* s = reinterpret_cast<const int16_t*>(raw.data());
        const size_t total_samples = raw.size() / sizeof(int16_t);

        captured.I.reserve(num_samples);
        for (size_t i = 0; i < total_samples && captured.I.size() < num_samples; ++i) {
            captured.I.push_back(s[i]);
        }

        captured.is_iq = false;
        std::cout << "  ✓ Read " << captured.I.size() << " REAL samples\n";
        return captured;
    }

    // ------------------------------------------------------------
    // 4) I/Q mode - read from BOTH addr_I and addr_Q of the SAME block
    // ------------------------------------------------------------
    std::cout << "  Reading I/Q mode from block " << block << " (I and Q addresses)\n";

    const auto& adc_map = rfdc_->get_adc_map();
    uint32_t idx = tile * 4 + block;
    
    uint32_t addr_i = adc_map[idx].addr_I;
    uint32_t addr_q = adc_map[idx].addr_Q;
    
    if (addr_i == 0xFFFFFFFF || addr_q == 0xFFFFFFFF) {
        throw std::runtime_error("Invalid ADC I/Q BRAM addresses");
    }

    // Read I channel from addr_I
    std::cout << "    Reading I channel (addr=0x" << std::hex << addr_i << std::dec << ")...\n";
    std::vector<uint8_t> raw_i;
    
    void* bram_i = mmap(nullptr, size_bytes, PROT_READ | PROT_WRITE,
                        MAP_SHARED, info_.fd, addr_i);
    if (bram_i == MAP_FAILED) {
        perror("mmap ADC I");
        throw std::runtime_error("Failed to mmap ADC I channel");
    }
    
    raw_i.resize(size_bytes);
    std::memcpy(raw_i.data(), bram_i, size_bytes);
    munmap(bram_i, size_bytes);
    std::cout << "    ✓ Read I channel (" << size_bytes << " bytes)\n";

    // Read Q channel from addr_Q
    std::cout << "    Reading Q channel (addr=0x" << std::hex << addr_q << std::dec << ")...\n";
    std::vector<uint8_t> raw_q;
    
    void* bram_q = mmap(nullptr, size_bytes, PROT_READ | PROT_WRITE,
                        MAP_SHARED, info_.fd, addr_q);
    if (bram_q == MAP_FAILED) {
        perror("mmap ADC Q");
        throw std::runtime_error("Failed to mmap ADC Q channel");
    }
    
    raw_q.resize(size_bytes);
    std::memcpy(raw_q.data(), bram_q, size_bytes);
    munmap(bram_q, size_bytes);
    std::cout << "    ✓ Read Q channel (" << size_bytes << " bytes)\n";

    // ------------------------------------------------------------
    // 5) Decode I samples (2 int16 per 32-bit word)
    // ------------------------------------------------------------
    const int16_t* i_ptr = reinterpret_cast<const int16_t*>(raw_i.data());
    const size_t total_i_samples = raw_i.size() / sizeof(int16_t);
    
    captured.I.reserve(num_samples);
    for (size_t i = 0; i < total_i_samples && captured.I.size() < num_samples; ++i) {
        captured.I.push_back(i_ptr[i]);
    }

    // ------------------------------------------------------------
    // 6) Decode Q samples (2 int16 per 32-bit word)
    // ------------------------------------------------------------
    const int16_t* q_ptr = reinterpret_cast<const int16_t*>(raw_q.data());
    const size_t total_q_samples = raw_q.size() / sizeof(int16_t);
    
    captured.Q.reserve(num_samples);
    for (size_t i = 0; i < total_q_samples && captured.Q.size() < num_samples; ++i) {
        captured.Q.push_back(q_ptr[i]);
    }

    captured.is_iq = true;
    
    std::cout << "  ✓ Read " << captured.I.size() << " I samples and " 
              << captured.Q.size() << " Q samples\n";

    return captured;
}



void RfDcApp::run_string_loopback_test() 
{
    std::cout << "━━━ Running String Loopback Test ━━━\n";
    std::cout << "  Testing: DAC Tile 0 Block 0 → ADC Tile 0 Block 0\n";
    std::cout << "  Message transmission over RF loopback\n\n";
    
    try {
        const uint32_t tile = 0;
        const uint32_t block = 0;
        const uint32_t channel_mask = 0x0001;
        
        // Test message
        const std::string test_message = "HELLO";
        
        // ===== Setup Codec with REDUCED amplitude =====
        codec::StringCodec::Config codec_config;
        codec_config.modulation = codec::StringCodec::ModulationType::BPSK;
        codec_config.samples_per_symbol = 64;
        codec_config.amplitude = 6000;  // ⚠️ REDUCED from 30000 to avoid clipping!
        codec_config.use_crc = true;
        codec_config.use_preamble = false;
        
        codec::StringCodec codec(codec_config);
        
        std::cout << "Codec Configuration:\n";
        std::cout << "  Modulation: BPSK (1 bit/symbol)\n";
        std::cout << "  Samples/Symbol: " << codec_config.samples_per_symbol << "\n";
        std::cout << "  Amplitude: " << codec_config.amplitude << " ⚠️ REDUCED to avoid clipping\n";
        std::cout << "  CRC: Enabled\n";
        std::cout << "  Preamble: Enabled\n\n";
        
        // ===== DAC TRANSMISSION =====
        std::cout << "━━━ DAC Tile 0 Block 0 Setup ━━━\n";
        
        if (!rfdc_->get_pll_lock_status(rfdc::TileType::DAC, tile)) {
            throw std::runtime_error("DAC Tile 0 PLL not locked!");
        }
        
        // Encode message
        std::vector<int16_t> tx_samples = codec.encode_real(test_message);

        std::cout << "\n━━━ TX Signal Analysis ━━━\n";
        std::cout << "  Total samples: " << tx_samples.size() << "\n";

        // Show preamble region
        std::cout << "  Preamble (samples 0-511):\n  ";
        for (size_t i = 0; i < 512; i += 64) {
            int32_t sum = 0;
            for (size_t j = 0; j < 64; ++j) {
                sum += tx_samples[i + j];
            }
            int16_t avg = sum / 64;
            std::cout << "Symbol " << (i/64) << ": " << avg << " ";
        }
        std::cout << "\n";

        // Show data region  
        std::cout << "  Data (samples 512-1023):\n  ";
        for (size_t i = 512; i < 1024; i += 64) {
            int32_t sum = 0;
            for (size_t j = 0; j < 64; ++j) {
                sum += tx_samples[i + j];
            }
            int16_t avg = sum / 64;
            std::cout << "Symbol " << ((i-512)/64) << ": " << avg << " ";
        }
        std::cout << "\n";
        size_t num_samples = tx_samples.size();
        
        std::cout << "  Message: \"" << test_message << "\"\n";
        std::cout << "  Encoded to " << num_samples << " samples\n\n";
        
        std::cout << "TX length: " << tx_samples.size() << " samples\n";
        std::cout << "Expected: " << (8 + (5+2)*8) * 64 << " samples\n";

        // Reset and configure
        local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, 0x0000);
        set_local_mem_sample(rfdc::TileType::DAC, tile, block, num_samples);
        write_dac_samples(tile, block, tx_samples);
        
        std::cout << "  ✓ Triggering DAC playback...\n";
        local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, channel_mask);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // ===== ADC CAPTURE =====
        std::cout << "\n━━━ ADC Tile 0 Block 0 Capture ━━━\n";
        
        if (!rfdc_->get_pll_lock_status(rfdc::TileType::ADC, tile)) {
            throw std::runtime_error("ADC Tile 0 PLL not locked!");
        }
        
        set_local_mem_sample(rfdc::TileType::ADC, tile, block, num_samples);
        local_mem_trigger(rfdc::TileType::ADC, tile, num_samples, channel_mask);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        auto captured = read_adc_samples_i_q(tile, block, num_samples);
        std::cout << "  ✓ Captured " << captured.I.size() << " samples\n";

        // ===== ADD THIS: SCAN FOR DATA START =====
        std::cout << "\n━━━ Scanning for Data Start ━━━\n";
        std::cout << "  Symbol averages (looking for stable data region):\n";

        size_t best_data_start = 512;  // Default fallback
        double best_stability = 0;

        for (size_t i = 400; i < 800; i += 16) {  // Scan every 16 samples
            if (i + 192 >= captured.I.size()) break;  // Need 3 symbols ahead
            
            // Calculate average and variance for 3 consecutive symbols
            double stability_score = 0;
            bool all_strong = true;
            
            for (int sym = 0; sym < 3; ++sym) {
                int32_t sum = 0;
                for (size_t j = 0; j < 64; ++j) {
                    sum += captured.I[i + sym * 64 + j];
                }
                int16_t avg = sum / 64;
                
                // Calculate variance
                double variance = 0;
                for (size_t j = 0; j < 64; ++j) {
                    int16_t val = captured.I[i + sym * 64 + j];
                    double diff = val - avg;
                    variance += diff * diff;
                }
                variance /= 64;
                
                // Good data: high magnitude, low variance
                if (std::abs(avg) < 4000) all_strong = false;
                stability_score += std::abs(avg) / (variance + 1000.0);
            }
            
            if (all_strong && stability_score > best_stability) {
                best_stability = stability_score;
                best_data_start = i;
            }
            
            // Print every 64 samples for debugging
            if (i % 64 == 0) {
                int32_t sum = 0;
                for (size_t j = 0; j < 64 && (i+j) < captured.I.size(); ++j) {
                    sum += captured.I[i + j];
                }
                int16_t avg = sum / 64;
                
                std::cout << "    Sample " << std::setw(4) << i 
                        << ": avg=" << std::setw(6) << avg;
                
                if (std::abs(avg) > 5000) {
                    std::cout << " ✓ STRONG";
                }
                if (i == best_data_start) {
                    std::cout << " ← BEST";
                }
                std::cout << "\n";
            }
        }

        std::cout << "\n  → Data start detected at sample: " << best_data_start << "\n";
        std::cout << "    (Expected: 512, Actual offset: " 
                << (int)best_data_start - 512 << " samples)\n";

        // Show samples at detected data start
        std::cout << "  First 32 samples at detected start:\n  ";
        for (size_t i = best_data_start; i < best_data_start + 32 && i < captured.I.size(); ++i) {
            std::cout << captured.I[i];
            if ((i - best_data_start + 1) % 16 == 0) std::cout << "\n  ";
            else std::cout << " ";
        }
        std::cout << "\n";

        // ===== NOW USE THIS IN ATTEMPT 2 =====

        std::cout << "\n━━━ RX Signal Analysis ━━━\n";

        // 1. Check signal strength
        auto minmax = std::minmax_element(captured.I.begin(), captured.I.end());
        std::cout << "  RX Range: [" << *minmax.first << ", " << *minmax.second << "]\n";
        std::cout << "  RX P-P: " << (*minmax.second - *minmax.first) << "\n";

        // ===== SIGNAL ANALYSIS =====
        codec.analyze_signal(captured.I, captured.Q);
        
        // Check for clipping
        int clipped = std::count_if(captured.I.begin(), captured.I.end(),
                                    [](int16_t s) { return std::abs(s) >= 32760; });
        if (clipped > captured.I.size() * 0.01) {  // More than 1%
            std::cout << "\n  ⚠️⚠️⚠️ SEVERE CLIPPING DETECTED ⚠️⚠️⚠️\n";
            std::cout << "  " << clipped << "/" << captured.I.size() 
                     << " samples are clipped (" 
                     << (100.0 * clipped / captured.I.size()) << "%)\n";
            std::cout << "  REDUCE AMPLITUDE FURTHER or adjust DAC/ADC gains!\n\n";
        }
        
        codec.save_constellation(captured.I, captured.Q, "constellation.csv");
        
        // ===== TRY DECODE WITH RELAXED PREAMBLE =====
        std::cout << "\n━━━ Message Decoding (Attempt 1: With Preamble) ━━━\n";
        
        std::string decoded_message;
        bool decode_success = codec.decode_real(captured.I, decoded_message);
        
        // If preamble detection fails, try without it
        if (!decode_success) {
            std::cout << "\n━━━ Attempt 2: Manual Preamble Skip (Adaptive) ━━━\n";

            // Use detected data start instead of fixed offset
            const size_t preamble_samples = best_data_start;  // ← CHANGED

            if (captured.I.size() <= preamble_samples) {
                std::cerr << "  ✗ Signal too short\n";
                return;
            }

            std::vector<int16_t> data_only(
                captured.I.begin() + preamble_samples,
                captured.I.end()
            );

            std::cout << "  Using detected data start: " << preamble_samples << " samples\n";
            std::cout << "  Decoding data: " << data_only.size() << " samples\n";

            // Show first data samples
            std::cout << "  First 64 data samples:\n  ";
            for (size_t i = 0; i < 64 && i < data_only.size(); ++i) {
                std::cout << data_only[i];
                if ((i+1) % 16 == 0 && i < 63) std::cout << "\n  ";
                else if (i < 63) std::cout << " ";
            }
            std::cout << "\n\n";

            // Decode WITHOUT preamble
            codec::StringCodec::Config cfg_no_preamble = codec_config;
            cfg_no_preamble.use_preamble = false;
            codec::StringCodec decoder(cfg_no_preamble);

            std::string decoded;
            bool success = decoder.decode_real(data_only, decoded);

            if (success && decoded == test_message) {
                std::cout << "\n━━━ Result ━━━\n";
                std::cout << "  ✓✓✓ SUCCESS! ✓✓✓\n";
                std::cout << "  TX: \"" << test_message << "\"\n";
                std::cout << "  RX: \"" << decoded << "\"\n";
                return;
            }
        }
        
        // ===== RESULT =====
        std::cout << "\n━━━ Result ━━━\n";
        if (decode_success && !decoded_message.empty()) {
            std::cout << "  ✓✓✓ DECODE SUCCESS! ✓✓✓\n";
            std::cout << "  Transmitted: \"" << test_message << "\"\n";
            std::cout << "  Received:    \"" << decoded_message << "\"\n";
            
            if (decoded_message == test_message) {
                std::cout << "\n  🎉 PERFECT MATCH!\n";
            } else {
                std::cout << "\n  ⚠ Partial match - some errors\n";
                std::cout << "  TX length: " << test_message.length() 
                         << ", RX length: " << decoded_message.length() << "\n";
            }
        } else {
            std::cout << "  ✗ DECODE FAILED\n";
            
            if (!decoded_message.empty()) {
                std::cout << "  Partial: \"" << decoded_message << "\"\n";
            }
            
            std::cout << "\n  Next Steps:\n";
            std::cout << "  1. ⚠️ CRITICAL: Reduce amplitude further (currently " 
                     << codec_config.amplitude << ")\n";
            std::cout << "  2. Check constellation.csv - symbols should cluster at ±amplitude\n";
            std::cout << "  3. Verify loopback cable is connected\n";
            std::cout << "  4. Try without CRC: config.use_crc = false\n";
        }
        
        // Save for analysis
        auto dac_pll = rfdc_->get_pll_config(rfdc::TileType::DAC, tile);
        auto adc_pll = rfdc_->get_pll_config(rfdc::TileType::ADC, tile);
        
        save_samples_to_csv(tx_samples, dac_pll.sample_rate() * 1e9, 
                           "string_tx.csv", "# TX\n");
        save_samples_to_csv(captured.I, adc_pll.sample_rate() * 1e9,
                           "string_rx.csv", "# RX\n");
        
        std::cout << "\n  Files: string_tx.csv, string_rx.csv, constellation.csv\n";
        
    } catch (const std::exception& e) {
        std::cerr << "\n✗ Error: " << e.what() << "\n";
    }
}

// ===== ALTERNATIVE: Try even simpler test without codec complexity =====

void RfDcApp::run_simple_pattern_test()
{
    std::cout << "━━━ Simple Pattern Test (No Encoding) ━━━\n";
    
    const uint32_t tile = 0;
    const uint32_t block = 0;
    const uint32_t channel_mask = 0x0001;
    
    // Create simple alternating pattern: +8000, -8000, +8000, -8000...
    const size_t num_samples = 1024;
    std::vector<int16_t> pattern;
    
    for (size_t i = 0; i < num_samples; ++i) {
        // Alternate every 16 samples (slower transitions)
        pattern.push_back(((i / 16) % 2) ? 8000 : -8000);
    }
    
    std::cout << "  Transmitting square wave pattern...\n";
    std::cout << "  Amplitude: ±8000\n";
    std::cout << "  Samples: " << num_samples << "\n\n";
    
    // DAC
    local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, 0x0000);
    set_local_mem_sample(rfdc::TileType::DAC, tile, block, num_samples);
    write_dac_samples(tile, block, pattern);
    local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, channel_mask);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // ADC
    set_local_mem_sample(rfdc::TileType::ADC, tile, block, num_samples);
    local_mem_trigger(rfdc::TileType::ADC, tile, num_samples, channel_mask);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    auto captured = read_adc_samples_i_q(tile, block, num_samples);
    
    std::cout << "  Captured " << captured.I.size() << " samples\n\n";
    
    // Analyze
    auto minmax = std::minmax_element(captured.I.begin(), captured.I.end());
    int clipped = std::count_if(captured.I.begin(), captured.I.end(),
                                [](int16_t s) { return std::abs(s) >= 32760; });
    
    std::cout << "  RX Range: [" << *minmax.first << ", " << *minmax.second << "]\n";
    std::cout << "  RX P-P: " << (*minmax.second - *minmax.first) << "\n";
    std::cout << "  Clipped: " << clipped << " samples\n";
    
    if (clipped > 0) {
        std::cout << "  ⚠️ Still clipping! Reduce amplitude below 8000\n";
    } else if ((*minmax.second - *minmax.first) < 1000) {
        std::cout << "  ⚠️ Signal too weak! Increase amplitude\n";
    } else {
        std::cout << "  ✓ Good signal level - no clipping\n";
        
        // Check for correct alternating pattern
        int transitions = 0;
        for (size_t i = 1; i < captured.I.size(); ++i) {
            if ((captured.I[i] > 0) != (captured.I[i-1] > 0)) {
                transitions++;
            }
        }
        
        std::cout << "  Transitions detected: " << transitions << "\n";
        std::cout << "  Expected: ~" << (num_samples / 16) << "\n";
    }
    
    // Display first few samples
    std::cout << "\n  First 32 samples:\n  ";
    for (size_t i = 0; i < 32 && i < captured.I.size(); ++i) {
        std::cout << captured.I[i];
        if (i < 31) std::cout << " ";
    }
    std::cout << "\n";
    
    save_samples_to_csv(pattern, 1e9, "pattern_tx.csv", "# TX pattern\n");
    save_samples_to_csv(captured.I, 1e9, "pattern_rx.csv", "# RX pattern\n");
}

// ===== AMPLITUDE CALIBRATION TEST =====

void RfDcApp::calibrate_amplitude()
{
    std::cout << "━━━ Amplitude Calibration Test ━━━\n";
    std::cout << "  Finding optimal amplitude without clipping...\n\n";
    
    const uint32_t tile = 0;
    const uint32_t block = 0;
    const uint32_t channel_mask = 0x0001;
    const size_t num_samples = 512;
    
    // Test different amplitudes
    const std::vector<int16_t> test_amplitudes = {
        1000, 2000, 4000, 6000, 8000, 10000, 12000, 15000, 20000
    };
    
    for (auto amp : test_amplitudes) {
        std::cout << "  Testing amplitude: " << amp << "... ";
        
        // Create test pattern
        std::vector<int16_t> pattern;
        for (size_t i = 0; i < num_samples; ++i) {
            pattern.push_back(((i / 8) % 2) ? amp : -amp);
        }
        
        // Transmit
        local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, 0x0000);
        set_local_mem_sample(rfdc::TileType::DAC, tile, block, num_samples);
        write_dac_samples(tile, block, pattern);
        local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, channel_mask);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Receive
        set_local_mem_sample(rfdc::TileType::ADC, tile, block, num_samples);
        local_mem_trigger(rfdc::TileType::ADC, tile, num_samples, channel_mask);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        auto captured = read_adc_samples_i_q(tile, block, num_samples);
        
        // Check for clipping
        int clipped = std::count_if(captured.I.begin(), captured.I.end(),
                                    [](int16_t s) { return std::abs(s) >= 32760; });
        
        auto minmax = std::minmax_element(captured.I.begin(), captured.I.end());
        int16_t pp = *minmax.second - *minmax.first;
        
        if (clipped > 0) {
            std::cout << "CLIPPED (" << clipped << " samples) ❌\n";
            std::cout << "    → Amplitude " << amp << " is TOO HIGH\n";
            break;
        } else {
            std::cout << "OK, P-P=" << pp << " ✓\n";
            if (amp == test_amplitudes.back()) {
                std::cout << "\n  ✓ Optimal amplitude: " << amp << "\n";
            }
        }
    }
    
    std::cout << "\n  Use the highest amplitude that doesn't clip.\n";
}


// Helper function to calculate bit error rate
double RfDcApp::calculate_ber(const std::string& original, const std::string& decoded)
{
    size_t errors = 0;
    size_t total_bits = 0;
    
    size_t min_len = std::min(original.length(), decoded.length());
    
    for (size_t i = 0; i < min_len; ++i) {
        uint8_t xor_result = original[i] ^ decoded[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (xor_result & (1 << bit)) {
                errors++;
            }
            total_bits++;
        }
    }
    
    // Count extra/missing bytes as all bits wrong
    if (original.length() > decoded.length()) {
        errors += (original.length() - decoded.length()) * 8;
        total_bits += (original.length() - decoded.length()) * 8;
    } else if (decoded.length() > original.length()) {
        errors += (decoded.length() - original.length()) * 8;
        total_bits += (decoded.length() - original.length()) * 8;
    }
    
    return total_bits > 0 ? static_cast<double>(errors) / total_bits : 0.0;
}

// ===== Example Usage in main() or run() =====

// In your RfDcApp::run() method, you can call:
// run_string_loopback_test();

// Or run different modulation schemes:
void RfDcApp::test_all_modulations()
{
    const std::vector<codec::StringCodec::ModulationType> modes = {
        codec::StringCodec::ModulationType::BPSK,
        codec::StringCodec::ModulationType::QPSK,
        codec::StringCodec::ModulationType::PSK8,
        codec::StringCodec::ModulationType::QAM16
    };
    
    const std::vector<std::string> mode_names = {
        "BPSK (1 bit/symbol)",
        "QPSK (2 bits/symbol)",
        "8PSK (3 bits/symbol)",
        "16QAM (4 bits/symbol)"
    };
    
    for (size_t i = 0; i < modes.size(); ++i) {
        std::cout << "\n\n";
        std::cout << "╔════════════════════════════════════════════╗\n";
        std::cout << "║  Testing: " << std::left << std::setw(32) << mode_names[i] << "║\n";
        std::cout << "╚════════════════════════════════════════════╝\n\n";
        
        codec::StringCodec::Config config;
        config.modulation = modes[i];
        config.samples_per_symbol = 32;
        config.amplitude = 25000;
        
        // Run test with this configuration
        // ... (adapt run_string_loopback_test to accept config)
    }
}

// Generate complex I/Q sine wave (returns AdcSamples for consistency)
RfDcApp::AdcSamples RfDcApp::generate_iq_sine_wave(
    double frequency_hz,           // Baseband frequency
    double dac_pll_rate_hz,        // PLL rate
    uint32_t dac_interpolation,
    size_t num_samples,
    int16_t amplitude,
    double noise_dbfs
)
{
    AdcSamples samples;
    samples.I.resize(num_samples);
    samples.Q.resize(num_samples);
    samples.is_iq = true;
    
    // Calculate fabric sample rate
    const double sample_rate_hz = dac_pll_rate_hz / dac_interpolation;
    
    const double two_pi = 2.0 * M_PI;
    const double phase_increment = two_pi * frequency_hz / sample_rate_hz;
    
    // Optional noise
    const double noise_rms = amplitude * std::pow(10.0, noise_dbfs / 20.0);
    std::default_random_engine rng(42);
    std::normal_distribution<double> gaussian(0.0, 1.0);
    
    for (size_t i = 0; i < num_samples; ++i) {
        const double phase = phase_increment * static_cast<double>(i);
        
        // Generate I (cosine) and Q (sine) for complex exponential
        const double i_signal = amplitude * std::cos(phase);
        const double q_signal = amplitude * std::sin(phase);
        
        // Add noise if specified
        const double i_noise = (noise_dbfs < 0.0) ? noise_rms * gaussian(rng) : 0.0;
        const double q_noise = (noise_dbfs < 0.0) ? noise_rms * gaussian(rng) : 0.0;
        
        double i_sample = i_signal + i_noise;
        double q_sample = q_signal + q_noise;
        
        // Clamp
        if (i_sample > 32767.0) i_sample = 32767.0;
        else if (i_sample < -32768.0) i_sample = -32768.0;
        
        if (q_sample > 32767.0) q_sample = 32767.0;
        else if (q_sample < -32768.0) q_sample = -32768.0;
        
        samples.I[i] = static_cast<int16_t>(std::lrint(i_sample));
        samples.Q[i] = static_cast<int16_t>(std::lrint(q_sample));
    }
    
    std::cout << "  ✓ Generated " << num_samples << " I/Q samples\n";
    std::cout << "      Baseband Freq:          " << frequency_hz / 1e6 << " MHz\n";
    std::cout << "      DAC PLL Rate:           " << dac_pll_rate_hz / 1e9 << " GSPS\n";
    std::cout << "      Fabric Rate:            " << sample_rate_hz / 1e6 << " MSPS\n";
    std::cout << "      Interpolation:          " << dac_interpolation << "x\n";
    std::cout << "      Amplitude (I/Q):        " << amplitude << " LSB\n";
    
    return samples;
}

// Write I/Q samples to DAC BRAM (I and Q blocks)
void RfDcApp::write_dac_iq_samples(
    uint32_t tile,
    uint32_t i_block,
    uint32_t q_block,
    const AdcSamples& samples
)
{
    if (!samples.is_iq) {
        throw std::runtime_error("Expected I/Q samples but got REAL samples");
    }
    
    if (samples.I.size() != samples.Q.size()) {
        throw std::runtime_error("I/Q size mismatch");
    }
    
    std::cout << "  Writing I/Q to DAC[" << tile << "][" 
              << i_block << "/" << q_block << "]...\n";
    
    // Write I channel
    const uint32_t size_bytes_i = samples.I.size() * sizeof(int16_t);
    std::vector<uint8_t> raw_i(size_bytes_i);
    std::memcpy(raw_i.data(), samples.I.data(), size_bytes_i);
    
    int ret = write_dac_bram_rftool_style(tile, i_block, size_bytes_i, raw_i);
    if (ret != SUCCESS) {
        throw std::runtime_error("Failed to write DAC I samples");
    }
    
    // Write Q channel
    const uint32_t size_bytes_q = samples.Q.size() * sizeof(int16_t);
    std::vector<uint8_t> raw_q(size_bytes_q);
    std::memcpy(raw_q.data(), samples.Q.data(), size_bytes_q);
    
    ret = write_dac_bram_rftool_style(tile, q_block, size_bytes_q, raw_q);
    if (ret != SUCCESS) {
        throw std::runtime_error("Failed to write DAC Q samples");
    }
    
    std::cout << "    ✓ Wrote " << samples.I.size() 
              << " I samples and " << samples.Q.size() << " Q samples\n";
}

// Save I/Q samples to separate CSV files
void RfDcApp::save_iq_samples_to_csv(
    const AdcSamples& samples,
    double sample_rate_hz,
    const std::string& i_filename,
    const std::string& q_filename,
    const std::string& metadata
)
{
    if (!samples.is_iq) {
        throw std::runtime_error("Expected I/Q samples but got REAL samples");
    }
    
    double time_step_ns = (1.0 / sample_rate_hz) * 1e9;
    
    // Save I channel
    {
        std::ofstream file(i_filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open I file: " + i_filename);
        }
        
        if (!metadata.empty()) {
            file << metadata;
        }
        file << "# channel: I\n";
        file << "sample_index,time_ns,value\n";
        
        for (size_t i = 0; i < samples.I.size(); ++i) {
            file << i << "," << (i * time_step_ns) << "," << samples.I[i] << "\n";
        }
    }
    
    // Save Q channel
    {
        std::ofstream file(q_filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open Q file: " + q_filename);
        }
        
        if (!metadata.empty()) {
            file << metadata;
        }
        file << "# channel: Q\n";
        file << "sample_index,time_ns,value\n";
        
        for (size_t i = 0; i < samples.Q.size(); ++i) {
            file << i << "," << (i * time_step_ns) << "," << samples.Q[i] << "\n";
        }
    }
    
    std::cout << "    ✓ Saved: " << i_filename << " and " << q_filename << "\n";
}

// ===== Updated I/Q Loopback Test =====

void RfDcApp::run_iq_loopback_test() 
{
    std::cout << "━━━ Running I/Q Loopback Test ━━━\n";
    std::cout << "  Testing: DAC Tile 0 Blocks 0/1 (I/Q) → ADC Tile 0 Blocks 0/1 (I/Q)\n";
    std::cout << "  (Loopback cables connected)\n\n";
    
    try {
        const uint32_t tile = 0;
        const uint32_t i_block = 0;
        const uint32_t q_block = 1;
        const uint32_t channel_mask = 0x0003;  // Enable both blocks 0 and 1
        const size_t num_samples = 16384;
        const double test_frequency = 50e6;  // 50 MHz baseband
        
        // Get DAC configuration
        auto dac_pll = rfdc_->get_pll_config(rfdc::TileType::DAC, tile);
        double dac_pll_rate_hz = dac_pll.sample_rate() * 1e9;
        uint32_t dac_interpolation = rfdc_->get_interpolation_factor(tile, i_block);
        
        // Get ADC configuration
        auto adc_pll = rfdc_->get_pll_config(rfdc::TileType::ADC, tile);
        double adc_pll_rate_hz = adc_pll.sample_rate() * 1e9;
        uint32_t adc_decimation = rfdc_->get_decimation_factor(tile, i_block);
        
        std::cout << "Configuration:\n";
        std::cout << "  DAC PLL Rate: " << dac_pll.sample_rate() << " GSPS\n";
        std::cout << "  DAC Interpolation: " << dac_interpolation << "x\n";
        std::cout << "  ADC PLL Rate: " << adc_pll.sample_rate() << " GSPS\n";
        std::cout << "  ADC Decimation: " << adc_decimation << "x\n\n";
        
        // Get mixer modes to verify I/Q configuration
        auto dac_mixer_i = rfdc_->get_mixer_settings(rfdc::TileType::DAC, tile, i_block);
        auto dac_mixer_q = rfdc_->get_mixer_settings(rfdc::TileType::DAC, tile, q_block);
        
        std::cout << "  DAC I Mixer Mode: " << rfdc_->to_string(dac_mixer_i.mode()) << "\n";
        std::cout << "  DAC Q Mixer Mode: " << rfdc_->to_string(dac_mixer_q.mode()) << "\n\n";
        
        // ===== DAC I/Q PLAYBACK =====
        std::cout << "━━━ DAC Tile 0 Blocks 0/1 (I/Q) Setup ━━━\n";
        
        if (!rfdc_->get_pll_lock_status(rfdc::TileType::DAC, tile)) {
            throw std::runtime_error("DAC Tile 0 PLL not locked!");
        }
        std::cout << "  ✓ DAC PLL locked\n";
        
        std::cout << "\n  Step 1: Reset DAC memory\n";
        local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, 0x0000);
        
        std::cout << "  Step 2: Configure sample count for I/Q\n";
        set_local_mem_sample(rfdc::TileType::DAC, tile, i_block, num_samples);
        set_local_mem_sample(rfdc::TileType::DAC, tile, q_block, num_samples);
        
        std::cout << "  Step 3: Generate I/Q sine wave\n";
        auto iq_samples = generate_iq_sine_wave(
            test_frequency,
            dac_pll_rate_hz,
            dac_interpolation,
            num_samples,
            30000  // amplitude
        );
        
        std::cout << "  Step 4: Write I/Q samples to DAC BRAM\n";
        write_dac_iq_samples(tile, i_block, q_block, iq_samples);
        
        // Save DAC data
        std::stringstream dac_meta;
        dac_meta << "# RFDC DAC Configuration (I/Q Mode)\n";
        dac_meta << "# type: DAC\n";
        dac_meta << "# tile: " << tile << "\n";
        dac_meta << "# i_block: " << i_block << "\n";
        dac_meta << "# q_block: " << q_block << "\n";
        dac_meta << "# pll_rate_mhz: " << (dac_pll_rate_hz / 1e6) << "\n";
        dac_meta << "# interpolation: " << dac_interpolation << "\n";
        dac_meta << "# baseband_frequency_mhz: " << (test_frequency / 1e6) << "\n";
        dac_meta << "# num_samples: " << num_samples << "\n";
        dac_meta << "# amplitude: 30000\n";
        
        save_iq_samples_to_csv(
            iq_samples,
            dac_pll_rate_hz / dac_interpolation,
            "dac_i_t0_b0_50MHz.csv",
            "dac_q_t0_b1_50MHz.csv",
            dac_meta.str()
        );
        
        std::cout << "  Step 5: **TRIGGER DAC I/Q playback**\n";
        local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, channel_mask);
        
        std::cout << "    Waiting 200ms for DAC to stabilize...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "  ✓ DAC is now outputting I/Q RF signal\n\n";
        
        // ===== ADC I/Q CAPTURE =====
        std::cout << "━━━ ADC Tile 0 Blocks 0/1 (I/Q) Capture ━━━\n";
        
        if (!rfdc_->get_pll_lock_status(rfdc::TileType::ADC, tile)) {
            throw std::runtime_error("ADC Tile 0 PLL not locked!");
        }
        std::cout << "  ✓ ADC PLL locked\n";
        
        std::cout << "\n  Step 1: Configure ADC capture size for I/Q\n";
        set_local_mem_sample(rfdc::TileType::ADC, tile, i_block, num_samples);
        set_local_mem_sample(rfdc::TileType::ADC, tile, q_block, num_samples);
        
        std::cout << "  Step 2: **TRIGGER ADC I/Q capture**\n";
        local_mem_trigger(rfdc::TileType::ADC, tile, num_samples, channel_mask);
        
        std::cout << "    Waiting 200ms for capture to complete...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "  Step 3: Reading captured I/Q data from ADC BRAM\n";
        // Use the updated read_adc_samples_i_q which now handles I/Q mode
        auto captured_iq = read_adc_samples_i_q(tile, i_block, num_samples);
        
        std::cout << "  ✓ Captured " << captured_iq.I.size() << " I samples and " 
                  << captured_iq.Q.size() << " Q samples\n";
        std::cout << "  ✓ Mode: " << (captured_iq.is_iq ? "I/Q" : "REAL") << "\n";
        
        // Save ADC data
        std::stringstream adc_meta;
        adc_meta << "# RFDC ADC Configuration (I/Q Mode)\n";
        adc_meta << "# type: ADC\n";
        adc_meta << "# tile: " << tile << "\n";
        adc_meta << "# i_block: " << i_block << "\n";
        adc_meta << "# q_block: " << q_block << "\n";
        adc_meta << "# pll_rate_mhz: " << (adc_pll_rate_hz / 1e6) << "\n";
        adc_meta << "# decimation: " << adc_decimation << "\n";
        adc_meta << "# baseband_frequency_mhz: " << (test_frequency / 1e6) << "\n";
        adc_meta << "# num_samples: " << num_samples << "\n";
        
        save_iq_samples_to_csv(
            captured_iq,
            adc_pll_rate_hz / adc_decimation,
            "adc_i_t0_b0_capture.csv",
            "adc_q_t0_b1_capture.csv",
            adc_meta.str()
        );
        
        // ===== ANALYSIS =====
        std::cout << "\n━━━ Analysis ━━━\n";
        
        std::cout << "  I Channel - First 16 samples:\n    ";
        for (size_t i = 0; i < std::min(size_t(16), captured_iq.I.size()); ++i) {
            std::cout << captured_iq.I[i] << " ";
        }
        std::cout << "\n";
        
        std::cout << "  Q Channel - First 16 samples:\n    ";
        for (size_t i = 0; i < std::min(size_t(16), captured_iq.Q.size()); ++i) {
            std::cout << captured_iq.Q[i] << " ";
        }
        std::cout << "\n\n";
        
        // Calculate statistics for both channels
        auto i_max = std::max_element(captured_iq.I.begin(), captured_iq.I.end());
        auto i_min = std::min_element(captured_iq.I.begin(), captured_iq.I.end());
        auto q_max = std::max_element(captured_iq.Q.begin(), captured_iq.Q.end());
        auto q_min = std::min_element(captured_iq.Q.begin(), captured_iq.Q.end());
        
        int16_t i_pp = *i_max - *i_min;
        int16_t q_pp = *q_max - *q_min;
        
        // Calculate I/Q balance
        double i_rms = 0.0, q_rms = 0.0;
        for (size_t i = 0; i < captured_iq.I.size(); ++i) {
            i_rms += captured_iq.I[i] * captured_iq.I[i];
            q_rms += captured_iq.Q[i] * captured_iq.Q[i];
        }
        i_rms = std::sqrt(i_rms / captured_iq.I.size());
        q_rms = std::sqrt(q_rms / captured_iq.Q.size());
        double iq_imbalance_db = 20 * std::log10(i_rms / (q_rms + 1e-10));
        
        std::cout << "  I Channel Statistics:\n";
        std::cout << "    Max: " << *i_max << ", Min: " << *i_min 
                  << ", P-P: " << i_pp << ", RMS: " << static_cast<int>(i_rms) << "\n";
        
        std::cout << "  Q Channel Statistics:\n";
        std::cout << "    Max: " << *q_max << ", Min: " << *q_min 
                  << ", P-P: " << q_pp << ", RMS: " << static_cast<int>(q_rms) << "\n";
        
        std::cout << "  I/Q Imbalance: " << std::fixed << std::setprecision(2) 
                  << iq_imbalance_db << " dB\n";
        
        std::cout << "\n━━━ Result ━━━\n";
        if (captured_iq.is_iq && i_pp > 1000 && q_pp > 1000) {
            std::cout << "  ✓✓✓ I/Q LOOPBACK SUCCESS! ✓✓✓\n";
            std::cout << "  Strong signals detected on both I and Q channels!\n";
            
            if (std::abs(iq_imbalance_db) < 1.0) {
                std::cout << "  ✓ Excellent I/Q balance (< 1 dB)\n";
            } else if (std::abs(iq_imbalance_db) < 3.0) {
                std::cout << "  ✓ Good I/Q balance (< 3 dB)\n";
            } else {
                std::cout << "  ⚠ I/Q imbalance detected - may need QMC calibration\n";
            }
            
            std::cout << "\n  To analyze results:\n";
            std::cout << "    python3 plot_iq_loopback.py \\\n";
            std::cout << "      dac_i_t0_b0_50MHz.csv dac_q_t0_b1_50MHz.csv \\\n";
            std::cout << "      adc_i_t0_b0_capture.csv adc_q_t0_b1_capture.csv\n";
        } else if (!captured_iq.is_iq) {
            std::cout << "  ✗ ADC NOT IN I/Q MODE!\n";
            std::cout << "  ADC detected REAL mode instead of I/Q\n";
            std::cout << "  Check mixer configuration (should be C2C)\n";
        } else {
            std::cout << "  ✗ WEAK/NO SIGNAL DETECTED\n";
            std::cout << "  I P-P: " << i_pp << ", Q P-P: " << q_pp << "\n";
            std::cout << "  Check:\n";
            std::cout << "    - Loopback cables connected to correct channels\n";
            std::cout << "    - DAC/ADC gains and attenuation\n";
            std::cout << "    - Mixer mode configuration (should be C2C)\n";
        }
        
    } catch (const std::exception& e) {
        std::cerr << "\n✗ Error: " << e.what() << "\n";
    }
}


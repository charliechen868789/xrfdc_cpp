#include "RfdcApp.hpp"
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

void RfDcApp::run() {
    try {
        std::cout << "Starting RF Data Converter Application...\n\n";
        
        // Initialize GPIO first
        init_gpio();
        std::cout << "\n✓ finish init_gpio!\n";
        
        // Initialize clocks
        initialize_clocks();
        std::cout << "\n✓ finish initialize_clocks!\n";
        
        // Initialize RFDC
        initialize_rfdc();
        std::cout << "\n✓ finish initialize_rfdc!\n";

         // **Initialize LocalMem controller**
        local_mem_ = std::make_unique<local_mem::LocalMem>(rfdc_.get());
        std::cout << "\n✓ finish initialize_local_mem!\n";  

        clock_wiz_ = std::make_unique<clock_wizard::ClockWizard>(rfdc_.get());
        std::cout << "\n✓ finish initialize_clock_wiz!\n"; 
        // Initialize memory mapping (for clock wizards)
        initialize_memory_mapping();
        std::cout << "\n✓ finish initialize_memory_mapping!\n";
        
        // **Initialize UIO memory (CRITICAL!) - using RFTool compatible API**
        if (init_mem() != SUCCESS) {
            throw std::runtime_error("Failed to initialize UIO memory");
        }
        std::cout << "\n✓ finish init_mem!\n";
        
        // Configure tiles
        configure_dac_tiles();
        std::cout << "\n✓ finish configure_dac_tiles!\n";
        
        configure_adc_tiles();
        std::cout << "\n✓ finish configure_adc_tiles!\n";
        
        initialize_mmcm_adc();
        std::cout << "\n✓ finish initialize_mmcm_adc!\n";
        
        initialize_mmcm_dac();
        std::cout << "\n✓ finish initialize_mmcm_dac!\n";
        
        verify_configuration();
        display_status();
        
        // Run tests
        run_loopback_test();
        
        std::cout << "\n✓ Application completed successfully!\n";
        
        // Cleanup
        deinit_mem();
        deinit_gpio();
        
    } catch (const rfdc::RFDCException& e) {
        std::cerr << "\n✗ RFDC Error: " << e.what() 
                  << " (code: " << e.error_code() << ")\n";
        deinit_mem();
        deinit_gpio();
        throw;
    } catch (const std::exception& e) {
        std::cerr << "\n✗ Error: " << e.what() << "\n";
        deinit_mem();
        deinit_gpio();
        throw;
    }
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

int RfDcApp::read_data_from_memory_bram(uint32_t block_id, int tile_id,
                                        uint32_t size, std::vector<int16_t>& samples)
{
    std::cout << "  Reading from ADC[" << tile_id << "][" << block_id << "]...\n";
    
    // Validate size
    if (size > FIFO_SIZE) {
        std::cerr << "    ✗ Size " << size << " exceeds FIFO size " << FIFO_SIZE << "\n";
        return FAIL;
    }
    
    if ((size == 0) || (size % ADC_DAC_SZ_ALIGNMENT) != 0) {
        std::cerr << "    ✗ Size must be multiple of " << ADC_DAC_SZ_ALIGNMENT << " bytes\n";
        return FAIL;
    }
    
    // Get channel index
    uint32_t channel = tile_id * 4 + block_id;
    if (channel >= 16) {
        std::cerr << "    ✗ Invalid channel " << channel << "\n";
        return FAIL;
    }
    
    // Enable FIFO for this tile BEFORE reading
    change_fifo_stat(0, tile_id, 1);  // 0 = ADC
    
    // Get ADC map
    const auto& adc_map = rfdc_->get_adc_map();
    uint16_t numblockpertile = 4;  // ZCU216 has 4 blocks per tile
    uint32_t idx = tile_id * numblockpertile + block_id;
    uint32_t paddr_adc = adc_map[idx].addr_I;
    
    if (paddr_adc == 0xFFFFFFFF) {
        std::cerr << "    ✗ Channel not available\n";
        return FAIL;
    }
    
    // Map BRAM
    void* bram_base_adc = mmap(nullptr, size, PROT_READ | PROT_WRITE,
                               MAP_SHARED, info_.fd, paddr_adc);
    if (bram_base_adc == MAP_FAILED) {
        std::cerr << "    ✗ Failed to mmap BRAM at 0x" << std::hex << paddr_adc << std::dec << "\n";
        return FAIL;
    }
    
    signed int* vaddr_adc = (signed int*)bram_base_adc;
    
    // Read and convert from 32-bit to 16-bit samples
    samples.resize(size / sizeof(int16_t));
    for (size_t i = 0; i < samples.size() / 2; i++) {
        uint32_t word = vaddr_adc[i];
        samples[i * 2] = (int16_t)(word & 0xFFFF);
        if (i * 2 + 1 < samples.size()) {
            samples[i * 2 + 1] = (int16_t)((word >> 16) & 0xFFFF);
        }
    }
    
    munmap(bram_base_adc, size);
    
    std::cout << "    ✓ Read " << samples.size() << " samples (" << size << " bytes)\n";
    
    return SUCCESS;
}

void RfDcApp::init_gpio() 
{
    std::cout << "━━━ Initializing GPIO Pins ━━━\n";
    
    uint32_t max_dac = MAX_DAC_PER_TILE * MAX_DAC_TILE;
    
    // Initialize DAC user select GPIOs
    std::cout << "  Configuring DAC user select GPIOs...\n";
    for (uint32_t i = 0; i < max_dac; i++) {
        auto gpio = std::make_unique<gpio::Gpio>(dac_userselect_gpio[i]);
        
        if (!gpio->enable()) {
            throw std::runtime_error("Unable to enable DAC user select GPIO " + std::to_string(i));
        }
        
        if (!gpio->set_direction(gpio::Gpio::Direction::Output)) {
            throw std::runtime_error("Unable to set direction for DAC user select GPIO " + std::to_string(i));
        }
        
        dac_userselect_gpios_.push_back(std::move(gpio));
    }
    std::cout << "  ✓ DAC user select GPIOs initialized (" << max_dac << " pins)\n";
    
    // Initialize DAC MTS clock enable GPIOs
    std::cout << "  Configuring DAC MTS clock GPIOs...\n";
    for (uint32_t i = 0; i < MAX_DAC_TILE; i++) {
        auto gpio = std::make_unique<gpio::Gpio>(dac_mts_clk_en[i]);
        
        if (!gpio->enable()) {
            throw std::runtime_error("Unable to enable DAC MTS clock GPIO " + std::to_string(i));
        }
        
        if (!gpio->set_direction(gpio::Gpio::Direction::Output)) {
            throw std::runtime_error("Unable to set direction for DAC MTS clock GPIO " + std::to_string(i));
        }
        
        if (!gpio->set_value(gpio::Gpio::Value::High)) {
            throw std::runtime_error("Unable to set value for DAC MTS clock GPIO " + std::to_string(i));
        }
        
        dac_mts_clk_gpios_.push_back(std::move(gpio));
    }
    std::cout << "  ✓ DAC MTS clock GPIOs initialized and enabled\n";
    
    // Initialize ADC MTS clock enable GPIOs
    std::cout << "  Configuring ADC MTS clock GPIOs...\n";
    for (uint32_t i = 0; i < MAX_ADC_TILE; i++) {
        auto gpio = std::make_unique<gpio::Gpio>(adc_mts_clk_en[i]);
        
        if (!gpio->enable()) {
            throw std::runtime_error("Unable to enable ADC MTS clock GPIO " + std::to_string(i));
        }
        
        if (!gpio->set_direction(gpio::Gpio::Direction::Output)) {
            throw std::runtime_error("Unable to set direction for ADC MTS clock GPIO " + std::to_string(i));
        }
        
        if (!gpio->set_value(gpio::Gpio::Value::High)) {
            throw std::runtime_error("Unable to set value for ADC MTS clock GPIO " + std::to_string(i));
        }
        
        adc_mts_clk_gpios_.push_back(std::move(gpio));
    }
    std::cout << "  ✓ ADC MTS clock GPIOs initialized and enabled\n";
    
    // Initialize ADC AXI switch reset GPIO
    std::cout << "  Configuring ADC AXI switch reset GPIO...\n";
    adc_axiswitch_reset_gpio_ = std::make_unique<gpio::Gpio>(adc_axiswitchrst);
    
    if (!adc_axiswitch_reset_gpio_->enable()) {
        throw std::runtime_error("Unable to enable ADC AXI switch reset GPIO");
    }
    
    if (!adc_axiswitch_reset_gpio_->set_direction(gpio::Gpio::Direction::Output)) {
        throw std::runtime_error("Unable to set direction for ADC AXI switch reset GPIO");
    }
    
    if (!adc_axiswitch_reset_gpio_->set_value(gpio::Gpio::Value::High)) {
        throw std::runtime_error("Unable to set value for ADC AXI switch reset GPIO");
    }
    std::cout << "  ✓ ADC AXI switch reset GPIO initialized\n";
    
    std::cout << "  ✓ All GPIO pins configured successfully\n\n";
}

void RfDcApp::deinit_gpio() {
    std::cout << "\n━━━ Cleaning up GPIO Pins ━━━\n";
    
    // Clear all GPIO vectors - destructors will handle cleanup
    dac_userselect_gpios_.clear();
    dac_mts_clk_gpios_.clear();
    adc_mts_clk_gpios_.clear();
    adc_axiswitch_reset_gpio_.reset();
    
    std::cout << "  ✓ GPIO cleanup complete\n";
}

// ===== Clock Initialization =====

void RfDcApp::initialize_clocks() {
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
    std::cout << "━━━ Configuring DAC Tiles ━━━\n";
    
    constexpr double NCO_FREQ = 0.0;  // MHz
    
    for (uint32_t tile = 0; tile < 4; ++tile) {
        if (!rfdc_->check_tile_enabled(rfdc::TileType::DAC, tile)) {
            continue;
        }
        
        std::cout << format_msg("  Configuring DAC Tile ", tile, "...\n");
        
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
        }
        
        // **CRITICAL: Reprogram MMCM after configuration changes**
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
            
            // Configure mixer
            rfdc::MixerSettings mixer(
                NCO_FREQ,
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
            
            // Configure decimation
            rfdc_->set_decimation_factor(tile, block, 2);
            std::cout << "        - Decimation: 2x\n";
            
            // Configure QMC
            rfdc::QMCSettings qmc(
                false, false, 0.0, 0.0, 0,
                rfdc::EventSource::Tile
            );
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

// ===== Data Transfer Wrapper Functions =====

void RfDcApp::write_dac_samples(uint32_t tile, uint32_t block,
                                const std::vector<int16_t>& samples)
{
    uint32_t size_bytes = samples.size() * sizeof(int16_t);
    int ret = write_data_to_memory_bram(block, tile, size_bytes, samples);
    if (ret != SUCCESS) {
        throw std::runtime_error("Failed to write DAC samples");
    }
}

std::vector<int16_t> RfDcApp::read_adc_samples(uint32_t tile, uint32_t block,
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

// ===== Test Functions =====

void RfDcApp::run_loopback_test() {
    std::cout << "━━━ Running Loopback Test ━━━\n";
    std::cout << "  Testing: DAC Tile 0 Block 0 → ADC Tile 0 Block 0\n";
    std::cout << "  (Loopback cable connected)\n\n";
    
    try {
        const double sample_rate = 7.86432e9;  // 7.86432 GSPS
        const size_t num_samples = 16384;
        const double test_frequency = 25e6;  // 100 MHz test tone
        
        const uint32_t tile = 0;
        const uint32_t block = 0;
        const uint32_t channel_mask = 0x0001;  // Only block 0
        
        // ===== DAC PLAYBACK SEQUENCE (RFTool exact order) =====
        std::cout << "━━━ DAC Tile 0 Block 0 Setup ━━━\n";
        
        // Check PLL lock
        if (!rfdc_->get_pll_lock_status(rfdc::TileType::DAC, tile)) {
            throw std::runtime_error("DAC Tile 0 PLL not locked!");
        }
        std::cout << "  ✓ DAC PLL locked\n";
        
        // **STEP 1: RESET DAC Memory**
        std::cout << "\n  Step 1: Reset DAC memory\n";
        local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, 0x0000);
        
        // **STEP 2: SetLocalMemSample - Configure transfer size**
        std::cout << "  Step 2: Configure sample count (" << num_samples << " samples)\n";
        set_local_mem_sample(rfdc::TileType::DAC, tile, block, num_samples);
        
        // **STEP 3: Generate sine wave**
        std::cout << "  Step 3: Generate " << test_frequency/1e6 << " MHz sine wave\n";
        auto samples = generate_sine_wave(test_frequency, sample_rate, num_samples, 30000);
        
        // **STEP 4: Write to BRAM**
        std::cout << "  Step 4: Write samples to DAC BRAM\n";
        write_dac_samples(tile, block, samples);
        
        // Save for analysis
        save_samples_to_csv(samples, sample_rate, "dac_t0_b0_100MHz.csv");
        std::cout << "    ✓ Saved: dac_t0_b0_100MHz.csv\n";
        
        // **STEP 5: TRIGGER DAC PLAYBACK**
        std::cout << "  Step 5: **TRIGGER DAC playback**\n";
        local_mem_trigger(rfdc::TileType::DAC, tile, num_samples, channel_mask);
        
        // Wait for playback to stabilize
        std::cout << "    Waiting 200ms for DAC to stabilize...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "  ✓ DAC is now outputting RF signal\n\n";
        
        // ===== ADC CAPTURE SEQUENCE (RFTool exact order) =====
        std::cout << "━━━ ADC Tile 0 Block 0 Capture ━━━\n";
        
        // Check PLL lock
        if (!rfdc_->get_pll_lock_status(rfdc::TileType::ADC, tile)) {
            throw std::runtime_error("ADC Tile 0 PLL not locked!");
        }
        std::cout << "  ✓ ADC PLL locked\n";
        
        // **STEP 1: SetLocalMemSample for ADC**
        std::cout << "\n  Step 1: Configure ADC capture size (" << num_samples << " samples)\n";
        set_local_mem_sample(rfdc::TileType::ADC, tile, block, num_samples);
        
        // **STEP 2: TRIGGER ADC CAPTURE**
        std::cout << "  Step 2: **TRIGGER ADC capture**\n";
        local_mem_trigger(rfdc::TileType::ADC, tile, num_samples, channel_mask);
        
        // Wait for capture to complete
        std::cout << "    Waiting 200ms for capture to complete...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // **STEP 3: Read captured data**
        std::cout << "  Step 3: Reading captured data from ADC BRAM\n";
        auto captured = read_adc_samples(tile, block, num_samples);
        
        // Save to CSV
        save_samples_to_csv(captured, sample_rate, "adc_t0_b0_capture.csv");
        std::cout << "    ✓ Saved: adc_t0_b0_capture.csv\n";
        
        // ===== ANALYSIS =====
        std::cout << "\n━━━ Analysis ━━━\n";
        
        // Print first 32 samples
        std::cout << "  First 32 captured samples:\n    ";
        for (size_t i = 0; i < std::min(size_t(32), captured.size()); ++i) {
            std::cout << captured[i] << " ";
            if ((i + 1) % 16 == 0 && i < 31) std::cout << "\n    ";
        }
        std::cout << "\n\n";
        
        // Calculate statistics
        auto max_it = std::max_element(captured.begin(), captured.end());
        auto min_it = std::min_element(captured.begin(), captured.end());
        
        int16_t max_val = (max_it != captured.end()) ? *max_it : 0;
        int16_t min_val = (min_it != captured.end()) ? *min_it : 0;
        int16_t peak_to_peak = max_val - min_val;
        
        // Calculate RMS
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
        
        // Determine result
        std::cout << "\n━━━ Result ━━━\n";
        if (peak_to_peak > 1000) {
            std::cout << "  ✓✓✓ LOOPBACK SUCCESS! ✓✓✓\n";
            std::cout << "  Strong signal detected!\n";
            std::cout << "  Expected: ~60000 counts P-P for 30000 amplitude sine wave\n";
            std::cout << "  Measured: " << peak_to_peak << " counts P-P\n";
            std::cout << "\n  System is working correctly!\n";
        } else if (peak_to_peak > 100) {
            std::cout << "  ⚠ WEAK SIGNAL DETECTED\n";
            std::cout << "  Measured: " << peak_to_peak << " counts P-P\n";
            std::cout << "\n  Possible issues:\n";
            std::cout << "    - Check cable connection\n";
            std::cout << "    - Check RF attenuators/gain settings\n";
            std::cout << "    - Verify clock frequencies match\n";
        } else {
            std::cout << "  ✗ NO SIGNAL DETECTED\n";
            std::cout << "  Measured: " << peak_to_peak << " counts P-P (too low)\n";
            std::cout << "\n  Troubleshooting:\n";
            std::cout << "    1. Verify /dev/plmem* UIO devices exist:\n";
            std::cout << "       $ ls -l /dev/plmem*\n";
            std::cout << "    2. Check loopback cable is connected:\n";
            std::cout << "       DAC228_T0_CH0 → ADC228_T0_CH0\n";
            std::cout << "    3. Verify PLL locks shown above\n";
            std::cout << "    4. Check data in CSV files:\n";
            std::cout << "       - dac_t0_b0_100MHz.csv (should show sine wave)\n";
            std::cout << "       - adc_t0_b0_capture.csv (check for noise floor)\n";
            std::cout << "    5. Verify sysfs settings:\n";
            std::cout << "       $ cat /sys/class/plmem/plmem0/device/select_mem\n";
            std::cout << "       (should be 0x1)\n";
        }
        
    } catch (const std::exception& e) {
        std::cerr << "\n✗ Error: " << e.what() << "\n";
    }
}

void RfDcApp::save_samples_to_csv(const std::vector<int16_t>& samples, 
                                   double sample_rate_hz,
                                   const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "  ✗ Failed to open file: " << filename << "\n";
        return;
    }
    
    // Write CSV header
    file << "sample,value,time_ns\n";
    
    // Write data
    for (size_t i = 0; i < samples.size(); ++i) {
        double time_ns = (i / sample_rate_hz) * 1e9;
        file << i << "," 
             << samples[i] << ","
             << std::fixed << std::setprecision(3) << time_ns << "\n";
    }
    
    file.close();
}

// ===== Waveform Generation Functions =====

std::vector<int16_t> RfDcApp::generate_sine_wave(double frequency_hz,
                                                 double sample_rate_hz,
                                                 size_t num_samples,
                                                 int16_t amplitude)
{
    std::vector<int16_t> samples(num_samples);
    
    const double two_pi = 2.0 * M_PI;
    const double phase_increment = two_pi * frequency_hz / sample_rate_hz;
    
    for (size_t i = 0; i < num_samples; ++i) {
        double phase = phase_increment * i;
        double sample = amplitude * sin(phase);
        samples[i] = static_cast<int16_t>(sample);
    }
    
    std::cout << "  ✓ Generated " << num_samples << " sample sine wave: "
              << frequency_hz / 1e6 << " MHz @ "
              << sample_rate_hz / 1e9 << " GSPS\n";
    
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
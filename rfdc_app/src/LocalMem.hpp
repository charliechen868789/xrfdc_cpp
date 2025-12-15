#pragma once

#include <cstdint>
#include <memory>
#include "rfdc_wrapper/RfDc.hpp"

namespace local_mem {

/**
 * @brief Local Memory controller for RFDC data capture/generation
 * 
 * Manages the AXI Data Mover IP blocks that transfer data between
 * PL memory (BRAM) and the RFDC fabric interface.
 */
class LocalMem {
public:
    // Register offsets (from RFTool local_mem.h)
    static constexpr uint32_t LMEM_INFO = 0x00;
    static constexpr uint32_t LMEM_TRIGGER = 0x04;
    static constexpr uint32_t LMEM_ENABLE = 0x08;
    static constexpr uint32_t LMEM_ENABLE_TILE = 0x0C;
    static constexpr uint32_t LMEM0_ENDADDR = 0x10;
    
    // Memory type
    enum class MemType {
        BRAM = 0,
        DDR = 1
    };
    
    struct MemInfo {
        uint32_t num_tiles;
        uint32_t num_mem;
        uint32_t mem_size;
        int num_words;
        int mem_enable;
        int mem_clksel;
    };
    
    /**
     * @brief Construct LocalMem controller
     * @param rfdc Pointer to RFDC instance (for mixer/PLL queries)
     */
    explicit LocalMem(rfdc::RFDC* rfdc);
    
    /**
     * @brief Set the number of samples for a channel
     * @param type DAC or ADC
     * @param tile_id Tile ID
     * @param block_id Block ID
     * @param num_samples Number of samples to transfer
     * @param mem_base_addr Base address of memory region
     * @param channel_map Pointer to channel map array
     * @return true on success
     */
    bool set_sample_count(rfdc::TileType type, 
                         uint32_t tile_id,
                         uint32_t block_id,
                         uint32_t num_samples,
                         void* mem_base_addr,
                         const void* channel_map);
    
    /**
     * @brief Trigger data transfer for specified channels
     * @param type DAC or ADC
     * @param mem_base_addr Base address of memory region
     * @param channel_mask Bitmask of channels to trigger
     * @param channel_map Pointer to channel map array
     * @return true on success
     */
    bool trigger(rfdc::TileType type,
                void* mem_base_addr,
                uint32_t channel_mask,
                const void* channel_map);
    
    /**
     * @brief Get memory information from hardware
     * @param type DAC or ADC
     * @param mem_base_addr Base address of memory region
     * @return MemInfo structure
     */
    MemInfo get_mem_info(rfdc::TileType type, void* mem_base_addr);
    
private:
    rfdc::RFDC* rfdc_;
    
    // Helper to write 32-bit value to memory-mapped register
    void write_reg32(void* addr, uint32_t value);
    
    // Helper to read 32-bit value from memory-mapped register
    uint32_t read_reg32(void* addr);
};

} // namespace local_mem
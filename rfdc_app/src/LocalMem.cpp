#include "LocalMem.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>

namespace local_mem {

LocalMem::LocalMem(rfdc::RFDC* rfdc)
    : rfdc_(rfdc)
{
}

void LocalMem::write_reg32(void* addr, uint32_t value) {
    volatile uint32_t* reg = static_cast<volatile uint32_t*>(addr);
    *reg = value;
}

uint32_t LocalMem::read_reg32(void* addr) {
    volatile uint32_t* reg = static_cast<volatile uint32_t*>(addr);
    return *reg;
}

bool LocalMem::set_sample_count(rfdc::TileType type,
                                uint32_t tile_id,
                                uint32_t block_id,
                                uint32_t num_samples,
                                void* mem_base_addr,
                                const void* channel_map)
{
    if (!mem_base_addr || !channel_map) {
        std::cerr << "Invalid memory base address or channel map\n";
        return false;
    }
    
    // Cast channel_map to appropriate structure
    // Assuming channel map has Channel_I member at known offset
    struct ChannelMapEntry {
        uint32_t addr_I;
        uint32_t addr_Q;
        uint16_t Channel_I;
        uint16_t Channel_Q;
    };
    const ChannelMapEntry* map = static_cast<const ChannelMapEntry*>(channel_map);
    
    // Calculate numblockpertile based on type and ADC speed
    uint32_t numblockpertile = 4;
    if (type == rfdc::TileType::ADC) {
        // For high-speed ADC (4GSPS), only 2 blocks per tile
        numblockpertile = rfdc_->check_high_speed_adc(tile_id) ? 2 : 4;
    }
    
    uint32_t numsamples_channel = num_samples;
    
    // For ADC in IQ mode, double the sample count
    if (type == rfdc::TileType::ADC && !rfdc_->check_high_speed_adc(tile_id)) {
        auto mixer = rfdc_->get_mixer_settings(type, tile_id, block_id);
        
        // Check if in IQ mode (not R2R and not bypassed)
        bool is_r2r = (mixer.mode() == rfdc::MixerMode::R2R);
        bool is_bypass = (mixer.type() == rfdc::MixerType::Coarse &&
                         mixer.frequency() == XRFDC_COARSE_MIX_BYPASS);
        
        if (!is_r2r && !is_bypass) {
            numsamples_channel = num_samples * 2;
        }
    }
    
    // Validate sample count is multiple of 16
    if ((numsamples_channel % 16) != 0) {
        std::cerr << "Sample size must be multiple of 16 for Tile " 
                  << tile_id << " Block " << block_id << "\n";
        return false;
    }
    
    // Calculate register address
    uint32_t channel_index = (tile_id * numblockpertile) + block_id;
    uint32_t channel_i = map[channel_index].Channel_I;
    
    void* reg_addr = static_cast<char*>(mem_base_addr) + LMEM0_ENDADDR + (channel_i * 4);
    
    // Write sample count to hardware register
    write_reg32(reg_addr, numsamples_channel);
    
    std::cout << "  • Set sample count: " << numsamples_channel 
              << " for " << (type == rfdc::TileType::DAC ? "DAC" : "ADC")
              << "[" << tile_id << "][" << block_id << "]\n";
    
    // For high-speed ADC, also set Q channel
    if (type == rfdc::TileType::ADC && rfdc_->check_high_speed_adc(tile_id)) {
        uint32_t channel_q = map[channel_index].Channel_Q;
        void* reg_addr_q = static_cast<char*>(mem_base_addr) + LMEM0_ENDADDR + (channel_q * 4);
        write_reg32(reg_addr_q, numsamples_channel);
    }
    
    return true;
}

bool LocalMem::trigger(rfdc::TileType type,
                      void* mem_base_addr,
                      uint32_t channel_mask,
                      const void* channel_map)
{
    if (!mem_base_addr || !channel_map) {
        std::cerr << "Invalid memory base address or channel map\n";
        return false;
    }
    
    struct ChannelMapEntry {
        uint32_t addr_I;
        uint32_t addr_Q;
        uint16_t Channel_I;
        uint16_t Channel_Q;
    };
    const ChannelMapEntry* map = static_cast<const ChannelMapEntry*>(channel_map);
    
    // Disable all channels first
    write_reg32(static_cast<char*>(mem_base_addr) + LMEM_ENABLE_TILE, 0);
    write_reg32(static_cast<char*>(mem_base_addr) + LMEM_ENABLE, 0);
    
    if (channel_mask == 0) {
        // Reset/stop mode - just return after disabling
        std::cout << "  • Reset " << (type == rfdc::TileType::DAC ? "DAC" : "ADC") << " trigger\n";
        return true;
    }
    
    // Build memory enable mask
    uint32_t mem_ids = 0;
    uint32_t tile_id = 0;
    uint32_t blocks_in_tile = 0;
    uint32_t num_blocks = 4;
    
    for (uint32_t i = 0; i < 16; i++) {
        // Determine tile boundaries
        if (type == rfdc::TileType::ADC) {
            num_blocks = rfdc_->check_high_speed_adc(tile_id) ? 2 : 4;
        }
        
        if (blocks_in_tile == num_blocks) {
            tile_id++;
            blocks_in_tile = 0;
        }
        
        // Check if this channel is in the mask
        if ((channel_mask >> i) & 0x1) {
            // Enable I channel
            mem_ids |= (0x01 << map[i].Channel_I);
            
            // For high-speed ADC, also enable Q channel
            if (type == rfdc::TileType::ADC && rfdc_->check_high_speed_adc(tile_id)) {
                mem_ids |= (0x01 << map[i].Channel_Q);
            }
        }
        
        blocks_in_tile++;
    }
    
    // Enable selected memory channels
    write_reg32(static_cast<char*>(mem_base_addr) + LMEM_ENABLE, mem_ids);
    
    std::cout << "  • Memory enable mask: 0x" << std::hex << mem_ids << std::dec << "\n";
    
    // Issue the trigger
    write_reg32(static_cast<char*>(mem_base_addr) + LMEM_TRIGGER, 0x1);
    
    // Enable all tiles
    write_reg32(static_cast<char*>(mem_base_addr) + LMEM_ENABLE_TILE, 0xF);
    
    std::cout << "  • Triggered data mover\n";
    
    return true;
}

LocalMem::MemInfo LocalMem::get_mem_info(rfdc::TileType type, void* mem_base_addr)
{
    MemInfo info = {};
    
    if (!mem_base_addr) {
        return info;
    }
    
    // Read memory info register
    uint32_t data = read_reg32(static_cast<char*>(mem_base_addr) + LMEM_INFO);
    
    // Parse bit fields
    constexpr uint32_t MEM_SIZE_MASK = 0x00FFFFFF;
    constexpr uint32_t MEM_NUM_MASK = 0x7F;
    constexpr uint32_t MEM_NUM_SHIFT = 24;
    
    info.num_mem = (data >> MEM_NUM_SHIFT) & MEM_NUM_MASK;
    info.mem_size = (data & MEM_SIZE_MASK) / (info.num_mem + 1) / 16;
    
    // Get fabric words from RFDC
    uint32_t fab_words = 0;
    if (type == rfdc::TileType::ADC) {
        rfdc_->get_fab_rd_vld_words(type,0, 0, fab_words);
        info.num_tiles = 4;
    } else {
        rfdc_->get_fab_wr_vld_words(type,0, 0, fab_words);
        info.num_tiles = rfdc_->check_high_speed_adc(0) ? 2 : 4;
    }
    info.num_words = fab_words;
    
    // Read enable register
    data = read_reg32(static_cast<char*>(mem_base_addr) + LMEM_ENABLE);
    info.mem_enable = data;
    info.mem_clksel = 0;
    
    return info;
}

} // namespace local_mem
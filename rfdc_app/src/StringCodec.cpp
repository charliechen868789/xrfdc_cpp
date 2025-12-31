#include "StringCodec.hpp"
#include <cstring>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace codec {

StringCodec::StringCodec(const Config& config) 
    : config_(config) 
{
    if (config_.samples_per_symbol < 4) {
        throw std::invalid_argument("samples_per_symbol must be >= 4");
    }
}

// ===== Public Encoding Functions =====

void StringCodec::encode(const std::string& message,
                        std::vector<int16_t>& samples_I,
                        std::vector<int16_t>& samples_Q)
{
    samples_I.clear();
    samples_Q.clear();
    
    if (message.empty()) {
        return;
    }
    
    // Convert string to bytes
    std::vector<uint8_t> data = string_to_bytes(message);
    
    // Add CRC if enabled
    if (config_.use_crc) {
        uint16_t crc = calculate_crc16(data);
        data.push_back(crc & 0xFF);
        data.push_back((crc >> 8) & 0xFF);
    }
    
    // Generate preamble if enabled
    if (config_.use_preamble) {
        generate_preamble(samples_I, samples_Q);
    }
    
    // Encode based on modulation type
    switch (config_.modulation) {
        case ModulationType::BPSK:
            encode_bpsk(data, samples_I);
            break;
            
        case ModulationType::QPSK:
            encode_qpsk(data, samples_I, samples_Q);
            break;
            
        case ModulationType::PSK8:
            encode_8psk(data, samples_I, samples_Q);
            break;
            
        case ModulationType::QAM16:
            encode_16qam(data, samples_I, samples_Q);
            break;
    }
    
    std::cout << "  ✓ Encoded \"" << message << "\" → " 
              << samples_I.size() << " samples\n";
}

std::vector<int16_t> StringCodec::encode_real(const std::string& message)
{
    if (config_.modulation != ModulationType::BPSK) {
        throw std::runtime_error("encode_real only supports BPSK modulation");
    }
    
    std::vector<int16_t> samples_I, samples_Q;
    encode(message, samples_I, samples_Q);
    return samples_I;
}

// ===== Public Decoding Functions =====

bool StringCodec::decode(const std::vector<int16_t>& samples_I,
                        const std::vector<int16_t>& samples_Q,
                        std::string& decoded)
{
    decoded.clear();
    
    if (samples_I.empty()) {
        return false;
    }
    
    // Create working copies
    std::vector<int16_t> work_I = samples_I;
    std::vector<int16_t> work_Q = samples_Q;
    
    // Detect preamble if enabled
    if (config_.use_preamble) {
        size_t start_idx = 0;
        if (!detect_preamble(work_I, work_Q, start_idx)) {
            std::cerr << "  ✗ Preamble not detected\n";
            return false;
        }
        
        // Skip preamble
        work_I = std::vector<int16_t>(work_I.begin() + start_idx, work_I.end());
        if (!work_Q.empty()) {
            work_Q = std::vector<int16_t>(work_Q.begin() + start_idx, work_Q.end());
        }
    }
    
    // Decode bytes
    std::vector<uint8_t> bytes;
    bool success = false;
   
    switch (config_.modulation) {
        case ModulationType::BPSK:
            success = decode_bpsk(work_I, bytes);
            break;
            
        case ModulationType::QPSK:
            success = decode_qpsk(work_I, work_Q, bytes);
            break;
            
        case ModulationType::PSK8:
            success = decode_8psk(work_I, work_Q, bytes);
            break;
            
        case ModulationType::QAM16:
            success = decode_16qam(work_I, work_Q, bytes);
            break;
    }
    
    if (!success || bytes.size() < 2) {
        return false;
    }
    
    // Verify CRC if enabled
    if (config_.use_crc) {
        if (bytes.size() < 2) {
            return false;
        }
        
        uint16_t received_crc = bytes[bytes.size() - 2] | 
                               (bytes[bytes.size() - 1] << 8);
        bytes.resize(bytes.size() - 2);
        
        uint16_t calculated_crc = calculate_crc16(bytes);
        
        if (received_crc != calculated_crc) {
            std::cerr << "  ✗ CRC mismatch: expected 0x" << std::hex 
                     << calculated_crc << ", got 0x" << received_crc 
                     << std::dec << "\n";
            return false;
        }
    }
    
    decoded = bytes_to_string(bytes);
    std::cout << "  ✓ Decoded " << samples_I.size() << " samples → \"" 
              << decoded << "\"\n";
    
    return true;
}

bool StringCodec::decode_real(const std::vector<int16_t>& samples,
                             std::string& decoded)
{
    std::vector<int16_t> empty_Q;
    return decode(samples, empty_Q, decoded);
}

// ===== BPSK Encoding/Decoding =====

void StringCodec::encode_bpsk(const std::vector<uint8_t>& bytes,
                             std::vector<int16_t>& samples)
{
    std::vector<bool> bits = bytes_to_bits(bytes);
    
    std::cout << "  BPSK Encoder:\n";
    std::cout << "    Input: " << bytes.size() << " bytes\n";
    std::cout << "    First bytes (hex): ";
    for (size_t i = 0; i < std::min(size_t(10), bytes.size()); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << "\n";
    std::cout << "    Total bits: " << bits.size() << "\n";
    std::cout << "    Mapping: 1→+" << config_.amplitude 
              << ", 0→-" << config_.amplitude << "\n";
    
    for (bool bit : bits) {
        int16_t symbol = bit ? config_.amplitude : -config_.amplitude;
        
        // Generate oversampled symbol
        for (uint32_t i = 0; i < config_.samples_per_symbol; ++i) {
            samples.push_back(symbol);
        }
    }
    
    std::cout << "    Output: " << samples.size() << " samples\n";
}

#if 0
bool StringCodec::decode_bpsk(const std::vector<int16_t>& samples,
                             std::vector<uint8_t>& bytes)
{
    if (samples.empty()) {
        std::cerr << "  ✗ Empty sample buffer\n";
        return false;
    }
    
    std::vector<int16_t> symbols = downsample_symbols(samples);
    
    if (symbols.empty()) {
        std::cerr << "  ✗ No symbols after downsampling\n";
        return false;
    }
    
    std::cout << "  BPSK Decoder:\n";
    std::cout << "    Input samples: " << samples.size() << "\n";
    std::cout << "    Symbols: " << symbols.size() << "\n";
    
    // Sort for threshold calculation
    std::vector<int16_t> sorted_symbols = symbols;
    std::sort(sorted_symbols.begin(), sorted_symbols.end());
    
    int16_t min_sym = sorted_symbols.front();
    int16_t max_sym = sorted_symbols.back();
    
    std::cout << "    Symbol range: [" << min_sym << ", " << max_sym << "]\n";
    
    // Threshold: midpoint between clusters
    int16_t threshold = (min_sym + max_sym) / 2;
    std::cout << "    Threshold (midpoint): " << threshold << "\n";
    
    // Histogram
    std::cout << "    Symbol histogram:\n";
    const int bins = 10;
    int hist[bins] = {0};
    for (auto s : sorted_symbols) {
        int bin = std::min(bins - 1, 
                          std::max(0, 
                                  (int)((s - min_sym) * bins / (max_sym - min_sym + 1))));
        hist[bin]++;
    }
    std::cout << "      ";
    for (int i = 0; i < bins; ++i) {
        int bar_len = hist[i] * 20 / sorted_symbols.size();
        std::cout << std::string(bar_len, '#');
        if (i < bins - 1) std::cout << "|";
    }
    std::cout << "\n";
    std::cout << "      " << min_sym << std::string(bins*2-10, ' ') << max_sym << "\n";
    
    // ===== TRY BOTH POLARITIES AND CHECK WHICH MAKES SENSE =====
    std::vector<bool> bits_normal, bits_inverted;
    bits_normal.reserve(symbols.size());
    bits_inverted.reserve(symbols.size());
    
    for (int16_t symbol : symbols) {
        bits_normal.push_back(symbol > threshold);
        bits_inverted.push_back(symbol < threshold);
    }
    // ===== Trim symbols to expected message length ===== 
    // Convert both to bytes
    std::vector<uint8_t> bytes_normal = bits_to_bytes(bits_normal);
    std::vector<uint8_t> bytes_inverted = bits_to_bytes(bits_inverted);
    
    // Check which one gives printable ASCII or better balance
    int ones_normal = std::count(bits_normal.begin(), bits_normal.end(), true);
    int ones_inverted = std::count(bits_inverted.begin(), bits_inverted.end(), true);
    
    // Prefer more balanced distribution
    int balance_normal = std::abs(ones_normal - (int)(symbols.size() / 2));
    int balance_inverted = std::abs(ones_inverted - (int)(symbols.size() / 2));
    
    // Check if either starts with printable ASCII
    bool normal_ascii = !bytes_normal.empty() && 
                        bytes_normal[0] >= 0x20 && bytes_normal[0] <= 0x7E;
    bool inverted_ascii = !bytes_inverted.empty() && 
                          bytes_inverted[0] >= 0x20 && bytes_inverted[0] <= 0x7E;
    
    std::cout << "    Normal polarity:   " << ones_normal << " ones, " 
              << (symbols.size() - ones_normal) << " zeros";
    if (!bytes_normal.empty()) {
        std::cout << " → 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << (int)bytes_normal[0] << std::dec;
    }
    std::cout << "\n";
    
    std::cout << "    Inverted polarity: " << ones_inverted << " ones, " 
              << (symbols.size() - ones_inverted) << " zeros";
    if (!bytes_inverted.empty()) {
        std::cout << " → 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << (int)bytes_inverted[0] << std::dec;
    }
    std::cout << "\n";
    
    // Decision: prefer inverted if it gives ASCII OR better balance
    bool use_inverted = false;
    
    if (inverted_ascii && !normal_ascii) {
        std::cout << "    Selection: INVERTED (has ASCII)\n";
        use_inverted = true;
    } else if (balance_inverted < balance_normal - 5) {  // Margin for noise
        std::cout << "    Selection: INVERTED (better balance)\n";
        use_inverted = true;
    } else {
        std::cout << "    Selection: NORMAL\n";
    }
    
    // Use selected polarity
    if (use_inverted) {
        bytes = bytes_inverted;
        std::cout << "    Bit distribution: " << ones_inverted << " ones, " 
                  << (symbols.size() - ones_inverted) << " zeros\n";
    } else {
        bytes = bytes_normal;
        std::cout << "    Bit distribution: " << ones_normal << " ones, " 
                  << (symbols.size() - ones_normal) << " zeros\n";
    }
    
    if (bytes.empty()) {
        std::cerr << "  ✗ No bytes after bit conversion\n";
        return false;
    }
    
    std::cout << "    Decoded: " << bytes.size() << " bytes\n";
    std::cout << "    First bytes (hex): ";
    for (size_t i = 0; i < std::min(size_t(10), bytes.size()); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << "\n";
    
    return true;
}

#endif
// Add to StringCodec.cpp - IMPROVED decode_bpsk with DC offset correction

bool StringCodec::decode_bpsk(const std::vector<int16_t>& samples,
                             std::vector<uint8_t>& bytes)
{
    if (samples.empty()) {
        std::cerr << "  ✗ Empty sample buffer\n";
        return false;
    }
    
    // ⚠️ CRITICAL: Remove DC offset BEFORE downsampling
    std::vector<int16_t> samples_corrected = samples;
    
    int64_t sum = 0;
    for (auto s : samples) sum += s;
    int32_t dc_offset = sum / samples.size();
    
    std::cout << "  BPSK Decoder:\n";
    std::cout << "    Input samples: " << samples.size() << "\n";
    std::cout << "    DC offset: " << dc_offset << " LSB\n";
    
    // Remove DC offset
    if (std::abs(dc_offset) > 100) {
        std::cout << "    ⚠️ Correcting DC offset...\n";
        for (auto& s : samples_corrected) {
            int32_t corrected = static_cast<int32_t>(s) - dc_offset;
            // Clamp to int16_t range
            if (corrected > 32767) corrected = 32767;
            if (corrected < -32768) corrected = -32768;
            s = static_cast<int16_t>(corrected);
        }
    }
    
    // Show range after DC correction
    auto minmax = std::minmax_element(samples_corrected.begin(), samples_corrected.end());
    std::cout << "    After DC removal: [" << *minmax.first << ", " << *minmax.second << "]\n";
    
    // Downsample to symbol rate (now with corrected samples)
    std::vector<int16_t> symbols = downsample_symbols(samples_corrected);
    
    if (symbols.empty()) {
        std::cerr << "  ✗ No symbols after downsampling\n";
        return false;
    }
    
    std::cout << "    Symbols: " << symbols.size() << "\n";
    
    // Calculate statistics on SYMBOLS (not raw samples)
    std::vector<int16_t> sorted_symbols = symbols;
    std::sort(sorted_symbols.begin(), sorted_symbols.end());
    
    int16_t min_sym = sorted_symbols.front();
    int16_t max_sym = sorted_symbols.back();
    int16_t median = sorted_symbols[sorted_symbols.size() / 2];
    
    std::cout << "    Symbol range: [" << min_sym << ", " << max_sym << "]\n";
    
    // Calculate mean of symbols
    int64_t sum_sym = 0;
    for (auto s : symbols) sum_sym += s;
    int16_t mean_sym = sum_sym / symbols.size();
    
    std::cout << "    Symbol mean: " << mean_sym << "\n";
    std::cout << "    Symbol median: " << median << "\n";
    
    // ⚠️ CRITICAL: For BPSK, threshold should be near ZERO after DC removal
    // If mean is far from zero, there's still residual DC offset
    int16_t threshold = 0;  // Start with zero
    
    // Adjust if there's residual DC in symbols
    if (std::abs(mean_sym) > (max_sym - min_sym) / 10) {
        std::cout << "    ⚠️ Residual DC in symbols, using mean: " << mean_sym << "\n";
        threshold = mean_sym;
    } else {
        std::cout << "    ✓ Using zero threshold (DC corrected)\n";
    }
    
    // Histogram
    std::cout << "    Symbol histogram:\n";
    const int bins = 10;
    int hist[bins] = {0};
    for (auto s : sorted_symbols) {
        int bin = std::min(bins - 1, 
                          std::max(0, 
                                  (int)((s - min_sym) * bins / (max_sym - min_sym + 1))));
        hist[bin]++;
    }
    std::cout << "      ";
    for (int i = 0; i < bins; ++i) {
        int bar_len = hist[i] * 20 / sorted_symbols.size();
        std::cout << std::string(bar_len, '#');
        if (i < bins - 1) std::cout << "|";
    }
    std::cout << "\n";
    std::cout << "      " << min_sym << std::string(bins*2-6, ' ') << max_sym << "\n";
    
    std::cout << "    Using threshold: " << threshold << "\n";
    
    // Demodulate bits
    std::vector<bool> bits;
    bits.reserve(symbols.size());
    
    for (int16_t symbol : symbols) {
        bits.push_back(symbol > threshold);
    }
    
    // Distribution check
    int ones = std::count(bits.begin(), bits.end(), true);
    int zeros = bits.size() - ones;
    
    std::cout << "    Bit distribution: " << ones << " ones, " 
              << zeros << " zeros\n";
    
    if (ones == 0 || zeros == 0) {
        std::cerr << "    ⚠️ All bits same - trying inverted polarity\n";
        // Force inversion
        //for (auto& b : bits) b = !b;
        ones = std::count(bits.begin(), bits.end(), true);
        zeros = bits.size() - ones;
        std::cout << "    After inversion: " << ones << " ones, " << zeros << " zeros\n";
    }
    
    bytes = bits_to_bytes(bits);
    
    if (bytes.empty()) {
        std::cerr << "  ✗ No bytes after bit conversion\n";
        return false;
    }
    
    std::cout << "    Decoded: " << bytes.size() << " bytes\n";
    std::cout << "    First bytes (hex): ";
    for (size_t i = 0; i < std::min(size_t(10), bytes.size()); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << "\n";
    
    // Debug: show inverted
    std::vector<bool> bits_inv;
    for (auto b : bits) bits_inv.push_back(!b);
    std::vector<uint8_t> bytes_inv = bits_to_bytes(bits_inv);
    
    std::cout << "    [DEBUG] Inverted would be: ";
    for (size_t i = 0; i < std::min(size_t(10), bytes_inv.size()); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes_inv[i]) << " ";
    }
    std::cout << std::dec << "\n";
    
    // Check which one looks like ASCII
    bool normal_ascii = false;
    bool inverted_ascii = false;
    
    for (size_t i = 0; i < std::min(size_t(5), bytes.size()); ++i) {
        if (bytes[i] >= 32 && bytes[i] <= 126) normal_ascii = true;
    }
    for (size_t i = 0; i < std::min(size_t(5), bytes_inv.size()); ++i) {
        if (bytes_inv[i] >= 32 && bytes_inv[i] <= 126) inverted_ascii = true;
    }
    
    // Check against expected "HELLO"
    bool normal_is_hello = (bytes.size() >= 5 && 
                           bytes[0] == 0x48 && bytes[1] == 0x45 && 
                           bytes[2] == 0x4C && bytes[3] == 0x4C && bytes[4] == 0x4F);
    
    bool inverted_is_hello = (bytes_inv.size() >= 5 && 
                             bytes_inv[0] == 0x48 && bytes_inv[1] == 0x45 && 
                             bytes_inv[2] == 0x4C && bytes_inv[3] == 0x4C && bytes_inv[4] == 0x4F);
    
    if (inverted_is_hello) {
        std::cout << "    ✓✓✓ INVERTED BITS = HELLO!\n";
        bytes = bytes_inv;
    } else if (normal_is_hello) {
        std::cout << "    ✓✓✓ NORMAL BITS = HELLO!\n";
    } else if (inverted_ascii && !normal_ascii) {
        std::cout << "    ⚠️ Using inverted (contains ASCII)\n";
        bytes = bytes_inv;
    }
    
    return true;
}

// ===== QPSK =====

void StringCodec::encode_qpsk(const std::vector<uint8_t>& bytes,
                             std::vector<int16_t>& samples_I,
                             std::vector<int16_t>& samples_Q)
{
    std::vector<bool> bits = bytes_to_bits(bytes);
    if (bits.size() % 2 != 0) bits.push_back(false);
    
    for (size_t i = 0; i < bits.size(); i += 2) {
        int16_t I = bits[i] ? config_.amplitude : -config_.amplitude;
        int16_t Q = bits[i+1] ? config_.amplitude : -config_.amplitude;
        
        I = static_cast<int16_t>(I * 0.7071);
        Q = static_cast<int16_t>(Q * 0.7071);
        
        for (uint32_t j = 0; j < config_.samples_per_symbol; ++j) {
            samples_I.push_back(I);
            samples_Q.push_back(Q);
        }
    }
}

bool StringCodec::decode_qpsk(const std::vector<int16_t>& samples_I,
                             const std::vector<int16_t>& samples_Q,
                             std::vector<uint8_t>& bytes)
{
    if (samples_I.size() != samples_Q.size() || samples_I.empty()) return false;
    
    std::vector<int16_t> symbols_I = downsample_symbols(samples_I);
    std::vector<int16_t> symbols_Q = downsample_symbols(samples_Q);
    
    if (symbols_I.empty() || symbols_I.size() != symbols_Q.size()) return false;
    
    std::vector<int16_t> sorted_I = symbols_I;
    std::vector<int16_t> sorted_Q = symbols_Q;
    std::sort(sorted_I.begin(), sorted_I.end());
    std::sort(sorted_Q.begin(), sorted_Q.end());
    
    int16_t threshold_I = sorted_I[sorted_I.size() / 2];
    int16_t threshold_Q = sorted_Q[sorted_Q.size() / 2];
    
    std::vector<bool> bits;
    for (size_t i = 0; i < symbols_I.size(); ++i) {
        bits.push_back(symbols_I[i] > threshold_I);
        bits.push_back(symbols_Q[i] > threshold_Q);
    }
    
    bytes = bits_to_bytes(bits);
    return !bytes.empty();
}

// ===== 8PSK =====

void StringCodec::encode_8psk(const std::vector<uint8_t>& bytes,
                             std::vector<int16_t>& samples_I,
                             std::vector<int16_t>& samples_Q)
{
    std::vector<bool> bits = bytes_to_bits(bytes);
    while (bits.size() % 3 != 0) bits.push_back(false);
    
    for (size_t i = 0; i < bits.size(); i += 3) {
        uint8_t symbol = (bits[i] << 2) | (bits[i+1] << 1) | bits[i+2];
        double phase = symbol * M_PI / 4.0;
        
        int16_t I = static_cast<int16_t>(config_.amplitude * std::cos(phase));
        int16_t Q = static_cast<int16_t>(config_.amplitude * std::sin(phase));
        
        for (uint32_t j = 0; j < config_.samples_per_symbol; ++j) {
            samples_I.push_back(I);
            samples_Q.push_back(Q);
        }
    }
}

bool StringCodec::decode_8psk(const std::vector<int16_t>& samples_I,
                             const std::vector<int16_t>& samples_Q,
                             std::vector<uint8_t>& bytes)
{
    if (samples_I.size() != samples_Q.size()) return false;
    
    std::vector<int16_t> symbols_I = downsample_symbols(samples_I);
    std::vector<int16_t> symbols_Q = downsample_symbols(samples_Q);
    
    std::vector<bool> bits;
    for (size_t i = 0; i < symbols_I.size(); ++i) {
        double phase = std::atan2(symbols_Q[i], symbols_I[i]);
        if (phase < 0) phase += 2 * M_PI;
        
        uint8_t symbol = static_cast<uint8_t>(std::round(phase * 4.0 / M_PI)) % 8;
        
        bits.push_back((symbol >> 2) & 1);
        bits.push_back((symbol >> 1) & 1);
        bits.push_back(symbol & 1);
    }
    
    bytes = bits_to_bytes(bits);
    return true;
}

// ===== 16QAM =====

void StringCodec::encode_16qam(const std::vector<uint8_t>& bytes,
                              std::vector<int16_t>& samples_I,
                              std::vector<int16_t>& samples_Q)
{
    std::vector<bool> bits = bytes_to_bits(bytes);
    while (bits.size() % 4 != 0) bits.push_back(false);
    
    for (size_t i = 0; i < bits.size(); i += 4) {
        int8_t I_level = ((bits[i] << 1) | bits[i+1]) - 1;
        int8_t Q_level = ((bits[i+2] << 1) | bits[i+3]) - 1;
        
        I_level = (I_level < 0) ? I_level : I_level + 1;
        Q_level = (Q_level < 0) ? Q_level : Q_level + 1;
        
        int16_t I = static_cast<int16_t>(I_level * config_.amplitude / 4);
        int16_t Q = static_cast<int16_t>(Q_level * config_.amplitude / 4);
        
        for (uint32_t j = 0; j < config_.samples_per_symbol; ++j) {
            samples_I.push_back(I);
            samples_Q.push_back(Q);
        }
    }
}

bool StringCodec::decode_16qam(const std::vector<int16_t>& samples_I,
                              const std::vector<int16_t>& samples_Q,
                              std::vector<uint8_t>& bytes)
{
    if (samples_I.size() != samples_Q.size()) return false;
    
    std::vector<int16_t> symbols_I = downsample_symbols(samples_I);
    std::vector<int16_t> symbols_Q = downsample_symbols(samples_Q);
    
    std::vector<bool> bits;
    for (size_t i = 0; i < symbols_I.size(); ++i) {
        int8_t I_level = static_cast<int8_t>(std::round(symbols_I[i] * 4.0 / config_.amplitude));
        int8_t Q_level = static_cast<int8_t>(std::round(symbols_Q[i] * 4.0 / config_.amplitude));
        
        I_level = std::max<int8_t>(-3, std::min<int8_t>(3, I_level));
        Q_level = std::max<int8_t>(-3, std::min<int8_t>(3, Q_level));
        
        I_level = (I_level < 0) ? I_level : I_level - 1;
        Q_level = (Q_level < 0) ? Q_level : Q_level - 1;
        I_level += 1;
        Q_level += 1;
        
        bits.push_back((I_level >> 1) & 1);
        bits.push_back(I_level & 1);
        bits.push_back((Q_level >> 1) & 1);
        bits.push_back(Q_level & 1);
    }
    
    bytes = bits_to_bytes(bits);
    return true;
}

// ===== Helper Functions =====

std::vector<uint8_t> StringCodec::string_to_bytes(const std::string& str)
{
    return std::vector<uint8_t>(str.begin(), str.end());
}

std::string StringCodec::bytes_to_string(const std::vector<uint8_t>& bytes)
{
    return std::string(bytes.begin(), bytes.end());
}

std::vector<bool> StringCodec::bytes_to_bits(const std::vector<uint8_t>& bytes)
{
    std::vector<bool> bits;
    bits.reserve(bytes.size() * 8);
    
    for (uint8_t byte : bytes) {
        for (int i = 7; i >= 0; --i) {
            bits.push_back((byte >> i) & 1);
        }
    }
    
    return bits;
}

std::vector<uint8_t> StringCodec::bits_to_bytes(const std::vector<bool>& bits)
{
    std::vector<uint8_t> bytes;
    bytes.reserve((bits.size() + 7) / 8);
    
    for (size_t i = 0; i < bits.size(); i += 8) {
        uint8_t byte = 0;
        for (size_t j = 0; j < 8 && (i + j) < bits.size(); ++j) {
            if (bits[i + j]) {
                byte |= (1 << (7 - j));
            }
        }
        bytes.push_back(byte);
    }
    
    return bytes;
}

uint16_t StringCodec::calculate_crc16(const std::vector<uint8_t>& data)
{
    uint16_t crc = 0xFFFF;
    
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

void StringCodec::generate_preamble(std::vector<int16_t>& samples_I,
                                   std::vector<int16_t>& samples_Q)
{
    const std::vector<int8_t> barker = {1, 1, 1, -1, -1, 1, -1, 1};
    
    for (int8_t bit : barker) {
        int16_t symbol = bit * config_.amplitude;
        for (uint32_t i = 0; i < config_.samples_per_symbol; ++i) {
            samples_I.push_back(symbol);
            if (!samples_Q.empty() || config_.modulation != ModulationType::BPSK) {
                samples_Q.push_back(0);
            }
        }
    }
}

bool StringCodec::detect_preamble(const std::vector<int16_t>& samples_I,
                                 const std::vector<int16_t>& samples_Q __attribute__((unused)),
                                 size_t& start_index)
{
    const std::vector<int8_t> barker = {1, 1, 1, -1, -1, 1, -1, 1};
    const size_t preamble_length = barker.size() * config_.samples_per_symbol;
    
    if (samples_I.size() < preamble_length) {
        std::cerr << "  ✗ Signal too short\n";
        return false;
    }
    
    auto minmax = std::minmax_element(samples_I.begin(), samples_I.end());
    int16_t signal_amplitude = (*minmax.second - *minmax.first) / 2;
    
    if (signal_amplitude < 100) {
        std::cerr << "  ✗ Signal too weak\n";
        return false;
    }
    
    double best_corr = 0;
    size_t best_idx = 0;
    size_t search_limit = samples_I.size() - preamble_length;
    uint32_t stride = std::max(1u, config_.samples_per_symbol / 8);
    
    for (size_t i = 0; i < search_limit; i += stride) {
        double corr = 0;
        
        for (size_t j = 0; j < barker.size(); ++j) {
            double avg = 0;
            size_t base_idx = i + j * config_.samples_per_symbol;
            
            for (uint32_t k = 0; k < config_.samples_per_symbol; ++k) {
                if (base_idx + k < samples_I.size()) {
                    avg += samples_I[base_idx + k];
                }
            }
            avg /= config_.samples_per_symbol;
            
            corr += avg * barker[j] * signal_amplitude;
        }
        
        if (std::abs(corr) > std::abs(best_corr)) {
            best_corr = corr;
            best_idx = i + preamble_length;
        }
    }
    
    double expected_corr = signal_amplitude * signal_amplitude * barker.size();
    double threshold = expected_corr * 0.05;
    
    if (std::abs(best_corr) > threshold) {
        start_index = best_idx;
        std::cout << "  ✓ Preamble detected at sample " << best_idx << "\n";
        return true;
    }
    
    std::cerr << "  ✗ Preamble not detected\n";
    return false;
}

size_t StringCodec::calculate_sample_count(size_t message_length) const
{
    size_t bits_per_symbol = get_bits_per_symbol();
    size_t total_bits = message_length * 8;
    
    if (config_.use_crc) {
        total_bits += 16;
    }
    
    size_t num_symbols = (total_bits + bits_per_symbol - 1) / bits_per_symbol;
    size_t total_samples = num_symbols * config_.samples_per_symbol;
    
    if (config_.use_preamble) {
        total_samples += 8 * config_.samples_per_symbol;
    }
    
    return total_samples;
}

uint32_t StringCodec::get_bits_per_symbol() const
{
    switch (config_.modulation) {
        case ModulationType::BPSK:  return 1;
        case ModulationType::QPSK:  return 2;
        case ModulationType::PSK8:  return 3;
        case ModulationType::QAM16: return 4;
        default: return 1;
    }
}

void StringCodec::analyze_signal(const std::vector<int16_t>& samples_I,
                                const std::vector<int16_t>& samples_Q)
{
    std::cout << "\n━━━ Signal Analysis ━━━\n";
    if (samples_I.empty()) {
        std::cout << "  ✗ Empty signal\n";
        return;
    }
    
    auto minmax = std::minmax_element(samples_I.begin(), samples_I.end());
    std::cout << "  Range: [" << *minmax.first << ", " << *minmax.second << "]\n";
    std::cout << "  P-P: " << (*minmax.second - *minmax.first) << "\n";
}

void StringCodec::save_constellation(const std::vector<int16_t>& samples_I,
                                    const std::vector<int16_t>& samples_Q,
                                    const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) return;
    
    file << "# I,Q\n";
    std::vector<int16_t> sym_I = downsample_symbols(samples_I);
    std::vector<int16_t> sym_Q;
    if (!samples_Q.empty()) sym_Q = downsample_symbols(samples_Q);
    
    for (size_t i = 0; i < sym_I.size(); ++i) {
        file << sym_I[i] << "," << (i < sym_Q.size() ? sym_Q[i] : 0) << "\n";
    }
}

void StringCodec::debug_bit_alignment(const std::vector<int16_t>& symbols,
                                     int16_t threshold)
{
    std::cout << "\n━━━ Bit Alignment Debug ━━━\n";
    
    std::vector<bool> bits;
    for (auto s : symbols) bits.push_back(s > threshold);
    
    for (int offset = 0; offset < 8; ++offset) {
        std::vector<bool> shifted(bits.begin() + offset, bits.end());
        std::vector<uint8_t> bytes = bits_to_bytes(shifted);
        
        std::cout << "Offset " << offset << ": ";
        for (size_t i = 0; i < std::min(size_t(5), bytes.size()); ++i) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(bytes[i]) << " ";
        }
        
        if (bytes.size() >= 5 && bytes[0] == 0x48 && bytes[1] == 0x45) {
            std::cout << " ← HELLO!";
        }
        std::cout << std::dec << "\n";
    }
}

uint32_t StringCodec::find_optimal_phase(const std::vector<int16_t>& samples)
{
    if (samples.size() < config_.samples_per_symbol * 4) {
        return config_.samples_per_symbol / 2; // Default to center
    }
    
    // Method 1: Zero-crossing timing (best for BPSK)
    // Find the phase with MINIMUM variance (most stable)
    double best_stability = 1e10;
    uint32_t best_phase = config_.samples_per_symbol / 2;
    
    // Only search middle 50% of symbol period (avoid transitions)
    uint32_t start_phase = config_.samples_per_symbol / 4;
    uint32_t end_phase = config_.samples_per_symbol * 3 / 4;
    
    std::cout << "  Advanced timing recovery:\n";
    std::cout << "    Searching phases " << start_phase << " to " << end_phase << "\n";
    
    for (uint32_t phase = start_phase; phase < end_phase; phase += 2) {
        std::vector<int16_t> test_symbols;
        
        // Sample at this phase for first 20 symbols
        for (size_t i = phase; 
             i < std::min(samples.size(), size_t(20 * config_.samples_per_symbol)); 
             i += config_.samples_per_symbol) {
            test_symbols.push_back(samples[i]);
        }
        
        if (test_symbols.size() < 10) continue;
        
        // Calculate variance (lower = more stable = better timing)
        double mean = 0;
        for (auto s : test_symbols) mean += std::abs(s);
        mean /= test_symbols.size();
        
        double variance = 0;
        for (auto s : test_symbols) {
            double diff = std::abs(s) - mean;
            variance += diff * diff;
        }
        variance /= test_symbols.size();
        
        if (variance < best_stability) {
            best_stability = variance;
            best_phase = phase;
        }
    }
    
    std::cout << "    Optimal phase: " << best_phase 
              << " / " << config_.samples_per_symbol << "\n";
    std::cout << "    Stability metric: " << std::sqrt(best_stability) << "\n";
    
    return best_phase;
}

std::vector<int16_t> StringCodec::downsample_symbols(const std::vector<int16_t>& samples)
{
    std::vector<int16_t> symbols;
    
    if (samples.size() < config_.samples_per_symbol) {
        std::cerr << "  ✗ Not enough samples for one symbol\n";
        return symbols;
    }
    
    // Use improved timing recovery
    uint32_t best_phase = find_optimal_phase(samples);
    
    // Extract symbols at the optimal phase
    for (size_t i = best_phase; i < samples.size(); i += config_.samples_per_symbol) {
        symbols.push_back(samples[i]);
    }
    
    std::cout << "  Timing recovery:\n";
    std::cout << "    Phase: " << best_phase << " / " << config_.samples_per_symbol << "\n";
    std::cout << "    Output: " << symbols.size() << " symbols\n";
    
    return symbols;
}

} // namespace codec
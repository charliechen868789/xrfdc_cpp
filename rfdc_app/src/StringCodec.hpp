#ifndef STRINGCODEC_HPP
#define STRINGCODEC_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <random>

namespace codec {

/**
 * @brief Digital modulation codec for string transmission
 * 
 * Supports BPSK, QPSK, 8PSK, and 16QAM modulation schemes
 * with optional preamble and CRC16 error detection.
 */
class StringCodec {
public:
    /**
     * @brief Supported modulation types
     */
    enum class ModulationType {
        BPSK,   ///< Binary Phase Shift Keying (1 bit/symbol)
        QPSK,   ///< Quadrature Phase Shift Keying (2 bits/symbol)
        PSK8,   ///< 8-Phase Shift Keying (3 bits/symbol)
        QAM16   ///< 16-Quadrature Amplitude Modulation (4 bits/symbol)
    };
    
    /**
     * @brief Codec configuration
     */
    struct Config {
        ModulationType modulation;
        uint32_t samples_per_symbol;    ///< Oversampling rate
        int16_t amplitude;               ///< Signal amplitude (±)
        bool use_crc;                    ///< Enable CRC16 error detection
        bool use_preamble;               ///< Enable preamble for synchronization
        
        // Constructor with defaults (C++14 compatible)
        Config() 
            : modulation(ModulationType::BPSK)
            , samples_per_symbol(8)
            , amplitude(10000)
            , use_crc(true)
            , use_preamble(true)
        {}
    };
    
    /**
     * @brief Construct codec with configuration
     * @param config Codec configuration
     */
    explicit StringCodec(const Config& config);
    
    // ===== Encoding Functions =====
    
    /**
     * @brief Encode string to I/Q samples
     * @param message Input string to encode
     * @param samples_I Output I channel samples
     * @param samples_Q Output Q channel samples (empty for BPSK)
     */
    void encode(const std::string& message,
                std::vector<int16_t>& samples_I,
                std::vector<int16_t>& samples_Q);
    
    /**
     * @brief Encode string to real samples (BPSK only)
     * @param message Input string to encode
     * @return Real-valued samples
     * @throws std::runtime_error if modulation is not BPSK
     */
    std::vector<int16_t> encode_real(const std::string& message);
    
    // ===== Decoding Functions =====
    
    /**
     * @brief Decode I/Q samples to string
     * @param samples_I I channel samples
     * @param samples_Q Q channel samples (empty for BPSK)
     * @param decoded Output decoded string
     * @return true if decode successful, false otherwise
     */
    bool decode(const std::vector<int16_t>& samples_I,
                const std::vector<int16_t>& samples_Q,
                std::string& decoded);
    
    /**
     * @brief Decode real samples to string (BPSK only)
     * @param samples Input real samples
     * @param decoded Output decoded string
     * @return true if decode successful, false otherwise
     */
    bool decode_real(const std::vector<int16_t>& samples,
                     std::string& decoded);
    
    // ===== Utility Functions =====
    
    /**
     * @brief Calculate number of samples needed for message
     * @param message_length Length of message in bytes
     * @return Total samples required
     */
    size_t calculate_sample_count(size_t message_length) const;
    
    /**
     * @brief Get bits per symbol for current modulation
     * @return Bits per symbol
     */
    uint32_t get_bits_per_symbol() const;
    
    /**
     * @brief Analyze received signal quality
     * @param samples_I I channel samples
     * @param samples_Q Q channel samples
     */
    void analyze_signal(const std::vector<int16_t>& samples_I,
                       const std::vector<int16_t>& samples_Q);
    
    /**
     * @brief Save constellation diagram to CSV
     * @param samples_I I channel samples
     * @param samples_Q Q channel samples
     * @param filename Output CSV filename
     */
    void save_constellation(const std::vector<int16_t>& samples_I,
                           const std::vector<int16_t>& samples_Q,
                           const std::string& filename);
    
    /**
     * @brief Debug bit alignment issues
     * @param symbols Demodulated symbols
     * @param threshold Decision threshold
     */
    void debug_bit_alignment(const std::vector<int16_t>& symbols,
                            int16_t threshold);
    
private:
    // ===== Configuration =====
    Config config_;
    
    // ===== Modulation-Specific Encoding =====
    void encode_bpsk(const std::vector<uint8_t>& bytes,
                     std::vector<int16_t>& samples);
    
    void encode_qpsk(const std::vector<uint8_t>& bytes,
                     std::vector<int16_t>& samples_I,
                     std::vector<int16_t>& samples_Q);
    
    void encode_8psk(const std::vector<uint8_t>& bytes,
                     std::vector<int16_t>& samples_I,
                     std::vector<int16_t>& samples_Q);
    
    void encode_16qam(const std::vector<uint8_t>& bytes,
                      std::vector<int16_t>& samples_I,
                      std::vector<int16_t>& samples_Q);
    
    // ===== Modulation-Specific Decoding =====
    bool decode_bpsk(const std::vector<int16_t>& samples,
                     std::vector<uint8_t>& bytes);
    
    bool decode_qpsk(const std::vector<int16_t>& samples_I,
                     const std::vector<int16_t>& samples_Q,
                     std::vector<uint8_t>& bytes);
    
    bool decode_8psk(const std::vector<int16_t>& samples_I,
                     const std::vector<int16_t>& samples_Q,
                     std::vector<uint8_t>& bytes);
    
    bool decode_16qam(const std::vector<int16_t>& samples_I,
                      const std::vector<int16_t>& samples_Q,
                      std::vector<uint8_t>& bytes);
    
    // ===== Helper Functions =====
    
    /**
     * @brief Convert string to bytes
     */
    std::vector<uint8_t> string_to_bytes(const std::string& str);
    
    /**
     * @brief Convert bytes to string
     */
    std::string bytes_to_string(const std::vector<uint8_t>& bytes);
    
    /**
     * @brief Convert bytes to bit stream (MSB first)
     */
    std::vector<bool> bytes_to_bits(const std::vector<uint8_t>& bytes);
    
    /**
     * @brief Convert bit stream to bytes (MSB first)
     */
    std::vector<uint8_t> bits_to_bytes(const std::vector<bool>& bits);
    
    /**
     * @brief Calculate CRC16 checksum
     */
    uint16_t calculate_crc16(const std::vector<uint8_t>& data);
    
    /**
     * @brief Generate preamble sequence (8-bit for byte alignment)
     */
    void generate_preamble(std::vector<int16_t>& samples_I,
                          std::vector<int16_t>& samples_Q);
    
    /**
     * @brief Detect preamble in received samples
     * @param samples_I I channel samples
     * @param samples_Q Q channel samples
     * @param start_index Output: index where data starts (after preamble)
     * @return true if preamble detected
     */
    bool detect_preamble(const std::vector<int16_t>& samples_I,
                        const std::vector<int16_t>& samples_Q,
                        size_t& start_index);
    
    /**
     * @brief Downsample oversampled signal to symbol rate
     * Uses timing recovery to find optimal sampling phase
     */
    std::vector<int16_t> downsample_symbols(const std::vector<int16_t>& samples);
    
    /**
     * @brief Find optimal sampling phase (returns phase offset 0..samples_per_symbol-1)
     */
    uint32_t find_optimal_phase(const std::vector<int16_t>& samples);
};

} // namespace codec

#endif // STRINGCODEC_HPP
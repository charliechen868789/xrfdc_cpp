/**
 * @file main.cpp
 * @brief Main entry point for RF Data Converter application
 * 
 * This application initializes and configures the Xilinx RF Data Converter
 * on the ZCU216 evaluation board, including:
 * - Clock initialization (LMK04828 + LMX2594 PLLs)
 * - DAC configuration
 * - ADC configuration
 * - Loopback testing
 */

#include "RfdcApp.hpp"
#include <iostream>
#include <cstdlib>

int main(int argc, char* argv[]) {
    // Parse command line arguments (if any)
    std::string app_name = "RF Data Converter Application";
    if (argc > 1) {
        app_name = argv[1];
    }
    
    try {
        // Create and run the application
        RfDcApp app(app_name);
        app.run();
        
        return EXIT_SUCCESS;
        
    } catch (const std::exception& e) {
        std::cerr << "\nFatal Error: " << e.what() << "\n";
        std::cerr << "Application terminated.\n";
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "\nUnknown fatal error occurred.\n";
        std::cerr << "Application terminated.\n";
        return EXIT_FAILURE;
    }
}
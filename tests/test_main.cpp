/**
 * Main test runner for STN unit tests
 */
#include <gtest/gtest.h>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    // Set up any global test environment here
    std::cout << "Running STN Navigation Unit Tests\n";
    std::cout << "==================================\n\n";
    
    int result = RUN_ALL_TESTS();
    
    std::cout << "\n==================================\n";
    std::cout << "Test run complete\n";
    
    return result;
}
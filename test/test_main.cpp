/**
 * Main test runner for GPS-Free Navigation System
 * Uses Google Test framework
 */

#include <gtest/gtest.h>
#include <iostream>

// Custom test environment for setup/teardown
class NavigationTestEnvironment : public ::testing::Environment {
public:
    void SetUp() override {
        std::cout << "Setting up test environment..." << std::endl;
        // Initialize any global test resources here
    }

    void TearDown() override {
        std::cout << "Tearing down test environment..." << std::endl;
        // Clean up global test resources here
    }
};

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    // Add custom test environment
    ::testing::AddGlobalTestEnvironment(new NavigationTestEnvironment);

    // Run all tests
    return RUN_ALL_TESTS();
}
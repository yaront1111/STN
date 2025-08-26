/**
 * Unit tests for gravity model
 * Validates WGS84 gravity calculations and anomaly handling
 */
#include <gtest/gtest.h>
#include <cmath>
#include "gravity_model.h"

class GravityTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize gravity model if needed
        GravityModel::initialize();
    }
};

TEST_F(GravityTest, NormalGravityAtEquator) {
    // Test normal gravity at equator, sea level
    GravityQuery q;
    q.lat_rad = 0.0;
    q.lon_rad = 0.0;
    q.alt_m = 0.0;
    
    GravityResult result = GravityModel::normal(q);
    
    // WGS84 gravity at equator: 9.7803253359 m/s²
    EXPECT_NEAR(result.g_mps2, 9.7803253359, 1e-6);
    
    // Gravity should point down (positive in NED)
    EXPECT_NEAR(result.g_NED.x(), 0.0, 1e-10);
    EXPECT_NEAR(result.g_NED.y(), 0.0, 1e-10);
    EXPECT_GT(result.g_NED.z(), 9.7);
}

TEST_F(GravityTest, NormalGravityAtPole) {
    // Test normal gravity at pole, sea level
    GravityQuery q;
    q.lat_rad = M_PI/2;  // 90 degrees
    q.lon_rad = 0.0;
    q.alt_m = 0.0;
    
    GravityResult result = GravityModel::normal(q);
    
    // WGS84 gravity at pole: ~9.8321849378 m/s²
    EXPECT_NEAR(result.g_mps2, 9.8321849378, 1e-2);
    EXPECT_GT(result.g_mps2, 9.82);  // Should be stronger than equator
}

TEST_F(GravityTest, GravityVariationWithLatitude) {
    // Gravity should increase from equator to pole
    GravityQuery q;
    q.lon_rad = 0.0;
    q.alt_m = 0.0;
    
    q.lat_rad = 0.0;
    double g_equator = GravityModel::normal(q).g_mps2;
    
    q.lat_rad = M_PI/4;  // 45 degrees
    double g_mid = GravityModel::normal(q).g_mps2;
    
    q.lat_rad = M_PI/2;  // 90 degrees
    double g_pole = GravityModel::normal(q).g_mps2;
    
    EXPECT_LT(g_equator, g_mid);
    EXPECT_LT(g_mid, g_pole);
    
    // Total variation should be about 0.05 m/s²
    EXPECT_NEAR(g_pole - g_equator, 0.052, 0.001);
}

TEST_F(GravityTest, FreeAirCorrection) {
    // Test altitude correction (free-air)
    GravityQuery q;
    q.lat_rad = M_PI/4;  // 45 degrees
    q.lon_rad = 0.0;
    
    q.alt_m = 0.0;
    double g_sea_level = GravityModel::normal(q).g_mps2;
    
    q.alt_m = 1000.0;
    double g_1km = GravityModel::normal(q).g_mps2;
    
    q.alt_m = 10000.0;
    double g_10km = GravityModel::normal(q).g_mps2;
    
    // Gravity decreases with altitude
    EXPECT_LT(g_1km, g_sea_level);
    EXPECT_LT(g_10km, g_1km);
    
    // Free-air gradient is approximately -3.086e-6 m/s² per meter
    double expected_decrease_1km = 3.086e-6 * 1000;
    EXPECT_NEAR(g_sea_level - g_1km, expected_decrease_1km, 1e-7);
}

TEST_F(GravityTest, GravityAnomalyRange) {
    // Test that anomalies are in reasonable range
    GravityQuery q;
    q.lat_rad = 0.7854;  // 45 degrees
    q.lon_rad = 0.1745;  // 10 degrees
    q.alt_m = 500.0;
    
    GravityResult result = GravityModel::withAnomaly(q);
    
    // Anomalies should typically be within ±100 mGal (±0.001 m/s²)
    double anomaly_mps2 = result.g_mps2 - GravityModel::normal(q).g_mps2;
    EXPECT_LT(std::abs(anomaly_mps2), 0.002);  // ±200 mGal max
    
    // Anomaly should be reported in mGal
    if (result.anomaly_mgal != 0.0) {
        EXPECT_NEAR(anomaly_mps2, result.anomaly_mgal * 1e-5, 1e-7);
    }
}

TEST_F(GravityTest, GravityDirectionAlwaysDown) {
    // Gravity should always point down (positive Z in NED)
    for (double lat = -90; lat <= 90; lat += 30) {
        for (double lon = -180; lon <= 180; lon += 60) {
            GravityQuery q;
            q.lat_rad = lat * M_PI / 180.0;
            q.lon_rad = lon * M_PI / 180.0;
            q.alt_m = 1000.0;
            
            GravityResult result = GravityModel::normal(q);
            
            // X and Y components should be zero (no horizontal gravity)
            EXPECT_NEAR(result.g_NED.x(), 0.0, 1e-10);
            EXPECT_NEAR(result.g_NED.y(), 0.0, 1e-10);
            
            // Z component should be positive (down in NED)
            EXPECT_GT(result.g_NED.z(), 9.7);
            EXPECT_LT(result.g_NED.z(), 9.85);
        }
    }
}

TEST_F(GravityTest, ConsistencyCheck) {
    // Multiple calls with same input should give same output
    GravityQuery q;
    q.lat_rad = 0.5;
    q.lon_rad = 1.0;
    q.alt_m = 2000.0;
    
    GravityResult r1 = GravityModel::normal(q);
    GravityResult r2 = GravityModel::normal(q);
    
    EXPECT_EQ(r1.g_mps2, r2.g_mps2);
    EXPECT_EQ(r1.g_NED, r2.g_NED);
}
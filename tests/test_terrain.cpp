/**
 * Unit tests for terrain provider
 * Tests both synthetic and real terrain functionality
 */
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include "terrain_provider.h"

class TerrainTest : public ::testing::Test {
protected:
    void SetUp() override {
        terrain = std::make_unique<TerrainProvider>();
    }
    
    std::unique_ptr<TerrainProvider> terrain;
};

TEST_F(TerrainTest, SyntheticTerrainContinuity) {
    // Terrain should be continuous (no jumps)
    double n = 100.0;
    double e = 200.0;
    
    TerrainSample s1 = terrain->sample(n, e);
    TerrainSample s2 = terrain->sample(n + 0.1, e);
    TerrainSample s3 = terrain->sample(n, e + 0.1);
    
    // Small position changes should give small height changes
    EXPECT_LT(std::abs(s2.h - s1.h), 1.0);  // Less than 1m for 0.1m move
    EXPECT_LT(std::abs(s3.h - s1.h), 1.0);
}

TEST_F(TerrainTest, GradientConsistency) {
    // Gradient should match finite difference approximation
    double n = 500.0;
    double e = 1000.0;
    double delta = 1.0;  // 1 meter for finite difference
    
    TerrainSample center = terrain->sample(n, e);
    TerrainSample north = terrain->sample(n + delta, e);
    TerrainSample south = terrain->sample(n - delta, e);
    TerrainSample east = terrain->sample(n, e + delta);
    TerrainSample west = terrain->sample(n, e - delta);
    
    // Finite difference gradients
    double grad_n_fd = (north.h - south.h) / (2 * delta);
    double grad_e_fd = (east.h - west.h) / (2 * delta);
    
    // Should match analytical gradient
    EXPECT_NEAR(center.grad.x(), grad_n_fd, 0.01);  // Within 1% slope
    EXPECT_NEAR(center.grad.y(), grad_e_fd, 0.01);
}

TEST_F(TerrainTest, TerrainAmplitude) {
    // Check that terrain has reasonable amplitude
    double max_height = -1e10;
    double min_height = 1e10;
    
    // Sample grid of points
    for (double n = -5000; n <= 5000; n += 500) {
        for (double e = -5000; e <= 5000; e += 500) {
            TerrainSample s = terrain->sample(n, e);
            max_height = std::max(max_height, s.h);
            min_height = std::min(min_height, s.h);
        }
    }
    
    double terrain_range = max_height - min_height;
    
    // Synthetic terrain should have significant variation
    EXPECT_GT(terrain_range, 50.0);   // At least 50m variation
    EXPECT_LT(terrain_range, 500.0);  // But not unrealistic
}

TEST_F(TerrainTest, SlopeRange) {
    // Test that slopes are in reasonable range
    double max_slope = 0.0;
    
    for (double n = -2000; n <= 2000; n += 200) {
        for (double e = -2000; e <= 2000; e += 200) {
            TerrainSample s = terrain->sample(n, e);
            double slope = s.grad.norm();
            max_slope = std::max(max_slope, slope);
        }
    }
    
    // Slopes should be reasonable (not cliffs)
    EXPECT_LT(max_slope, 1.0);  // Less than 45 degrees (100% grade)
    EXPECT_GT(max_slope, 0.01); // But not completely flat
}

TEST_F(TerrainTest, TerrainPeriodicity) {
    // Synthetic terrain is based on sine waves, should have some periodicity
    double n0 = 0.0;
    double e0 = 0.0;
    
    TerrainSample s0 = terrain->sample(n0, e0);
    
    // Check for approximate periodicity (based on L1, L2 parameters)
    double period = 2000.0;  // Approximate from default L1
    TerrainSample s_period = terrain->sample(n0 + period, e0);
    
    // Heights should be similar (not necessarily identical due to multiple frequencies)
    EXPECT_LT(std::abs(s_period.h - s0.h), 50.0);
}

TEST_F(TerrainTest, ZeroMeanTerrain) {
    // Synthetic terrain should have approximately zero mean
    double sum = 0.0;
    int count = 0;
    
    for (double n = -5000; n <= 5000; n += 100) {
        for (double e = -5000; e <= 5000; e += 100) {
            TerrainSample s = terrain->sample(n, e);
            sum += s.h;
            count++;
        }
    }
    
    double mean_height = sum / count;
    
    // Mean should be near zero for synthetic terrain
    EXPECT_NEAR(mean_height, 0.0, 10.0);
}

TEST_F(TerrainTest, CoordinateTransform) {
    // Test NED to Lat/Lon conversion (if real terrain is used)
    if (terrain->srtm_reader) {
        double n = 1000.0;  // 1km north
        double e = 2000.0;  // 2km east
        
        double lat, lon;
        terrain->nedToLatLon(n, e, lat, lon);
        
        // Check that we moved approximately right amount
        // 1 degree latitude ≈ 111km
        double expected_lat = terrain->ref_lat + n / 111320.0;
        double expected_lon = terrain->ref_lon + e / (111320.0 * cos(terrain->ref_lat * M_PI / 180.0));
        
        EXPECT_NEAR(lat, expected_lat, 0.0001);  // Within ~10m
        EXPECT_NEAR(lon, expected_lon, 0.0001);
    }
}

TEST_F(TerrainTest, GradientSymmetry) {
    // Gradient should be anti-symmetric for opposite directions
    double n = 100.0;
    double e = 200.0;
    double delta = 10.0;
    
    TerrainSample s_center = terrain->sample(n, e);
    
    // Sample in circle around center
    for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 4) {
        double dn = delta * cos(angle);
        double de = delta * sin(angle);
        
        TerrainSample s1 = terrain->sample(n + dn, e + de);
        TerrainSample s2 = terrain->sample(n - dn, e - de);
        
        // Heights should be symmetric around center (approximately)
        double h_diff1 = s1.h - s_center.h;
        double h_diff2 = s2.h - s_center.h;
        
        // For small distances, Taylor expansion gives:
        // h(x+dx) ≈ h(x) + grad·dx
        // h(x-dx) ≈ h(x) - grad·dx
        // So h(x+dx) + h(x-dx) ≈ 2*h(x)
        
        EXPECT_NEAR(s1.h + s2.h, 2 * s_center.h, 5.0);  // Within 5m
    }
}
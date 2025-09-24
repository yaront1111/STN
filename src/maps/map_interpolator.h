/**
 * Map Interpolator
 * Provides high-performance interpolation methods for map data
 * Supports bilinear, bicubic, and spline interpolation
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <optional>

namespace Navigation {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

/**
 * Interpolation method enum
 */
enum class InterpolationMethod {
    NEAREST,     // Nearest neighbor
    BILINEAR,    // Bilinear interpolation (2x2 grid)
    BICUBIC,     // Bicubic interpolation (4x4 grid)
    SPLINE       // B-spline interpolation
};

/**
 * Grid data structure for interpolation
 */
struct GridData {
    MatrixXd values;           // Grid values (rows=lat, cols=lon)
    double min_lat, max_lat;   // Latitude bounds
    double min_lon, max_lon;   // Longitude bounds
    double lat_spacing;        // Grid spacing in latitude
    double lon_spacing;        // Grid spacing in longitude

    // Get grid indices for a coordinate
    bool getIndices(double lat, double lon, int& i, int& j) const {
        if (lat < min_lat || lat > max_lat || lon < min_lon || lon > max_lon) {
            return false;
        }

        i = static_cast<int>((lat - min_lat) / lat_spacing);
        j = static_cast<int>((lon - min_lon) / lon_spacing);

        // Clamp to grid bounds
        i = std::max(0, std::min(i, static_cast<int>(values.rows() - 1)));
        j = std::max(0, std::min(j, static_cast<int>(values.cols() - 1)));

        return true;
    }
};

/**
 * Interpolation result with gradient
 */
struct InterpolationResult {
    double value;                // Interpolated value
    Vector2d gradient;           // Gradient [dv/dlat, dv/dlon]
    double confidence = 1.0;     // Interpolation confidence [0,1]
    bool valid = true;           // Validity flag
};

/**
 * Map Interpolator class
 */
class MapInterpolator {
private:
    InterpolationMethod method_;
    GridData grid_data_;

    // Precomputed coefficients for bicubic
    mutable std::vector<std::vector<Eigen::Matrix4d>> bicubic_coeffs_;
    bool coeffs_computed_ = false;

    // Spline control points
    mutable MatrixXd spline_coeffs_;

public:
    MapInterpolator(InterpolationMethod method = InterpolationMethod::BICUBIC);
    ~MapInterpolator() = default;

    // Set grid data
    void setGridData(const GridData& data);
    void setGridData(const MatrixXd& values,
                    double min_lat, double max_lat,
                    double min_lon, double max_lon);

    // Main interpolation function
    InterpolationResult interpolate(double lat, double lon) const;

    // Batch interpolation for efficiency
    std::vector<InterpolationResult> interpolateBatch(
        const std::vector<Vector2d>& coords) const;

    // Get gradient at location
    Vector2d getGradient(double lat, double lon) const;

    // Get Hessian matrix (second derivatives)
    Matrix2d getHessian(double lat, double lon) const;

    // Precompute coefficients for faster interpolation
    void precomputeCoefficients();

private:
    // Interpolation methods
    InterpolationResult nearestNeighbor(double lat, double lon) const;
    InterpolationResult bilinearInterpolate(double lat, double lon) const;
    InterpolationResult bicubicInterpolate(double lat, double lon) const;
    InterpolationResult splineInterpolate(double lat, double lon) const;

    // Bicubic helpers
    void computeBicubicCoeffs(int i, int j);
    double bicubicKernel(double x) const;
    Eigen::Matrix4d getBicubicMatrix(int i, int j) const;

    // Numerical derivatives
    Vector2d numericalGradient(double lat, double lon, double h = 1e-6) const;
    Matrix2d numericalHessian(double lat, double lon, double h = 1e-5) const;

    // Boundary handling
    double extrapolate(double lat, double lon) const;
    bool isInBounds(double lat, double lon) const;
};

/**
 * Specialized gravity field interpolator
 */
class GravityFieldInterpolator : public MapInterpolator {
private:
    GridData anomaly_grid_;      // Gravity anomaly grid
    GridData gradient_grid_;     // Gradient tensor grid (5 components)
    GridData geoid_grid_;        // Geoid height grid

public:
    GravityFieldInterpolator();

    // Set gravity-specific grids
    void setAnomalyGrid(const GridData& data);
    void setGradientGrid(const GridData& data);
    void setGeoidGrid(const GridData& data);

    // Gravity field queries
    double getGravityAnomaly(double lat, double lon) const;
    Eigen::Matrix<double, 5, 1> getGradientTensor(double lat, double lon) const;
    double getGeoidHeight(double lat, double lon) const;

    // Combined query for efficiency
    struct GravityResult {
        double anomaly;
        Eigen::Matrix<double, 5, 1> gradient_tensor;
        double geoid_height;
        double confidence;
    };
    GravityResult queryGravityField(double lat, double lon) const;
};

/**
 * Terrain elevation interpolator
 */
class TerrainInterpolator : public MapInterpolator {
private:
    GridData elevation_grid_;
    bool handle_water_ = true;
    double water_level_ = 0.0;

public:
    TerrainInterpolator();

    // Set terrain grid
    void setElevationGrid(const GridData& data);
    void setWaterHandling(bool enable, double level = 0.0);

    // Terrain queries
    double getElevation(double lat, double lon) const;
    Vector3d getSurfaceNormal(double lat, double lon) const;
    double getSlope(double lat, double lon) const;
    double getAspect(double lat, double lon) const;
    double getRoughness(double lat, double lon, double radius = 0.01) const;

    // Line-of-sight analysis
    bool hasLineOfSight(const Vector2d& from, const Vector2d& to,
                       double observer_height = 0) const;
    std::vector<double> getElevationProfile(const Vector2d& from,
                                           const Vector2d& to,
                                           int num_points = 100) const;
};

/**
 * Adaptive grid interpolator with LOD
 */
class AdaptiveInterpolator {
private:
    std::vector<GridData> lod_grids_;  // Level-of-detail grids
    std::vector<double> lod_thresholds_; // Distance thresholds for LOD
    InterpolationMethod method_;

public:
    AdaptiveInterpolator(InterpolationMethod method = InterpolationMethod::BICUBIC);

    // Add LOD level
    void addLODLevel(const GridData& grid, double threshold);

    // Adaptive interpolation based on required resolution
    InterpolationResult interpolate(double lat, double lon,
                                   double required_resolution) const;

    // Get appropriate LOD level
    int selectLOD(double required_resolution) const;

    // Memory management
    void clearLOD(int level);
    size_t getMemoryUsage() const;
};

} // namespace Navigation
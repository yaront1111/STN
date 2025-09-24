/**
 * Map Interpolator Implementation
 * High-performance interpolation for map data
 */

#include "map_interpolator.h"
#include "../utils/logger.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <sstream>

namespace Navigation {

/**
 * Map Interpolator
 */
MapInterpolator::MapInterpolator(InterpolationMethod method)
    : method_(method) {
    {

        std::stringstream msg;

        msg << "MapInterpolator initialized with method: " << static_cast<int>(method);

        LOG_DEBUG(msg.str());

    }
}

void MapInterpolator::setGridData(const GridData& data) {
    grid_data_ = data;
    coeffs_computed_ = false;

    // Validate grid data
    if (grid_data_.values.rows() < 2 || grid_data_.values.cols() < 2) {
        LOG_ERROR("Grid data too small for interpolation");
        throw std::invalid_argument("Grid must be at least 2x2");
    }

    {


        std::stringstream msg;


        msg << "Grid data set: " << grid_data_.values.rows();


        LOG_INFO(msg.str());


    }
}

void MapInterpolator::setGridData(const MatrixXd& values,
                                 double min_lat, double max_lat,
                                 double min_lon, double max_lon) {
    grid_data_.values = values;
    grid_data_.min_lat = min_lat;
    grid_data_.max_lat = max_lat;
    grid_data_.min_lon = min_lon;
    grid_data_.max_lon = max_lon;

    grid_data_.lat_spacing = (max_lat - min_lat) / (values.rows() - 1);
    grid_data_.lon_spacing = (max_lon - min_lon) / (values.cols() - 1);

    coeffs_computed_ = false;
}

InterpolationResult MapInterpolator::interpolate(double lat, double lon) const {
    // Check bounds
    if (!isInBounds(lat, lon)) {
        InterpolationResult result;
        result.valid = false;
        result.confidence = 0.0;
        result.value = extrapolate(lat, lon);
        return result;
    }

    // Select interpolation method
    switch (method_) {
        case InterpolationMethod::NEAREST:
            return nearestNeighbor(lat, lon);
        case InterpolationMethod::BILINEAR:
            return bilinearInterpolate(lat, lon);
        case InterpolationMethod::BICUBIC:
            return bicubicInterpolate(lat, lon);
        case InterpolationMethod::SPLINE:
            return splineInterpolate(lat, lon);
        default:
            return bilinearInterpolate(lat, lon);
    }
}

std::vector<InterpolationResult> MapInterpolator::interpolateBatch(
    const std::vector<Vector2d>& coords) const {

    std::vector<InterpolationResult> results;
    results.reserve(coords.size());

    for (const auto& coord : coords) {
        results.push_back(interpolate(coord.x(), coord.y()));
    }

    return results;
}

Vector2d MapInterpolator::getGradient(double lat, double lon) const {
    if (method_ == InterpolationMethod::BICUBIC && coeffs_computed_) {
        // Use analytical gradient from bicubic
        auto result = bicubicInterpolate(lat, lon);
        return result.gradient;
    }

    // Use numerical gradient
    return numericalGradient(lat, lon);
}

Matrix2d MapInterpolator::getHessian(double lat, double lon) const {
    return numericalHessian(lat, lon);
}

void MapInterpolator::precomputeCoefficients() {
    if (method_ != InterpolationMethod::BICUBIC) {
        return;
    }

    LOG_INFO("Precomputing bicubic coefficients...");

    int rows = grid_data_.values.rows();
    int cols = grid_data_.values.cols();

    bicubic_coeffs_.resize(rows - 1);
    for (int i = 0; i < rows - 1; ++i) {
        bicubic_coeffs_[i].resize(cols - 1);
        for (int j = 0; j < cols - 1; ++j) {
            computeBicubicCoeffs(i, j);
        }
    }

    coeffs_computed_ = true;
    LOG_INFO("Bicubic coefficients computed");
}

InterpolationResult MapInterpolator::nearestNeighbor(double lat, double lon) const {
    InterpolationResult result;

    int i, j;
    if (!grid_data_.getIndices(lat, lon, i, j)) {
        result.valid = false;
        return result;
    }

    result.value = grid_data_.values(i, j);
    result.gradient = numericalGradient(lat, lon);
    result.confidence = 0.5;  // Lower confidence for nearest neighbor

    return result;
}

InterpolationResult MapInterpolator::bilinearInterpolate(double lat, double lon) const {
    InterpolationResult result;

    // Get grid indices
    double fi = (lat - grid_data_.min_lat) / grid_data_.lat_spacing;
    double fj = (lon - grid_data_.min_lon) / grid_data_.lon_spacing;

    int i = static_cast<int>(std::floor(fi));
    int j = static_cast<int>(std::floor(fj));

    // Check bounds
    if (i < 0 || i >= grid_data_.values.rows() - 1 ||
        j < 0 || j >= grid_data_.values.cols() - 1) {
        result.valid = false;
        return result;
    }

    // Fractional parts
    double u = fi - i;
    double v = fj - j;

    // Get four corner values
    double v00 = grid_data_.values(i, j);
    double v10 = grid_data_.values(i + 1, j);
    double v01 = grid_data_.values(i, j + 1);
    double v11 = grid_data_.values(i + 1, j + 1);

    // Bilinear interpolation
    result.value = (1 - u) * (1 - v) * v00 +
                   u * (1 - v) * v10 +
                   (1 - u) * v * v01 +
                   u * v * v11;

    // Compute gradient
    double dv_du = (1 - v) * (v10 - v00) + v * (v11 - v01);
    double dv_dv = (1 - u) * (v01 - v00) + u * (v11 - v10);

    result.gradient.x() = dv_du / grid_data_.lat_spacing;
    result.gradient.y() = dv_dv / grid_data_.lon_spacing;

    result.confidence = 0.8;

    return result;
}

InterpolationResult MapInterpolator::bicubicInterpolate(double lat, double lon) const {
    InterpolationResult result;

    // Get grid indices
    double fi = (lat - grid_data_.min_lat) / grid_data_.lat_spacing;
    double fj = (lon - grid_data_.min_lon) / grid_data_.lon_spacing;

    int i = static_cast<int>(std::floor(fi));
    int j = static_cast<int>(std::floor(fj));

    // Need 4x4 grid for bicubic
    if (i < 1 || i >= grid_data_.values.rows() - 2 ||
        j < 1 || j >= grid_data_.values.cols() - 2) {
        // Fall back to bilinear near boundaries
        return bilinearInterpolate(lat, lon);
    }

    // Fractional parts
    double u = fi - i;
    double v = fj - j;

    // Bicubic convolution
    double value = 0;
    double grad_u = 0;
    double grad_v = 0;

    for (int di = -1; di <= 2; ++di) {
        for (int dj = -1; dj <= 2; ++dj) {
            double val = grid_data_.values(i + di, j + dj);
            double wu = bicubicKernel(u - di);
            double wv = bicubicKernel(v - dj);
            double dwu = bicubicKernel(u - di) * (di == 0 ? -1 : 1);  // Derivative

            value += val * wu * wv;
            grad_u += val * dwu * wv;
            grad_v += val * wu * bicubicKernel(v - dj) * (dj == 0 ? -1 : 1);
        }
    }

    result.value = value;
    result.gradient.x() = grad_u / grid_data_.lat_spacing;
    result.gradient.y() = grad_v / grid_data_.lon_spacing;
    result.confidence = 0.95;

    return result;
}

InterpolationResult MapInterpolator::splineInterpolate(double lat, double lon) const {
    // B-spline interpolation (simplified cubic B-spline)
    // For production, consider using a proper spline library

    // Fall back to bicubic for now
    return bicubicInterpolate(lat, lon);
}

void MapInterpolator::computeBicubicCoeffs(int i, int j) {
    if (static_cast<size_t>(i) >= bicubic_coeffs_.size() || static_cast<size_t>(j) >= bicubic_coeffs_[i].size()) {
        bicubic_coeffs_.resize(std::max(i + 1, static_cast<int>(bicubic_coeffs_.size())));
        bicubic_coeffs_[i].resize(std::max(j + 1, static_cast<int>(bicubic_coeffs_[i].size())));
    }

    // Get 4x4 grid of values
    Eigen::Matrix4d values;
    for (int di = 0; di < 4; ++di) {
        for (int dj = 0; dj < 4; ++dj) {
            int gi = std::max(0, std::min(i + di - 1, static_cast<int>(grid_data_.values.rows() - 1)));
            int gj = std::max(0, std::min(j + dj - 1, static_cast<int>(grid_data_.values.cols() - 1)));
            values(di, dj) = grid_data_.values(gi, gj);
        }
    }

    bicubic_coeffs_[i][j] = values;
}

double MapInterpolator::bicubicKernel(double x) const {
    // Cubic convolution kernel
    double ax = std::abs(x);

    if (ax <= 1) {
        return 1 - 2 * ax * ax + ax * ax * ax;
    } else if (ax < 2) {
        return 4 - 8 * ax + 5 * ax * ax - ax * ax * ax;
    }

    return 0;
}

Vector2d MapInterpolator::numericalGradient(double lat, double lon, double h) const {
    Vector2d gradient;

    // Central differences
    auto plus_lat = interpolate(lat + h, lon);
    auto minus_lat = interpolate(lat - h, lon);
    gradient.x() = (plus_lat.value - minus_lat.value) / (2 * h);

    auto plus_lon = interpolate(lat, lon + h);
    auto minus_lon = interpolate(lat, lon - h);
    gradient.y() = (plus_lon.value - minus_lon.value) / (2 * h);

    return gradient;
}

Matrix2d MapInterpolator::numericalHessian(double lat, double lon, double h) const {
    Matrix2d hessian;

    // Second derivatives using central differences
    auto center = interpolate(lat, lon);
    auto plus_lat = interpolate(lat + h, lon);
    auto minus_lat = interpolate(lat - h, lon);
    auto plus_lon = interpolate(lat, lon + h);
    auto minus_lon = interpolate(lat, lon - h);

    // d²f/dlat²
    hessian(0, 0) = (plus_lat.value - 2 * center.value + minus_lat.value) / (h * h);

    // d²f/dlon²
    hessian(1, 1) = (plus_lon.value - 2 * center.value + minus_lon.value) / (h * h);

    // Mixed derivative d²f/dlat·dlon
    auto plus_plus = interpolate(lat + h, lon + h);
    auto minus_minus = interpolate(lat - h, lon - h);
    auto plus_minus = interpolate(lat + h, lon - h);
    auto minus_plus = interpolate(lat - h, lon + h);

    hessian(0, 1) = (plus_plus.value - plus_minus.value - minus_plus.value + minus_minus.value) / (4 * h * h);
    hessian(1, 0) = hessian(0, 1);  // Symmetric

    return hessian;
}

double MapInterpolator::extrapolate(double lat, double lon) const {
    // Simple nearest-edge extrapolation
    double clamp_lat = std::max(grid_data_.min_lat, std::min(grid_data_.max_lat, lat));
    double clamp_lon = std::max(grid_data_.min_lon, std::min(grid_data_.max_lon, lon));

    return interpolate(clamp_lat, clamp_lon).value;
}

bool MapInterpolator::isInBounds(double lat, double lon) const {
    return lat >= grid_data_.min_lat && lat <= grid_data_.max_lat &&
           lon >= grid_data_.min_lon && lon <= grid_data_.max_lon;
}

/**
 * Gravity Field Interpolator
 */
GravityFieldInterpolator::GravityFieldInterpolator()
    : MapInterpolator(InterpolationMethod::BICUBIC) {
    LOG_DEBUG("GravityFieldInterpolator initialized");
}

void GravityFieldInterpolator::setAnomalyGrid(const GridData& data) {
    anomaly_grid_ = data;
    setGridData(data);  // Use base class for primary interpolation
}

void GravityFieldInterpolator::setGradientGrid(const GridData& data) {
    gradient_grid_ = data;
}

void GravityFieldInterpolator::setGeoidGrid(const GridData& data) {
    geoid_grid_ = data;
}

double GravityFieldInterpolator::getGravityAnomaly(double lat, double lon) const {
    return interpolate(lat, lon).value;
}

Eigen::Matrix<double, 5, 1> GravityFieldInterpolator::getGradientTensor(double lat, double lon) const {
    Eigen::Matrix<double, 5, 1> tensor;

    if (gradient_grid_.values.size() == 0) {
        // Compute from anomaly gradient
        auto grad = getGradient(lat, lon);
        tensor << grad.x(), grad.y(), 0, 0, 0;  // Simplified
    } else {
        // Interpolate each tensor component
        for (int i = 0; i < 5; ++i) {
            MapInterpolator interp(InterpolationMethod::BICUBIC);

            // Extract component grid
            GridData comp_grid = gradient_grid_;
            // Assuming gradient_grid stores 5 components somehow
            // This would need proper implementation based on data format

            tensor(i) = interp.interpolate(lat, lon).value;
        }
    }

    return tensor;
}

double GravityFieldInterpolator::getGeoidHeight(double lat, double lon) const {
    if (geoid_grid_.values.size() == 0) {
        return 0;  // No geoid data
    }

    MapInterpolator geoid_interp(InterpolationMethod::BICUBIC);
    geoid_interp.setGridData(geoid_grid_);
    return geoid_interp.interpolate(lat, lon).value;
}

GravityFieldInterpolator::GravityResult GravityFieldInterpolator::queryGravityField(
    double lat, double lon) const {

    GravityResult result;

    auto interp_result = interpolate(lat, lon);
    result.anomaly = interp_result.value;
    result.confidence = interp_result.confidence;
    result.gradient_tensor = getGradientTensor(lat, lon);
    result.geoid_height = getGeoidHeight(lat, lon);

    return result;
}

/**
 * Terrain Interpolator
 */
TerrainInterpolator::TerrainInterpolator()
    : MapInterpolator(InterpolationMethod::BICUBIC) {
    LOG_DEBUG("TerrainInterpolator initialized");
}

void TerrainInterpolator::setElevationGrid(const GridData& data) {
    elevation_grid_ = data;
    setGridData(data);
}

void TerrainInterpolator::setWaterHandling(bool enable, double level) {
    handle_water_ = enable;
    water_level_ = level;
}

double TerrainInterpolator::getElevation(double lat, double lon) const {
    double elev = interpolate(lat, lon).value;

    if (handle_water_ && elev < water_level_) {
        return water_level_;  // Clamp to water level
    }

    return elev;
}

Vector3d TerrainInterpolator::getSurfaceNormal(double lat, double lon) const {
    auto grad = getGradient(lat, lon);

    // Convert gradient to surface normal
    // Assuming gradient is [dz/dlat, dz/dlon]
    Vector3d normal;
    normal.x() = -grad.x();
    normal.y() = -grad.y();
    normal.z() = 1.0;
    normal.normalize();

    return normal;
}

double TerrainInterpolator::getSlope(double lat, double lon) const {
    auto grad = getGradient(lat, lon);
    double slope_rad = std::atan(grad.norm());
    return slope_rad * 180.0 / M_PI;  // Convert to degrees
}

double TerrainInterpolator::getAspect(double lat, double lon) const {
    auto grad = getGradient(lat, lon);
    double aspect_rad = std::atan2(grad.y(), grad.x());
    double aspect_deg = aspect_rad * 180.0 / M_PI;

    // Convert to compass bearing (0=N, 90=E, 180=S, 270=W)
    aspect_deg = 90.0 - aspect_deg;
    if (aspect_deg < 0) aspect_deg += 360.0;

    return aspect_deg;
}

double TerrainInterpolator::getRoughness(double lat, double lon, double radius) const {
    // Compute terrain roughness as standard deviation of elevations
    std::vector<double> elevations;

    int n_samples = 8;  // Sample in 8 directions
    for (int i = 0; i < n_samples; ++i) {
        double angle = 2.0 * M_PI * i / n_samples;
        double dlat = radius * std::cos(angle);
        double dlon = radius * std::sin(angle);

        elevations.push_back(getElevation(lat + dlat, lon + dlon));
    }

    // Compute standard deviation
    double mean = std::accumulate(elevations.begin(), elevations.end(), 0.0) / elevations.size();
    double sq_sum = 0;
    for (double e : elevations) {
        sq_sum += (e - mean) * (e - mean);
    }

    return std::sqrt(sq_sum / elevations.size());
}

bool TerrainInterpolator::hasLineOfSight(const Vector2d& from, const Vector2d& to,
                                        double observer_height) const {
    // Simple line-of-sight check
    int num_samples = 100;

    double from_elev = getElevation(from.x(), from.y()) + observer_height;
    double to_elev = getElevation(to.x(), to.y());

    for (int i = 1; i < num_samples - 1; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        double lat = from.x() + t * (to.x() - from.x());
        double lon = from.y() + t * (to.y() - from.y());

        double terrain_elev = getElevation(lat, lon);
        double sight_elev = from_elev + t * (to_elev - from_elev);

        if (terrain_elev > sight_elev) {
            return false;  // Terrain blocks line of sight
        }
    }

    return true;
}

std::vector<double> TerrainInterpolator::getElevationProfile(const Vector2d& from,
                                                            const Vector2d& to,
                                                            int num_points) const {
    std::vector<double> profile;
    profile.reserve(num_points);

    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        double lat = from.x() + t * (to.x() - from.x());
        double lon = from.y() + t * (to.y() - from.y());

        profile.push_back(getElevation(lat, lon));
    }

    return profile;
}

/**
 * Adaptive Interpolator
 */
AdaptiveInterpolator::AdaptiveInterpolator(InterpolationMethod method)
    : method_(method) {
    LOG_DEBUG("AdaptiveInterpolator initialized");
}

void AdaptiveInterpolator::addLODLevel(const GridData& grid, double threshold) {
    lod_grids_.push_back(grid);
    lod_thresholds_.push_back(threshold);

    {


        std::stringstream msg;


        msg << "Added LOD level " << lod_grids_.size();


        LOG_INFO(msg.str());


    }
}

InterpolationResult AdaptiveInterpolator::interpolate(double lat, double lon,
                                                     double required_resolution) const {
    int lod = selectLOD(required_resolution);

    if (lod < 0 || static_cast<size_t>(lod) >= lod_grids_.size()) {
        InterpolationResult result;
        result.valid = false;
        return result;
    }

    MapInterpolator interp(method_);
    interp.setGridData(lod_grids_[lod]);

    return interp.interpolate(lat, lon);
}

int AdaptiveInterpolator::selectLOD(double required_resolution) const {
    for (size_t i = 0; i < lod_thresholds_.size(); ++i) {
        if (required_resolution >= lod_thresholds_[i]) {
            return i;
        }
    }

    return lod_grids_.size() - 1;  // Use highest detail
}

void AdaptiveInterpolator::clearLOD(int level) {
    if (level >= 0 && static_cast<size_t>(level) < lod_grids_.size()) {
        lod_grids_[level].values.resize(0, 0);
        {

            std::stringstream msg;

            msg << "Cleared LOD level " << level;

            LOG_INFO(msg.str());

        }
    }
}

size_t AdaptiveInterpolator::getMemoryUsage() const {
    size_t total = 0;

    for (const auto& grid : lod_grids_) {
        total += grid.values.size() * sizeof(double);
    }

    return total;
}

} // namespace Navigation
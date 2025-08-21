#pragma once
#include <vector>
#include <string>
#include <algorithm>
#include <stdexcept>

struct RadaltSampler {
    std::vector<double> T, Z; // seconds, meters
    double delay_s = 0.05;     // fixed latency τ (50ms default)

    void load_csv(const std::string& path); // t,agl_m (header ok)
    // linear sample at time t - delay_s; returns false if out of range
    bool sample(double t_now, double& z_agl_out) const;
};
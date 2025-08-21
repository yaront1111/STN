#include "radalt_sampler.h"
#include <fstream>
#include <sstream>

void RadaltSampler::load_csv(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("radalt open fail: " + path);
    std::string line; bool header = true;
    while (std::getline(f, line)) {
        if (header) { header = false; continue; } // skip header
        if (line.empty()) continue;
        std::stringstream ss(line); std::string tok;
        if (!std::getline(ss, tok, ',')) continue;
        double t = std::stod(tok);
        if (!std::getline(ss, tok, ',')) continue;
        double z = std::stod(tok);
        T.push_back(t); Z.push_back(z);
    }
    if (T.size() < 2) throw std::runtime_error("radalt too short");
}

bool RadaltSampler::sample(double t_now, double& z_agl_out) const {
    const double t = t_now - delay_s;
    if (t < T.front() || t > T.back()) return false;
    auto it = std::lower_bound(T.begin(), T.end(), t);
    if (it == T.begin()) { z_agl_out = Z.front(); return true; }
    if (it == T.end())   { z_agl_out = Z.back();  return true; }
    size_t i = size_t(it - T.begin());
    size_t i0 = i - 1, i1 = i;
    double a = (t - T[i0]) / (T[i1] - T[i0]);
    z_agl_out = Z[i0] + a * (Z[i1] - Z[i0]);
    return true;
}
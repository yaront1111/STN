#include <iostream>
#include "terrain_provider.h"

int main() {
    TerrainProvider terrain;
    
    std::cout << "Testing terrain slopes:\n";
    for (double n = 0; n <= 3000; n += 1000) {
        for (double e = 0; e <= 2000; e += 1000) {
            auto ts = terrain.sample(n, e);
            double slope = ts.grad.norm();
            std::cout << "  (" << n << "," << e << "): "
                     << "h=" << ts.h << "m, "
                     << "slope=" << slope << " (" << slope*100 << "%)\n";
        }
    }
    
    // Check average slope over trajectory
    double total_slope = 0;
    int count = 0;
    for (double n = 0; n <= 6000; n += 10) {
        auto ts = terrain.sample(n, 0);
        total_slope += ts.grad.norm();
        count++;
    }
    std::cout << "\nAverage slope along trajectory: " 
              << (total_slope/count)*100 << "%\n";
              
    // Check max slope
    double max_slope = 0;
    for (double n = 0; n <= 6000; n += 10) {
        for (double e = -100; e <= 100; e += 10) {
            auto ts = terrain.sample(n, e);
            max_slope = std::max(max_slope, ts.grad.norm());
        }
    }
    std::cout << "Max slope in area: " << max_slope*100 << "%\n";
    
    return 0;
}
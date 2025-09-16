#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>

/**
 * @brief Filter Integrity Monitoring with Chi-Square Gating
 * 
 * Provides NEES/NIS computation, chi-square gating, and adaptive
 * measurement noise scaling for robust navigation filtering.
 */
class IntegrityMonitor {
public:
    struct Stats {
        double nees = 0.0;           // Normalized Estimation Error Squared
        double nis = 0.0;            // Normalized Innovation Squared  
        double nees_pass_rate = 1.0; // NEES chi-square test pass rate
        double nis_pass_rate = 1.0;  // NIS chi-square test pass rate
        
        // Running statistics
        std::vector<double> recent_nees;
        std::vector<double> recent_nis;
        static constexpr int WINDOW_SIZE = 100;
        
        void addNEES(double nees_val, int dof) {
            recent_nees.push_back(nees_val);
            if (recent_nees.size() > WINDOW_SIZE) {
                recent_nees.erase(recent_nees.begin());
            }
            
            // Chi-square 95% confidence test
            double chi2_95 = getChiSquare95(dof);
            bool pass = (nees_val < chi2_95);
            nees_pass_rate = 0.95 * nees_pass_rate + 0.05 * (pass ? 1.0 : 0.0);
            nees = nees_val;
        }
        
        void addNIS(double nis_val, int dof) {
            recent_nis.push_back(nis_val);
            if (recent_nis.size() > WINDOW_SIZE) {
                recent_nis.erase(recent_nis.begin());
            }
            
            // Chi-square 99% confidence test (more conservative for measurements)
            double chi2_99 = getChiSquare99(dof);
            bool pass = (nis_val < chi2_99);
            nis_pass_rate = 0.95 * nis_pass_rate + 0.05 * (pass ? 1.0 : 0.0);
            nis = nis_val;
        }
        
    private:
        double getChiSquare95(int dof) {
            // Chi-square 95% critical values for common DOF
            static const double chi2_95[] = {
                3.841, 5.991, 7.815, 9.488, 11.070, 12.592, 14.067, 15.507,
                16.919, 18.307, 19.675, 21.026, 22.362, 23.685, 24.996
            };
            if (dof <= 15) return chi2_95[dof-1];
            // Approximation for higher DOF
            return dof + 1.96 * sqrt(2.0 * dof);
        }
        
        double getChiSquare99(int dof) {
            // Chi-square 99% critical values for common DOF
            static const double chi2_99[] = {
                6.635, 9.210, 11.345, 13.277, 15.086, 16.812, 18.475, 20.090,
                21.666, 23.209, 24.725, 26.217, 27.688, 29.141, 30.578
            };
            if (dof <= 15) return chi2_99[dof-1];
            // Approximation for higher DOF  
            return dof + 2.576 * sqrt(2.0 * dof);
        }
    };
    
    /**
     * @brief Chi-square gating for measurement validation
     * @param innovation Innovation vector
     * @param S Innovation covariance
     * @param dof Degrees of freedom
     * @return true if measurement passes gate
     */
    static bool chiSquareGate(const Eigen::VectorXd& innovation, 
                             const Eigen::MatrixXd& S, 
                             int dof) {
        double nis = innovation.transpose() * S.inverse() * innovation;
        double threshold = getChiSquareThreshold(dof);
        return nis < threshold;
    }
    
    /**
     * @brief Compute adaptive noise scaling based on innovation statistics
     * @param innovation_history Recent innovation magnitudes
     * @return Adaptive scaling factor [0.1, 10.0]
     */
    static double computeAdaptiveNoiseScale(const std::vector<double>& innovation_history) {
        if (innovation_history.size() < 10) return 1.0;
        
        // Compute sample variance of innovations
        double mean = 0.0;
        for (double val : innovation_history) mean += val;
        mean /= innovation_history.size();
        
        double var = 0.0;
        for (double val : innovation_history) {
            var += (val - mean) * (val - mean);
        }
        var /= (innovation_history.size() - 1);
        
        // Expected innovation variance for well-tuned filter is ~1
        double scale = sqrt(var);
        return std::clamp(scale, 0.1, 10.0);
    }
    
    /**
     * @brief Check filter health based on NEES/NIS pass rates
     */
    static bool isFilterHealthy(const Stats& stats) {
        return stats.nees_pass_rate > 0.7 && stats.nis_pass_rate > 0.7;
    }
    
    static double getChiSquareThreshold(int dof) {
        // Use 95% confidence for more practical measurement gating
        if (dof == 1) return 3.841;
        if (dof == 3) return 7.815;
        if (dof == 6) return 12.592;
        if (dof == 9) return 16.919;
        // Approximation for higher DOF (95% confidence)
        return dof + 1.96 * sqrt(2.0 * dof);
    }
};

/**
 * @brief Robust measurement update with chi-square gating and adaptive noise
 */
template<int StateDim, int MeasDim>
struct RobustUpdate {
    using StateVec = Eigen::Matrix<double, StateDim, 1>;
    using MeasVec = Eigen::Matrix<double, MeasDim, 1>;
    using StateCov = Eigen::Matrix<double, StateDim, StateDim>;
    using MeasCov = Eigen::Matrix<double, MeasDim, MeasDim>;
    using MeasJac = Eigen::Matrix<double, MeasDim, StateDim>;
    
    struct Result {
        bool accepted = false;
        StateVec x_updated;
        StateCov P_updated;
        double nis = 0.0;
        double adaptive_scale = 1.0;
    };
    
    /**
     * @brief Perform gated measurement update with adaptive noise
     */
    static Result update(const StateVec& x_pred,
                        const StateCov& P_pred, 
                        const MeasVec& z_meas,
                        const MeasVec& z_pred,
                        const MeasJac& H,
                        const MeasCov& R_base,
                        std::vector<double>& innovation_history) {
        Result result;
        
        // Innovation and covariance
        MeasVec innovation = z_meas - z_pred;
        MeasCov S = H * P_pred * H.transpose() + R_base;
        
        // Adaptive noise scaling
        double adaptive_scale = IntegrityMonitor::computeAdaptiveNoiseScale(innovation_history);
        MeasCov R_adaptive = adaptive_scale * R_base;
        S = H * P_pred * H.transpose() + R_adaptive;
        
        // Compute NIS for diagnostics
        double nis = (innovation.transpose() * S.inverse() * innovation)(0,0);
        double threshold = IntegrityMonitor::getChiSquareThreshold(MeasDim);
        
        // Chi-square gating with diagnostics
        if (!IntegrityMonitor::chiSquareGate(innovation, S, MeasDim)) {
            std::cout << "Chi-square REJECT: NIS=" << nis << " > threshold=" << threshold 
                     << " (DOF=" << MeasDim << ", innovation_norm=" << innovation.norm() << ")\n";
            result.accepted = false;
            result.x_updated = x_pred;
            result.P_updated = P_pred;
            return result;
        }
        
        std::cout << "Chi-square ACCEPT: NIS=" << nis << " < threshold=" << threshold 
                 << " (DOF=" << MeasDim << ", innovation_norm=" << innovation.norm() << ")\n";
        
        // Kalman gain and update
        auto K = P_pred * H.transpose() * S.inverse();
        result.x_updated = x_pred + K * innovation;
        
        // Joseph form covariance update (numerically stable)
        auto I_KH = StateCov::Identity() - K * H;
        result.P_updated = I_KH * P_pred * I_KH.transpose() + K * R_adaptive * K.transpose();
        
        // Update innovation history
        innovation_history.push_back(innovation.norm());
        if (innovation_history.size() > 50) {
            innovation_history.erase(innovation_history.begin());
        }
        
        result.accepted = true;
        result.nis = (innovation.transpose() * S.inverse() * innovation)(0,0);
        result.adaptive_scale = adaptive_scale;
        
        return result;
    }
};
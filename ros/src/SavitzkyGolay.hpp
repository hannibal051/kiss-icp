#pragma once

#include <vector>
#include <deque>
#include <Eigen/Dense>

class SavitzkyGolay {
public:
    SavitzkyGolay() {}
    ~SavitzkyGolay() {}
    SavitzkyGolay(int window_size, int polynomial_order)
        : window_size_(window_size), polynomial_order_(polynomial_order) {
        computeCoefficients();
    }

    Eigen::Vector3d smooth(const std::deque<Eigen::Vector3d>& data) {
        int half_window = window_size_ / 2;
        Eigen::Vector3d smoothed_point = Eigen::Vector3d::Zero();
        
        // Ensure we have enough data points to smooth
        if (data.size() < window_size_) {
            return data.back(); // Return the latest data point if not enough data
        }

        for (int j = -half_window; j <= half_window; ++j) {
            int index = static_cast<int>(data.size()) - 1 + j;
            smoothed_point += coeffs_(half_window + j) * data[index];
        }
        
        return smoothed_point;
    }

    int getWindowSize() const {
        return window_size_;
    }

private:
    int window_size_;
    int polynomial_order_;
    Eigen::VectorXd coeffs_;

    void computeCoefficients() {
        int half_window = window_size_ / 2;
        Eigen::MatrixXd A(window_size_, polynomial_order_ + 1);

        for (int i = -half_window; i <= half_window; ++i) {
            for (int j = 0; j <= polynomial_order_; ++j) {
                A(half_window + i, j) = std::pow(i, j);
            }
        }

        Eigen::MatrixXd AtA = A.transpose() * A;
        Eigen::MatrixXd AtA_inv = AtA.inverse();
        Eigen::MatrixXd At = A.transpose();
        coeffs_ = AtA_inv * At.row(half_window).transpose();
    }
};
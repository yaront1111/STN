#pragma once
#include <Eigen/Dense>

struct IIR1 {
  double y{0.0};
  bool   inited{false};
  double alpha{0.1}; // dt/tau ~ 0.01/1.0 => 1s time-constant (set at runtime)

  double step(double x) {
    if(!inited){ y=x; inited=true; return y; }
    y = y + alpha*(x - y);
    return y;
  }
};
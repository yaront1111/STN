#pragma once
#include "types.h"

struct StrapdownParams {
  double dt{0.01};   // s
  double g{9.80665}; // m/s^2 (down positive in NED)
};

class StrapdownINS {
public:
  explicit StrapdownINS(const StrapdownParams& p): P(p) {}
  void propagate(State& x, const ImuSample& z);
private:
  StrapdownParams P;
};

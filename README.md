# STN v0.1 — Space-Time Navigation Prototype

A minimal proof-of-concept navigation stack that fuses strapdown INS with simple EKF positioning and (optionally) terrain-referenced navigation (TRN) via a synthetic radar-altimeter. Phase 1 is **software-only**, under $1k in tooling.

---

## Contents

- **C++ Core**
  - `cpp/core/ins.{h,cpp}` — strapdown INS (NED, Down positive)
  - `cpp/core/ekf.{h,cpp}` — tiny EKF with position updates
  - `cpp/core/gravity_model.{h,cpp}` — normal gravity + free-air correction
  - `cpp/core/terrain_provider.{h,cpp}` — synthetic terrain (height + gradient)
  - `core/main.cpp` — end-to-end demo: CSV in → CSV out
- **Python Tools**
  - `python/sim/run_sim.py` — generate IMU + truth
  - `python/sim/add_radalt.py` — generate radar-altimeter from truth + terrain
  - `python/visualization/metrics.py` — CEP/RMSE, quick metrics
  - `python/visualization/plot_results.py` — Plotly 3D truth vs estimate
- **Data**
  - `data/` — CSVs (`sim_imu.csv`, `sim_truth.csv`, `radalt.csv`, `run_output.csv`)

---

## Quick Start

### 1) Python: generate simulation data

Create a venv (optional) and install deps:

```bash
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -U pip
pip install numpy pandas plotly
```

Generate IMU + truth (120 s @ 100 Hz → 12,000 rows):

```bash
python python/sim/run_sim.py
# Outputs: data/sim_imu.csv, data/sim_truth.csv
```

Generate synthetic radar-altimeter consistent with the C++ terrain:

```bash
python python/sim/add_radalt.py
# Outputs: data/radalt.csv
```

> **Notes**
>
> * **Frames:** NED with Down positive. In straight & level, the accelerometer measures **specific force**, so `az ≈ -g`.
> * Terrain parameters in Python and C++ are matched (A1/L1/A2/L2). Keep them in sync.

### 2) C++: build the demo

Install Eigen:

* **Ubuntu/Debian:** `sudo apt-get install -y libeigen3-dev`
* **macOS (Homebrew):** `brew install eigen`
* **Windows (vcpkg):** `vcpkg install eigen3` (and hook vcpkg to CMake)

Configure & build:

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j
```

Binary: `build/stn_demo`

### 3) Run

```bash
# From repo root (after building)
./build/stn_demo data/sim_imu.csv data/run_output.csv
# Uses data/radalt.csv automatically (if present)
```

Expected console (example):

```
Loaded 12000 radalt measurements
Wrote data/run_output.csv (TRN updates: 120)
```

### 4) Inspect results

```bash
# Quick metrics (CEP, RMSE, etc.)
python python/visualization/metrics.py

# 3D truth vs estimate (opens in browser)
python python/visualization/plot_results.py --renderer=browser
```

---

## Data Formats

* **IMU CSV** — `data/sim_imu.csv`

  * Columns: `t, ax, ay, az, gx, gy, gz`
  * Units: `a*` in m/s² (specific force), `g*` in rad/s (body rates)
  * Sampling: typically 100 Hz (dt ≈ 0.01 s)

* **Truth CSV** — `data/sim_truth.csv`

  * Columns: `t, pn, pe, pd, vn, ve, vd`
  * Frame: NED; **Down positive** (so altitude Up = `-pd`)

* **Radar Altimeter CSV** — `data/radalt.csv`

  * Columns: `t, agl_m` (Above Ground Level, meters)
  * Derived from truth + synthetic terrain; noisy by default (σ≈0.5 m)

* **Output CSV** — `data/run_output.csv`

  * Columns: `t, pn, pe, pd, vn, ve, vd` (EKF estimate)

---

## How It Works (Phase 1)

1. **Strapdown INS** integrates IMU at 100 Hz; gravity is added in NED with Down positive.
2. **EKF** maintains a small covariance; position is updated when measurements are available.
3. **TRN (optional)**: at ~1 Hz, a position pseudo-measurement is formed from radar AGL + local terrain slope (better slope ⇒ better observability). Gravity-likelihood is scaffolded but **off by default**.

**Toggles / parameters (current state)**

* `USE_GRAVITY_LIKELIHOOD` (in `core/main.cpp`) — `false` by default.
* TRN update rate decimated by `DECIM = int(1.0/dt)` with `dt=0.01`.
* Terrain parameters: `A1=100, L1=2000, A2=50, L2=800` in both C++ and Python.

> **Planned improvement:** move these to CLI flags (see "Roadmap").

---

## Performance (with TRN enabled)

After implementing proper Kalman filtering and TRN fixes:

* **2D CEP50:** 3.6 m (was 2995 m without TRN)
* **2D CEP95:** 9.2 m (was 5707 m without TRN) 
* **Final 2D error:** 6.0 m (was 6011 m without TRN)
* **Vertical stability:** <0.3 m error maintained
* **Improvement:** ~1000x reduction in horizontal error

---

## Troubleshooting

* **CMake can't find Eigen**

  * Install it (see above). On macOS, ensure Homebrew's CMake toolchain sees `/opt/homebrew/include/eigen3`.
* **"Loaded 0 radalt measurements"**

  * Run `python python/sim/add_radalt.py` first, and ensure timestamps overlap with IMU.
* **Gravity sign / blowing up vertically**

  * Remember: NED, **Down positive**. In level flight, `az ≈ -9.81`. If your IMU has +Up conventions, convert before feeding.
* **No TRN updates**

  * TRN is gated (slope/AGL) and decimated to ~1 Hz. On a perfectly flat trajectory, expect few or no updates.
* **Weird paths in Plotly**

  * Ensure `data/run_output.csv` and `data/sim_truth.csv` have the same length (the metric script clamps to min(N)).

---

## Roadmap (Phase 1 → Phase 2)

* [ ] CLI flags: `--imu`, `--radalt`, `--out`, `--use-gravity-like`, `--trn-rate`, `--terrain-params`.
* [ ] Unit tests: transform sanity (NED sign), small-angle quaternion update, EKF position update against synthetic measurements.
* [ ] Compute `dt` from input timestamps (remove hard-coded 0.01 in `main.cpp` decimator).
* [ ] Package Python (`requirements.txt`) and add `Makefile` helpers.
* [ ] Separate library target for `cpp/core` + real tests (currently `stn_tests` is a placeholder).
* [ ] Add real terrain data support (SRTM tiles)
* [ ] Implement batch correlation for TRN bootstrap
* [ ] Add Coriolis/transport rate corrections for longer flights

---

## Dev Notes

* Coordinate frame: **NED** (North, East, Down). Altitude Up = `-pd`.
* Gravity: normal gravity model + free-air correction (`gravity_model.cpp`).
* Terrain: simple analytic height field (header), gradient used to shape TRN measurement covariance.
* EKF: Diagonal implementation with separate position covariance tracking
* TRN: Gradient-based corrections with adaptive step clamping (50m → 10m → 3m)

---

## License

MIT (TBD)

---

## Contributors

* Initial implementation: STN v0.1 team
* Phase 1 optimizations: Community contributions welcome!
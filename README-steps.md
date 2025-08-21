Absolutely—here’s a clean, “copy-paste ready” version of your **STN (Space-Time Navigation)** phases. It captures everything we built and learned, plus crisp goals, success criteria, and deliverables per phase.

# STN Program Plan (v0.1 → v1.0)

## Vision (1-liner)

A navigation stack that uses **inertial dead-reckoning** corrected by **spacetime signals**—gravity, time, and terrain—so vehicles can navigate **without GNSS**.

## Core Hypothesis

Local spacetime features (gravity magnitude/gradient, clock drift, terrain) provide enough **absolute** information to bound INS drift. With proper filtering (EKF → factor graph), you can converge to meter-level accuracy in GNSS-denied environments.

---

## Phase 0 — Foundations (done / ongoing)

**Goal:** Get a reproducible, testable baseline.

* ✅ C++ strapdown INS (specific force + gravity, small-angle quaternion; NED frame)
* ✅ Hardened vertical channel (no free-fall; <0.3 m error over 120 s)
* ✅ Synthetic sensor sim + truth (Python)
* ✅ Plot & metrics (CEP50/95, RMSE, vertical stability)
* ✅ TRN prototype (terrain-referenced nav) with synthetic terrain + AGL sensor
* ✅ EKF corrected (proper covariance; removed 0.1 attenuation; per-sample `dt`; AGL interpolation)
* ✅ Massive horizontal improvement (∼6 km → ∼6 m in 120 s)

**Artifacts:**
`ins.cpp`, `ekf.cpp`, `gravity_model.*`, `terrain_provider.*`, `sim/*.py`, `visualization/*.py`
**Success metric:** Stable vertical (<0.3 m @ 120 s) + 2D CEP50 < 5 m on synthetic TRN track.

---

## Phase 1 — Software-Only MVP (current)

**Goal:** Prove the physics + filtering in software, on public/synthetic data, repeatably.

### Scope

* Simulation tracks: straight & level, gentle turns, climbs/descents
* Sensors: IMU (noise + bias), mems-like specs; AGL (radar alt sim); optional baro
* Aids:

  1. **TRN (Terrain-Referenced Navigation)** via AGL + terrain gradient
  2. **Gravity-likelihood (vertical)** gated, low-pass, 1 Hz
* Filters: current **diagonal EKF** (position update); prep interface for **factor graph** (GTSAM)

### Deliverables

* Reproducible runs: `make run` → sim + fuse + plots + metrics
* Metrics report: CEP50/95, RMSE, time-series of errors; plot truth vs INS vs aided
* Unit tests: transforms, gravity signs, stationary sanity, TRN sign (residual vs gradient)
* Optional: Coarse-capture TRN (2-D correlation bootstrap) for large initial errors

### Success Criteria

* **Vertical**: < 0.5 m final error @ 120–300 s (no gravity-aid bias)
* **Horizontal (TRN)**: CEP50 ≤ 5–10 m on varied synthetic terrain; robust to ±1 km initial error after coarse bootstrap
* **Stability**: No step artifacts after easing (slew-limited or velocity-nudge TRN)

### Risks & Mitigations

* **Mis-gating / over-correction:** strict quasi-static gates + low-pass; 0.5–1 Hz cadence
* **Poor observability on flat terrain:** slope/roughness gating; coarse-capture bootstrap
* **Numerics:** per-sample `dt`; covariance inflation on large residuals

### Immediate Next Tasks

* [ ] Add **TRN easing** (slew-limit or small VE/VN nudges instead of position jumps)
* [ ] Add **coarse-capture bootstrap** (10–20 s AGL window, ±10 km 2-D sweep, pick best ΔN/ΔE)
* [ ] Unit tests for TRN sign & observability gating
* [ ] One-command runner (`main.py` / `Makefile`) + README walkthrough
* [ ] Optional: switch diagonal EKF → **proper 3×3 Kalman** with `P_pos_` (if not already)

---

## Phase 2 — Bench & Hardware-in-the-Loop (HIL)

**Goal:** Replace simulated sensors with **real signals** in the lab / vehicle, keep software unchanged.

### Scope

* Hardware: **tactical-grade IMU** (tactical MEMS or entry FOG), **CSAC** (for timebase), multi-band GNSS for ground truth
* HIL: feed recorded IMU/AGL to the stack; later real-time serial stream
* Gravity model: EGM2008/2020 normal gravity; later grid lookup for regional anomaly
* Terrain: SRTM 1″ tiles (GeoTIFF → lightweight grid) with bilinear interp + gradients

### Deliverables

* Data recorder (synchronization + formats)
* Real-world datasets (city drive, highway, hill region)
* Reproducible replays producing the same plots/metrics as Phase 1

### Success Criteria

* **Short drives** (5–15 min): CEP50 ≤ 10–20 m with TRN only; vertical < 1 m
* **Robustness**: works day/night, moderate dynamics, different terrains

### Budget (rough, from earlier):

* IMU: \$12–35k (tactical MEMS → entry FOG)
* CSAC: \$3–8k
* Misc I/O + compute + mounts: \$2–5k
* Test time (table rental/logistics): \$3–8k
  **Target total:** \~\$25k–\$75k depending on sensor choice

---

## Phase 3 — Vehicle Demonstrator

**Goal:** Prove **GPS-denied navigation** in a representative vehicle (car/UAV) over longer routes.

### Scope

* Integrate sensors on a vehicle; collect data over varied terrain
* Filters: introduce **factor graph** (GTSAM) for smoothing (back-end), keep EKF as front-end
* Aids: TRN (primary), gravity vertical (secondary), add **baro + wheel-odo** (car) or **airspeed** (UAV)

### Deliverables

* Demo runs & videos; comparison against GNSS truth logs
* Report: drift rates with/without aids; ablation study (TRN only, gravity only, both)

### Success Criteria

* **10–30 min routes:** CEP50 ≤ 15 m; vertical ≤ 2 m; smooth tracks, no steps
* Resilience to outages/turns; graceful degradation on flat terrain

---

## Phase 4 — Advanced Spacetime Aids (R\&D)

**Goal:** Push beyond TRN with genuine spacetime measurements.

* **Gravity anomalies/gradients:** regional grids (e.g., EGM + residual maps); explore cold-atom gradiometer data where available
* **Timing:** CSAC-disciplined inertial timing; prototype **clock-based constraints** in factor graph
* **Map:** toward a shared **Spacetime Map** (gravity, terrain, timing metadata)

Success = measurable reduction of drift **without** terrain, or in feature-poor regions.

---

## Program-Level Metrics (track each run)

* 2D CEP50 / CEP95, 3D RMSE, final vertical error
* Drift rates (% of distance / time) with/without each aid
* Update usage: gated % for TRN/gravity, average correction size
* Robustness: performance vs slope, speed, turns, noise levels

---

## Tech Stack (current & near-term)

* **Core:** C++17, Eigen, (GTSAM planned), CMake
* **Sim & Viz:** Python 3.11, NumPy/Pandas, Plotly (browser renderer), Matplotlib (optional)
* **Data:** CSV now; GeoTIFF → internal grid for SRTM later
* **Testing:** CTest + simple gtests; property tests for transforms

---

## “Do-Now” Checklist (to finish Phase 1 strong)

* [ ] **Slew-limit TRN updates** (or convert to velocity nudges) to remove visible steps, keep meter-level accuracy
* [ ] **Coarse-capture TRN** 2-D correlation to handle km-scale initial errors
* [ ] **AGL/terrain time alignment** (already added linear interp; keep it)
* [ ] **Per-sample `dt`** (done; keep guardrails/clamps)
* [ ] **Unit tests**: stationary, gravity sign, TRN residual direction, transform round-trips
* [ ] **One-command run** (`make run` / `python main.py`) + README
* [ ] **Metrics report template** (Markdown export of CEP/RMSE + figures)

If you want, I can generate:

1. a tiny **TRN “velocity-nudge” patch** (smooth state evolution, no steps), and
2. a **bootstrap\_trn.py** (grid search ±10 km, 250 m spacing, picks ΔN/ΔE) to snap you close before EKF tracking.

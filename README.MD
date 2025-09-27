Here’s a tightened, investor-ready rewrite that keeps **vision optional**, emphasizes **available hardware**, and stays laser-focused on **GPS-denied air/ground/maritime** with a clear path to **≤50 m CEP90** (stretch **10–20 m CEP50** with SoOP/beacons). I’ve also added explicit **modes** for drones and vessels, and kept gravity sensing as an R&D lane (not on the MVP critical path).

---

# AION — All-Weather Integrated Opportunistic Navigation (GPS-Free)

> **Mission:** Deliver reliable **position, navigation, and timing (PNT)** without GNSS for **drones, vehicles, and vessels** in contested environments.
> **Targets:** **≤50 m CEP90** (typical), **10–20 m CEP50** with beacons/SoOP, 100–200 Hz state rate, **all-weather/day-night**.
> **Why now:** Designed for rapid fielding in Israel and allies; modular, exportable, investable.

---

## 0) TL;DR

* **Backbone:** High-rate **error-state SR-UKF INS** with fixed-lag **factor-graph** fusion.
* **All-weather odometry:** **Radar odometry (RO)** from mmWave/automotive or marine radar.
* **Absolute clamps:** **TRN** (radar-alt + DEM slope) over land; **SoOP/STN** (LEO/terrestrial/5G/UWB) anywhere signals exist.
* **Integrity:** RAIM-like gating, **NEES/NIS**, mode manager, and optional **CSAC** time holdover.
* **Vision is optional:** Event/thermal VIO or LiDAR odometry only when it helps; otherwise off.

---

## 1) Value Proposition

* **GPS-independence with graceful degradation** — multiple heterogeneous aids; no single point of failure.
* **All-weather, day/night** — radar + TRN/SoOP don’t care about rain, fog, smoke, or darkness.
* **Air / Ground / Maritime** — one stack, profile-driven modes.
* **Product-ready foundation** — bounded compute/memory, deterministic timing, binary logs, metrics & integrity outputs.

**Performance Goals**

* **CEP90 ≤ 50 m** over 30–40 min legs (mixed terrain, 100–1500 m AGL; highways; coastal runs).
* **CEP50 10–30 m** when TRN+SoOP or beacon geometry is good.
* **Holdover:** sub-1% drift/km with RO+INS; **time** stable with PPS/PTP (CSAC optional).

---

## 2) System Overview

```
Sensors             Front-Ends                      Fusion Core                     Integrity / Outputs
┌─────────────┐   ┌──────────────────┐        ┌───────────────────────┐        ┌────────────────────────┐
│ IMU 200 Hz  ├──▶│ IMU Preintegr.   │───┐    │ SR-UKF (200 Hz)       │◀──────▶│ NEES/NIS, χ² gating     │
│ mmWave/Marine│▶ │ Radar Odom (RO)  │   │    │ Fixed-Lag Smoother    │   ┌───▶│ Mode mgr (NOMINAL/… )   │
│ Radar 10–20 │   │ TRN (DEM+AGL)    │   ├───▶│ (factor graph 20–60Hz)│   │    └───────────┬───────────┘
│ RAlt 5–50   │   │ SoOP/STN (TDOA/R)│   │    └─────────┬────────────┘   │                │
│ SoOP/UWB    │   │ Baro/Mag priors  │───┘        Soft-reset (1–5 Hz)   │        Outputs  │
│ (opt) VIO/LO│                                            ▲             └────▶ PVT, Cov, Integrity
└─────────────┘                                            └──────────────▶  Time (PTP/CSAC), Logs
```

**Key ideas**

* **Opportunistic fusion:** any subset of aids; each contributes with its **σ, DOP/SNR, validity**.
* **Two-stage:** front-ends → robust factors → **graph**; **UKF** runs the tight loop and is reconciled to graph via **soft resets**.
* **Environment-aware gating:** DEM slope, radar SNR, SoOP DOP, etc.

---

## 3) Operational Modes (profiles)

* **AIR-TRN/RO-SoOP:** RO + TRN (radar-alt + DEM) as primary; SoOP fixes when present; vision optional.
* **GROUND-RO-SoOP:** Automotive 77 GHz RO + SoOP/UWB; TRN disabled unless ralt present.
* **MARITIME-RO-COAST-SoOP:** Marine radar odometry + coastline TRN (chart/shoreline) + SoOP; ralt off; vision optional.

> Flat terrain/water? TRN auto-gates off by slope; stack leans on RO + SoOP/UWB and baro priors.

---

## 4) Hardware Snapshot (MVP, available today)

| Subsystem             | Example / Range                                           | Notes                                  |
| --------------------- | --------------------------------------------------------- | -------------------------------------- |
| **IMU**               | Industrial MEMS (3–5 °/hr gyro, 0.05–0.1 m/s² accel bias) | 200 Hz; rigid mount, temp-comp         |
| **Radar (RO)**        | 77 GHz FMCW **or** X-band marine                          | Robust odometry in all weather         |
| **Radar/Laser Alt**   | 2–1500 m AGL                                              | TRN scalar clamp over land             |
| **SoOP/STN**          | LEO devkit, 5G OTDOA, terrestrial beacons, **UWB**        | Absolute fixes/TDOA; cooperative ready |
| **Barometer/Magneto** | Weather-grade baro; low-bias mag                          | Baro altitude prior; mag yaw (robust)  |
| **Clock**             | PPS/PTP; **CSAC** optional                                | Timing holdover/spoof resistance       |
| **Compute**           | Jetson Orin NX **or** i7 NUC                              | Fanless if airborne                    |
| **(Optional) Vision** | Event or thermal; **LiDAR** 16–32ch                       | Only when it helps                     |

> **R&D lane (future):** **Gravity-aided** navigation (tensor/gradiometry) as an add-on for **specialized platforms** (not required for MVP).

---

## 5) Software Architecture

* **Threads:** per-sensor drivers → lock-free queues; front-ends (RO/TRN/SoOP/UWB/Baro) → normalized factors; fusion core (UKF+graph); integrity/mode; logging.
* **State (error-state UKF):** `x=[p v q b_g b_a s_a (clock_bias, clock_drift) (wind opt.)]`.
* **Models:**

  * **RO:** scan-to-scan Δpose + 6×6 cov (radar power image/point cloud alignment).
  * **TRN:** residual `z_ralt − (z_NED − DEM(x,y))`, gated by **slope** and validity.
  * **SoOP/STN:** TDOA/TOA pseudorange/time; **clock bias/drift** in state; RAIM-like checks.
  * **Baro/Mag:** priors with robust losses.
* **Integrity:** per-factor χ²; rolling **NEES/NIS**; **consistency score S∈[0,1]**; degraded modes explicit.

---

## 6) Data Rates & Budgets

| Module            |     Rate | Latency Budget |      CPU | Notes              |
| ----------------- | -------: | -------------: | -------: | ------------------ |
| INS mechanization |   200 Hz |          <2 ms |      low | tight loop         |
| Radar Odometry    | 10–20 Hz |         <50 ms |      med | GPU optional       |
| TRN               |  5–50 Hz |         <10 ms |      low | scalar update      |
| SoOP/UWB          | 0.1–2 Hz |        <500 ms |      low | absolute fixes     |
| Baro/Mag          |  5–50 Hz |         <10 ms |     tiny | priors only        |
| (opt) VIO/LO      | 20–60 Hz |      <30–70 ms | med-high | off unless helpful |

**Targets:** <500 MB RAM (MVP), <50% Orin NX CPU at 15 Hz RO and 200 Hz INS.

---

## 7) Public Interfaces

### 7.1 Outputs (conceptual)

```proto
message NavSolution {
  double time_s;
  double lat, lon, alt_msl;     // or NED relative to origin
  double vn, ve, vd;
  quaternion q_ned_body;
  double cov[36];               // 6x6 upper-tri pos/vel
  double integrity;             // 0..1
  string mode;                  // NOMINAL / TRN_ONLY / RO_SOOP / DEGRADED / MARITIME
}
```

### 7.2 Config (YAML excerpt)

```yaml
common:
  frame: NED
  origin: { lat: 32.0853, lon: 34.7818, alt_m: 20 }
  gravity_model: wgs84

ukf:
  rate_hz: 200
  include_clock_states: true
  proc_noise:
    gyro_rw: 0.02
    acc_rw:  0.03
    bg_rw:   0.0005
    ba_rw:   0.0005
    clk_rw:  1.0e-9   # sec/√Hz (bias)
    clkd_rw: 1.0e-11  # sec/s/√Hz (drift)

radar_odom:
  enabled: true
  rate_hz: 15
  robust_loss: huber

trn:
  enabled: true
  dem_path: data/dem/srtm
  min_slope_deg: 0.5
  chi2_gate_1d: 6.63

soop:
  enabled: true
  providers: [leo, terrestrial, 5g, uwb]
  min_beacons: 4
  sigma_tdoa_s: 2.0e-7
  beacons_file: configs/stn/beacons.yaml

baro: { enabled: true, sigma_m: 2.0 }
mag:  { enabled: true, sigma_deg: 5.0 }

integrity:
  nees_window_s: 10
  nis_window_s: 10
```

---

## 8) Build & Run

**Deps:** C++20, CMake ≥3.22, Eigen, Sophus/manif, yaml-cpp, spdlog, abseil, (optional) GTSAM, OpenCV, CUDA.

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Example: Air profile (TRN+RO, SoOP on)
./aion_node --config configs/aion_air.yaml

# Ground profile (RO+SoOP, no ralt)
./aion_node --config configs/aion_ground.yaml

# Maritime profile (marine RO + coastline TRN + SoOP)
./aion_node --config configs/aion_maritime.yaml
```

---

## 9) Calibration & Time

* **Lever arms & boresight:** rigid plate; jig + plane/board procedures for radar/camera/LiDAR; store in YAML.
* **IMU:** 6-face static + gentle motion; temp curves.
* **RAlt:** set range gates; verify against DEM on a survey line.
* **Time:** PPS/PTP preferred; log `t_meas` and `t_recv`; optional **CSAC** for holdover.

---

## 10) Test Plan (first 90 days)

**A. Ground (Weeks 1–3)** — Roof rig; day/night/rain. Deliver **RO+INS drift <1%/km** and stable integrity.
**B. TRN (Weeks 4–6)** — Add ralt; mixed terrain loops; **CEP90 25–40 m** over hills; auto-gate on flats.
**C. SoOP (Weeks 7–9)** — Integrate one LEO/5G/terrestrial source + UWB option; validate DOP & **soft-reset** behavior.
**D. Air/Maritime (Weeks 10–12)** — Small UAS at 300–1000 m AGL (25–40 m/s) and/or coastal vessel runs; **≤50 m CEP90** on 30–40 min legs.

**KPIs (1 Hz log):** CEP50/CEP90, RO drift %, TRN inliers vs slope, SoOP fix rate/DOP, NEES/NIS pass %, integrity S, mode histogram.

---

## 11) Failure Modes & Degraded Ops

* **Flat terrain/water:** TRN off → rely on RO + SoOP/UWB + baro prior.
* **Smoke/night/fog:** vision off; radar unchanged.
* **SoOP dropouts:** INS+RO holdover; inflate cov; flag **DEGRADED**; resume when DOP improves.
* **Radar ghosts/speckle:** robust loss + ego-motion sanity checks; temporary gating.

---

## 12) Security, Safety, Compliance

* **Integrity first:** consumers must honor **integrity** and **mode** flags for autonomy gates.
* **Logging:** binary + JSON summaries for incident replay/cert evidence.
* **Anti-spoof posture:** cross-check modalities; diverse SoOP providers; CSAC time hardening.
* **Certification path:** start as **Advisory Nav**; staged DO-178C/DO-254 plan; partitioned compute is possible.

---

## 13) Repo Layout

```
aion/
├── apps/aion_node.cpp
├── configs/ (air|ground|maritime).yaml, stn/beacons.yaml
├── core/    fusion/ graph/ integrity/ utils/
├── frontends/ radar_odom/ trn/ soop/ uwb/ (vio_event/ lidar_odom optional)
├── sensors/  imu/ radar/ ralt/ baro_mag/ (camera/ lidar optional)
├── msgs/     (gRPC/ROS2)
├── tools/    replay/ metrics/
├── tests/    unit + scenario
└── docs/     design/*.md
```

---

## 14) Roadmap (12 months)

* **Q1:** Field demos (air/ground/maritime), investor pilots, integrator SDK.
* **Q2:** More SoOP providers (5G OTDOA, terrestrial beacons), UWB mesh; desert & urban canyons.
* **Q3:** Redundant HW, maritime-specific coastline TRN, mining/ports deployments.
* **Q4:** Production HW spin, certification artifacts, first contracts.

---

## 15) Glossary

* **TRN:** Terrain-Referenced Navigation (DEM + AGL).
* **SoOP/STN:** Signals of Opportunity / terrestrial timing-ranging (LEO/5G/UWB).
* **SR-UKF:** Square-Root Unscented Kalman Filter.
* **NEES/NIS:** Estimator consistency metrics.

---

## License

© AION Navigation. Third-party components retain their licenses.

---

### Notes on Gravity (R&D)

Gravity-aided (including **gradiometry**) is compelling for **long-range** and **feature-poor** regions, but the **sensor availability, size/power, and calibration burden** put it outside the MVP. We keep a clean measurement interface so a gravity factor (map-matching or gradients) can drop into the graph later without touching the rest of the stack.

---

If you want, I can also generate **profile configs** (`aion_air.yaml`, `aion_ground.yaml`, `aion_maritime.yaml`) and a sample `configs/stn/beacons.yaml` you can commit immediately.

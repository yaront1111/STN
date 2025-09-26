# AION — All‑Weather Integrated Opportunistic Navigation (GPS‑Free)

> **Purpose**: Deliver reliable **position, navigation, and timing (PNT)** without GNSS in **all weather**, day/night, across air/ground domains. Target **≤50 m CEP90** in most environments; stretch **≤10–20 m CEP50** with beacons/SoOP. Real‑time on embedded compute.

---

## 0) TL;DR

* **Backbone**: High‑rate **error‑state SR‑UKF** INS.
* **All‑weather odometry**: **77 GHz radar odometry (RO)** + **TRN** (radar‑altimeter + DEM slope) for absolute clamps.
* **Absolute fixes**: **Signals of Opportunity (SoOP)** (LEO/Terrestrial/5G) + **cooperative UWB** when available.
* **Condition‑adaptive**: **Event/thermal VIO** or **LiDAR odometry** when vision helps; auto‑gated.
* **Integrity & holdover**: RAIM‑like gating, NEES/NIS health, optional **CSAC** timing.
* **Fusion**: **Fixed‑lag smoother** (factor graph) + **SR‑UKF** soft‑reset reconciliation.

---

## 1) Value Proposition

* **GPS‑independence** with **graceful degradation**: multiple heterogeneous aids; no single point of failure.
* **All‑weather, day/night**: radar + TRN don’t care about rain/fog/smoke; event/thermal handle HDR darkness.
* **Portable & modular**: drop‑in for **small aircraft, UAS, ground robots, mining/ports**.
* **Product‑ready**: bounded compute/memory, logging, calibration, and integrity metrics for certification path.

**Initial Targets**

* **CEP90 ≤ 50 m** for 30–40 min legs (mixed terrain, 100–1500 m AGL).
* **CEP50 10–30 m** where SoOP + TRN + VIO are healthy.
* **Update rate**: 100–200 Hz state, 10–60 Hz aiding.

---

## 2) System Overview

```
Sensors (pub)             Front‑Ends                 Fusion Core                 Integrity/Outputs
┌────────────────┐   ┌─────────────────┐      ┌─────────────────────┐       ┌──────────────────────┐
│ IMU 200 Hz     ├──▶│ IMU Preintegr.  │──┐   │  SR‑UKF (error‑state)│◀────▶│ Health/Integrity     │
│ Radar 10–20 Hz │──▶│ Radar Odom (RO) │  │   │  200 Hz mechanization│      │ NEES/NIS/χ² gating   │
│ RAlt 5–50 Hz   ├──▶│ TRN (DEM+AGL)   │  ├──▶│  Fixed‑Lag Smoother  │──┐   │ Mode/Gating Manager  │
│ Camera 20–120Hz│──▶│ VIO/Event/Therm │  │   │  (factor graph)      │  │   └─────────┬───────────┘
│ LiDAR 10–20 Hz │──▶│ LiDAR Odom      │  │   └─────────┬───────────┘  │             │
│ SoOP 0.1–2 Hz  ├──▶│ SoOP Ranging/T  │  │             │              │     Outputs  │
│ UWB 1–10 Hz    ├──▶│ Coop. Ranging   │  │       Soft‑reset           │   ┌───────────▼───────────┐
│ Baro/Mag       └──▶│ Baro/Mag Priors │──┘    reconciliation          └──▶│ PVT, Cov, Integrity    │
└────────────────┘   └─────────────────┘                                │   │ NMEA, ROS2/gRPC, Log  │
                                                                       └──▶│ Time (PTP/CSAC)       │
                                                                           └──────────────────────┘
```

### Key Concepts

* **Opportunistic fusion**: use any/a subset of aids; each carries its own **covariance & validity**.
* **Two‑stage fusion**: front‑ends build robust relative/absolute measurements; **fixed‑lag graph** fuses them; **SR‑UKF** runs high‑rate dynamics and is reconciled to the graph at 1–5 Hz via **soft resets**.
* **Environment‑aware gating**: DEM slope, radar SNR, VIO inliers, SoOP DOP; the system decides what to trust.

---

## 3) Hardware Snapshot (MVP BOM)

| Subsystem              | MVP Choice                                                           | Notes                                      |
| ---------------------- | -------------------------------------------------------------------- | ------------------------------------------ |
| IMU                    | Industrial MEMS (e.g., 3–5 °/hr gyro bias, 0.05–0.1 m/s² accel bias) | 200 Hz, temp‑comp; rigid mount             |
| mmWave Radar           | 77 GHz FMCW (automotive‑class)                                       | RO front‑end; robust to weather            |
| Radar/Laser Altimeter  | 2–1500 m AGL                                                         | TRN scalar constraint                      |
| Camera (Option A)      | Global‑shutter RGB or **event camera**                               | Event cam for HDR/high‑speed/night         |
| Thermal Cam (Option B) | LWIR                                                                 | Thermal‑inertial odom in darkness/fog      |
| LiDAR (Option C)       | 16–32 ch                                                             | Alternative to camera; good in low‑texture |
| SoOP                   | LEO (e.g., STL dev), terrestrial beacon, or 5G positioning dev kit   | Absolute fixes, timing                     |
| UWB (opt.)             | TDoA/Two‑way ranging                                                 | Cooperative constraints                    |
| Clock                  | PPS/PTP; **CSAC** optional                                           | Timing holdover/spoof resistance           |
| Compute                | Jetson Orin / x86 i7 + small GPU                                     | Fanless if airborne                        |

**Mounting**: co‑locate IMU + radar + alt antenna + vision on a rigid plate; measure **lever arms** and **boresight** precisely; isolate vibration.

---

## 4) Software Architecture

### 4.1 Processes & Threads

* **Sensor Drivers** (one thread each): timestamped pubs → lock‑free ring buffers.
* **Front‑Ends** (RO, TRN, VIO/LO, SoOP, UWB): convert raw data → normalized measurement factors with covariance & quality flags.
* **Fusion Core**:

  * **SR‑UKF @200 Hz**: mechanization (strapdown), bias states, wind (optional), scale factors.
  * **Fixed‑Lag Smoother @20–60 Hz**: factor graph (IMU preintegration, RO/VIO/LO factors, TRN pseudo‑range, SoOP ranges/timing).
  * **Reconciliation @1–5 Hz**: apply graph posterior to UKF (**soft reset** on position/velocity/attitude/bias window).
* **Integrity & Mode Manager**: χ²/NIS/NEES tests; environment classifiers (DEM slope, radar SNR, feature count, SoOP DOP); sensor fault isolation; mode switching (e.g., TRN‑only / RO‑SoOP).
* **Logging/Telemetry**: binary log + JSON summaries; replay tool.

### 4.2 State Vector (typical error‑state UKF)

```
x = [ p(3) v(3) q(4) b_g(3) b_a(3) s_a(3) wind(2) ... ]  (customizable)
```

* **Process model**: standard INS with coning/sculling comp; random‑walk biases; optional wind for small UAV.

### 4.3 Measurement Models (high‑level)

* **TRN (radar‑alt + DEM)**

  * Residual: r = z_ralt − (z_NED − DEM(x,y))
  * Jacobian: H = [−∂DEM/∂x, −∂DEM/∂y, +1] on position; gated by slope ≥ s_min & radar validity.
* **Radar Odometry (RO)**

  * Front‑end estimates Δpose_k,k+1 and cov via scan alignment (radar point cloud / power image).
  * Factor ties consecutive poses; robust loss for outliers.
* **VIO / Event / Thermal Odom**

  * Preintegrated IMU + reprojection/photometric factors (or thermal gradient factors); outputs Δpose with cov.
* **LiDAR Odom (LOAM‑style)**

  * ICP/feature factors; helpful in texture‑poor daylight.
* **SoOP**

  * Ranging/Time transfer from LEO/terrestrial/5G; inject as pseudorange/time factors with per‑link σ & DOP.
* **Cooperative UWB**

  * Two‑way range/TDoA to peer nodes or beacons; factors added when available.
* **Baro/Mag**

  * Baro → altitude prior; Mag → yaw prior with robust loss (avoid over‑trust).

### 4.4 Integrity Layer

* Per‑factor χ² gating; rolling **NEES/NIS**; **consistency score** S∈[0,1].
* **Voting & isolation**: down‑weight or exclude failing modalities; declare **degraded modes** with explicit flags.

### 4.5 Timing & Time Sync

* Hardware timestamps preferred; PTP discipline; PPS in; optional CSAC holdover.

---

## 5) Data Rates & Budgets

| Source            |      Rate | Latency Budget |      CPU | Notes               |
| ----------------- | --------: | -------------: | -------: | ------------------- |
| INS mechanization |    200 Hz |          <2 ms |      low | tight loop          |
| Radar Odometry    |  10–20 Hz |         <50 ms |      med | GPU assist optional |
| TRN               |   5–50 Hz |         <10 ms |      low | scalar update       |
| VIO/Event         | 20–120 Hz |         <30 ms | med‑high | adapt rate to load  |
| LiDAR Odom        |  10–20 Hz |         <70 ms |     high | optional            |
| SoOP              |  0.1–2 Hz |        <500 ms |      low | absolute fixes      |
| UWB               |   1–10 Hz |         <20 ms |      low | cooperative         |

Memory target < 500 MB (MVP), CPU < 50% of Orin NX.

---

## 6) Public APIs (PVT & Diagnostics)

### 6.1 gRPC/ROS2 Messages (conceptual)

```proto
message NavSolution {
  double time_s;            // UNIX epoch
  double lat, lon, alt_msl; // or NED position from origin
  double vn, ve, vd;        // m/s
  quaternion q_ned_body;    // attitude
  double cov[36];           // 6x6 upper‑tri; pos/vel cov
  double integrity;         // 0..1
  string mode;              // NOMINAL/TRN_ONLY/RO_SOOP/DEGRADED
}

message IntegrityReport {
  double nees, nis;
  double ro_inlier_rate;
  double trn_inlier_rate;
  double vio_inlier_rate;
  double sop_fix_rate;
  map<string,double> gates; // per‑modality χ² thresholds
}
```

### 6.2 Configuration (YAML excerpt)

```yaml
common:
  frame: NED
  origin: { lat: 32.0853, lon: 34.7818, alt_m: 20 }
  gravity_model: wgs84

ukf:
  rate_hz: 200
  proc_noise:
    gyro_rw: 0.02     # (deg/s)/√Hz
    acc_rw:  0.03     # m/s²/√Hz
    bg_rw:   0.0005   # deg/s²/√Hz
    ba_rw:   0.0005   # m/s³/√Hz

trn:
  dem_path: data/dem/srtm
  min_slope_deg: 0.5
  chi2_gate_1d: 6.63

radar_odom:
  rate_hz: 15
  robust_loss: huber
  max_range_m: 120

vio:
  type: event   # rgb|event|thermal
  keyframe_dt: 0.5
  outlier_chi2: 5.99

sop:
  providers: [leo, terrestrial, 5g]
  min_fix_rate_hz: 0.2

uwb:
  enabled: false

integrity:
  nees_window_s: 10
  nis_window_s: 10

logging:
  binlog_path: logs/run.bag
  summary_json: logs/summary.json
```

---

## 7) Build & Run (MVP)

### 7.1 Dependencies

* C++20, CMake ≥3.22
* **Eigen**, **Sophus**/**manif** (SO(3))
* **GTSAM** (fixed‑lag smoother) or our minimal factor‑graph lib
* **OpenCV** (VIO, even for event/thermal wrappers)
* **yaml‑cpp**, **spdlog**, **abseil**
* Optional: CUDA/cuDNN for radar/VIO acceleration

### 7.2 Build

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

### 7.3 Run

```bash
./aion_node \
  --config configs/aion_telaviv.yaml \
  --dem data/srtm/ \
  --sop providers=leo,terrestrial
```

---

## 8) Calibration & Time Sync

* **Camera**: intrinsics (cal grid), extrinsics to IMU (Kalibr‑style).
* **LiDAR/Radar**: boresight to IMU; plane/board procedures.
* **IMU**: 6‑face static + gentle motion; record temp curves.
* **Lever Arms**: tape + alignment jig; verify in flight (residual minimization).
* **Time**: hardware trigger lines where possible; otherwise PTP with known bounds.

---

## 9) Test Plan (First 90 Days)

**Phase A — Ground (Weeks 1–3)**

* Mount dev rig on car roof. Night/rain runs. Deliver **RO+INS** with <1% drift/km.

**Phase B — TRN (Weeks 4–6)**

* Add radar‑alt; enable TRN. Mixed terrain loops. Show **CEP90 <25–40 m** on rolling terrain segments.

**Phase C — SoOP (Weeks 7–9)**

* Integrate one SoOP provider. Validate absolute fixes rate/DOP. Sync graph + UKF soft‑resets.

**Phase D — Air (Weeks 10–12)**

* Lightweight aircraft/UAS at 300–1000 m AGL, 25–40 m/s. Enable event‑VIO or LiDAR‑odom. Demonstrate **≤50 m CEP90** over 30–40 min.

**KPIs** (logged every second)

* CEP50/CEP90; TRN inlier % vs DEM slope; RO drift %; VIO inlier count; SoOP fix rate; NEES/NIS pass%; integrity score S.

---

## 10) Failure Modes & Degraded Ops

* **Flat terrain / water** → TRN weak: rely on RO + SoOP; increase VIO keyframe density if any texture.
* **Soot/smoke/night** → switch to thermal/event; down‑weight RGB.
* **SoOP dropouts** → INS+RO+TRN holdover; increase covariance; expose **DEGRADED** mode to autopilot.
* **Radar speckle/ghosts** → robust loss; velocity/ego‑motion checks; temporary gate.

---

## 11) Security & Safety

* **Integrity flags** are first‑class outputs (user must honor them).
* **Logs** are for incident replay & certification evidence.
* **Spoof resistance**: multi‑modality cross‑checks; SoOP from diverse providers; optional CSAC timing.
* **Airworthiness path**: begin as **Advisory Nav**; later DO‑178C/DO‑254 increments.

---

## 12) Repository Layout (proposed)

```
aion/
├── apps/
│   └── aion_node.cpp
├── configs/
│   └── aion_telaviv.yaml
├── core/
│   ├── fusion/           # SR‑UKF, state, process model
│   ├── graph/            # fixed‑lag smoother, factors
│   ├── integrity/        # gating, NEES/NIS, mode mgr
│   └── utils/            # time, math, logging
├── frontends/
│   ├── radar_odom/
│   ├── trn/
│   ├── vio_event/
│   ├── lidar_odom/
│   ├── sop/
│   └── uwb/
├── sensors/
│   ├── imu/
│   ├── radar/
│   ├── ralt/
│   ├── camera/
│   ├── lidar/
│   └── baro_mag/
├── msgs/                 # ROS2/gRPC protos
├── scripts/              # calib, dataset tools
├── tests/                # unit + scenario tests
└── docs/
    └── design/*.md
```

---

## 13) Minimal Interfaces (C++ headers, conceptual)

```cpp
// frontends/trn/trn_update.h
struct TrnMeas {
  double alt_agl;                // radar alt
  double dem_h;                  // DEM height at (x,y)
  Eigen::Vector2d dem_grad;      // dDEM/dx, dDEM/dy
  double sigma_alt;              // m
  double slope_deg;              // for gating
};

class TrnUpdate {
 public:
  bool apply(const TrnMeas& z, State& x, Cov& P);
};
```

```cpp
// frontends/radar_odom/ro_factor.h
struct RoDelta { Eigen::Vector3d dpos; Eigen::Quaterniond dquat; Eigen::Matrix<double,6,6> cov; double dt; };
```

```cpp
// core/graph/fixed_lag_smoother.h
class FixedLagSmoother {
 public:
  void addImu(const ImuPacket&);
  void addRo(const RoDelta&);
  void addVio(const VioDelta&);
  void addTrn(const TrnMeas&);
  void addSoop(const SoopMeas&);
  PoseVelBias optimize(double t_now);
};
```

---

## 14) Roadmap (12 months)

* **Q1**: MVP flight demos; closed‑loop with small autopilots (PX4 OFFBOARD advisory).
* **Q2**: Starlink‑passive R&D path; 5G OTDOA adapter; mining/ports pilots with beacons.
* **Q3**: Certification strategy, redundancy HW; maritime & desert modes.
* **Q4**: Production hardware spin; SDK for integrators; initial sales.

---

## 15) Glossary

* **TRN**: Terrain‑Referenced Navigation (DEM + AGL constraints).
* **SoOP**: Signals of Opportunity (LEO/terrestrial/5G not GNSS).
* **SR‑UKF**: Square‑Root Unscented Kalman Filter.
* **NEES/NIS**: Estimator consistency statistics.

---

## License & Credits

© AION Navigation. All rights reserved. Third‑party libraries retain their licenses.

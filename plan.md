ORION 

Perfect—let’s move your team from “concept” to “shipping.” Below is a tight, engineer-facing build plan for **CHIMERA Forever (UAV-Lite)** with owners, milestones, acceptance tests, and code tasks. It assumes the canvas kit you’ve got (UKF, Fixed-Lag Smoother, OpticalFlowFactor, plus the new factors I just added).

---

# 🎯 Objective

Deliver an **indefinite GPS-denied nav** stack on UAV-Lite (IMU + baro + mag + mono cam + tiny ToF/radar altimeter; optional UWB/5G). Error stays **bounded for hours** via the **Anchor Cadence Contract** (≤10s cadence of {yaw OR altitude/scale OR horizontal velocity} constraints), with **UKF re-anchored every 2–4s** by a 5–10s **Fixed-Lag Smoother**.

---

## Phase 0 — Repo/CI hardening (2–3 days)

**Owner:** Infra

1. **CMake**: top-level targets

   * `chimera_core` (UKF, smoother, factors)
   * `chimera_apps` (`chimera_node`, test runners)
   * `chimera_tests` (GTest)
2. **Deps:** Eigen, GTSAM ≥ 4.2, YAML-CPP, GTest.
3. **CI matrix**: Linux x86_64 (GCC/Clang), Jetson (cross), build+unit, `-O3 -DNDEBUG`.
4. **Coding standards**: clang-format, clang-tidy (modernize-use-override, readability-identifier-naming).
5. **Telemetry**: JSONL (`/logs/chimera.jsonl`) with per-cycle `t`, `mode`, `latency_ms`, `cep_est`, `anchor_applied`, `anchor_age_s`, `accel_temp`.

**DoD:** green CI, `chimera_node` runs and emits JSONL.

---

## Phase 1 — Core estimator loop online (1 week)

**Owner:** Estimation

### Tasks

* Implement **UKF propagate/correct** (SO(3) mechanization; square-root covariance).
* Implement **re-anchor** calls: `ukf.resetPose(newPose)` every 2–4 s.
* Wire **FixedLagSmoother** in place of GraphCore for now; window=**7 s**, update **5 Hz**; robust loss (Huber).
* Add **BaroFactor**, **MagYawFactor** (gated), **AltimeterRangeFactor**, **OpticalFlowFactor** into smoother.
* Add **ZUPTFactor** trigger (hover detect: |v|<0.15 m/s, |ω|<2°/s for >0.5 s).

### Config (`config/system.yaml`)

```yaml
rates: {ukf_hz: 200, smoother_hz: 5}
runtime: {reanchor_period_s: 3.0, lag_seconds: 7.0}
sensors:
  baro: {enabled: true, sigma: 0.8}
  mag:  {enabled: true, yaw_sigma_deg: 6.0, gate_chi2: 0.99}
  tof:  {enabled: true, sigma: 0.15}
  camera: {enabled: true, of_hz: 80}
integrity: {robust_loss: huber, huber_k: 1.5}
```

### Acceptance

* **Bench sim**: straight & figure-8; no GPS; OF+ToF+Mag on.
* **SLO**: CEP95 < 10 m for 20-min run; anchor cadence histogram shows ≤10 s gaps.

---

## Phase 2 — Anchor Cadence Contract enforcement (3–5 days)

**Owner:** Controls/Integrity

### Tasks

* **Contract monitor**: rolling 15 s window; compute last timestamps of {yaw, alt, v_b}. If gap >10 s imminent, raise **priority request** to sensors (e.g., trigger OF burst, IR flood).
* **Scheduler**: adaptive smoother rate: 3 Hz nominal, **burst to 8–10 Hz** upon: new factor burst, contract breach risk, or re-lock.
* **Integrity gates**: mag yaw χ² test with temporal stability; ToF sanity (range band), OF quality (feature count, blur, focus).

### Acceptance

* Force-degrade each sensor; verify other heads fill the gaps and logs show **no Contract breaches** in 30-min scenario.

---

## Phase 3 — Feature-poor & night resilience (1 week)

**Owner:** Perception

### Tasks

* **IR flood** enable pin → increases OF success at night.
* **AeroVelocityFactor**: pitot airspeed + wind estimate (start with wind=0; later EKF a slow wind state).
* **OF fallback**: if OF quality < threshold, switch to **aero_velocity** factor automatically.

### Acceptance

* Night test (lights off / IR on): CEP95 ≤ 12 m over 30 min. Over grass/water mock: CEP95 ≤ 15 m using airspeed.

---

## Phase 4 — Optional absolute clamps (1 week, optional)

**Owner:** Comms/Range

### Tasks

* **UWBRangeFactor** (added) to fixed beacons or a teammate; fuse 0.2–0.5 Hz ranges.
* **5G RTT** integration (if API available) → convert to pseudo-range factor (looser noise).

### Acceptance

* With sparse UWB/RTT pings (one every 30–60 s), drift mean < 5–8 m for 60-min route.

---

## Phase 5 — Endurance & abuse testing (1–2 weeks)

**Owner:** QA / Flight

### Scenarios (each ≥60 min)

1. **Textured daylight**: OF+ToF+Mag.
2. **Night IR**: OF(IR)+ToF+Mag.
3. **Feature-poor**: airspeed+ToF+Mag (OF degraded).
4. **Indoor/UWB**: OF+UWB+baro (ToF optional).
5. **Mixed route**: cycle across all above.

### Metrics

* CEP50/95 per hour; **max gap** between re-anchors; % Contract breaches (**target: 0.0%**).
* **Latency budgets** (95th percentile): UKF step < 0.5 ms@200 Hz; smoother update < 10 ms@5 Hz; re-anchor apply < 1 ms.

**DoD:** publish plots + logs; failures have root-cause tags and patches.

---

## Wiring Guide (what to code now)

### 1) Smoother integration

* In `chimera_node.cpp`:

  * Keep a ring buffer of stamped keys (pose `x_t`, velocity `v_t`).
  * Every cycle (200 Hz): UKF propagate.
  * Every 200 ms: build `NonlinearFactorGraph`:

    * Add `BaroFactor(x_t)`, `AltimeterRangeFactor(x_t)`, **gated** `MagYawFactor(x_t)`.
    * Add `OpticalFlowFactor(x_t, v_t)` or `AeroVelocityFactor(x_t, v_t)` depending on quality flags.
  * Call `smoother.add(factors, values, t)`, then `smoother.updateUntil(t)`.

### 2) Re-anchor loop

* Every `reanchor_period_s`:

  * `est = smoother.calculateEstimate();`
  * Extract `Pose3` at the newest key → `ukf.resetPose(est_pose)`.
  * Log: `{"t":..., "anchor":true, "age_s": now - est_pose_time}`.

### 3) Sensor adapters (pseudo-code)

```cpp
// Optical flow adapter
bool get_optical_flow(Vector3& v_b_meas, double& quality);
// ToF adapter
bool get_tof(double& z_agl);
// Mag adapter
bool get_mag_yaw(double& yaw_rad, double& chi2, double& stability);
// Airspeed adapter
bool get_airspeed(double& airspeed_mps);
```

Gate each by quality before factor creation.

### 4) Integrity/robust loss

* Use **Huber** with scale tied to sensor σ.
* Pre-gate by chi²; on repeated rejections, **back-off** that sensor for N seconds.

---

## Telemetry & Ops

### JSONL fields (per 200 Hz cycle)

```json
{"t": 123.456, "ukf_cpu_ms": 0.12, "smoother_cpu_ms": 4.1,
 "anchor": false, "anchor_age_s": 2.8,
 "of_ok": true, "tof_ok": true, "mag_ok": true, "airspeed_ok": false,
 "contract_ok": true, "cep_est_m": 6.3}
```

### Anchor Cadence monitor

* Maintain `last_yaw_t`, `last_alt_t`, `last_vb_t`; if `now - min(...) > 10s` → `contract_ok=false` and trigger burst (increase smoother rate, request IR/OF burst).

---

## Hardware & Calibration Notes

* **ToF/radar**: mount rigidly, downward; filter outliers with median over 5–10 samples.
* **Mag**: soft/hard-iron calibration (ellipsoid fit); keep away from ESCs; dynamic drift detector (current draw → mag weight ↓).
* **IMU**: Allan variance quick-fit; thermal model table; enable **ZARU** on pad (zero angular rate).
* **Camera**: fixed focus to ground plane; IR flood for night; exposure locked for OF.

---

## Definitions of Done

* **MVP Forever** (no UWB/RTT): In any of the 4 main scenes for ≥60 min, **0 contract breaches**, CEP95 ≤ 15 m.
* **Enhanced Forever** (with UWB/RTT OR loop closures): CEP95 ≤ 8 m, same contract SLO.

---

## Task Breakdown (who does what)

* **Estimation**: UKF math, factor Jacobians, smoother integration, re-anchor logic.
* **Perception**: OF quality metrics, IR control, airspeed adapter.
* **Sensors/Drivers**: ToF/radar driver, mag calibration, pitot.
* **Integrity**: gating, robust loss scaling, contract monitor.
* **Infra/QA**: CI, log dashboards, scenario runner (shell scripts), flight safety checklist.

---

If you want, I’ll also:

* add **MagYaw**, **Altimeter**, **AeroVelocity**, **ZUPT**, **UWB** factor stubs (already dropped in canvas),
* push a **scenario runner** (Python) to toggle sensors and produce **Anchor Cadence plots** and CEP curves.



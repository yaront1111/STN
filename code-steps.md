Got it. Here’s a straight, hard-nosed plan to reach **sub-50 m CEP** in GPS-denied missions for **drones and small vessels**, using **available, survivable sensors**—no camera dependency, gravity kept realistic.

# What we’re building (one stack, modular aids)

**INS core (your SR-UKF) + factor graph** with plug-in aids:

* **TRN (Terrain-Referenced Navigation):** radar-altimeter + DEM. Night/IMC-proof, already working in your code.
* **STN/SoOP (Signals-of-Opportunity):** TDOA/Doppler from terrestrial/LEO where present. Passive, radio-agnostic.
* **Radar/LiDAR odometry (optional):** mmWave FMCW or short-range LiDAR for drift clamp when map/SoOP are weak.
* **Barometer + magnetometer (minimal):** baro clamps vertical; mag only if calibrated (or use gyro-compass on boats).
* **Marine variant:** depth + bathymetry (coastal TERCOM-style) and shoreline/radar map-matching.

All aids share **the same measurement API**, same χ²/NEES integrity, same logging. Enable/disable by config; no redesign.

---

# Reality check on “gravity”

Gravity-aided/gradient navigation that truly helps (<100 m) needs **sensitive, low-drift gravimeters or gradiometers** and **high-res gravity maps**—not practical for small UAV SWaP or near-term COTS. It’s a great R&D track, but **not the fastest path to a deployable sub-50 m system**. We’ll keep a placeholder in the graph, but focus investment on TRN+STN+radar odom first.

---

# Target performance (truthful, fieldable)

* **Air (100–800 m AGL, rolling terrain):** TRN (+baro) + INS → **CEP90 30–60 m** continuous; **<50 m** in most hilly/coastal regions.
* **Flat/urban air:** add STN TDOA/Doppler and/or mmWave odom → **CEP90 <50–70 m** depending on signal geometry.
* **Coastal surface craft:** depth+chart TRN near shore + radar shoreline matching → **CEP90 <50 m**.
* **Open sea:** doppler log + INS (no bottom lock) → drift grows; use STN (LF/VLF, 5G coastal, sat Doppler) opportunistically to stay **<100 m**; <50 m near infrastructure.

(Assumes **industrial MEMS IMU** (≤1–3 °/h gyro bias), **OCXO** for radio timing, **ralt σ≈0.5–2 m**, **10–20 Hz TRN**, **SoOP fixes every 10–120 s**.)

---

# Hardware we can actually buy (examples, not endorsements)

* **IMU:** STIM318/HG4930 class (sweet spot) or ADIS1649x if cost-pressed.
* **Clock:** OCXO (±0.01–0.1 ppm) on the RF path to stabilize TDOA/Doppler.
* **Radar altimeter:** small **77 GHz FMCW** or certified UAV ralt; σ≈1 m.
* **mmWave odometry radar (optional):** 60/77 GHz scanning or 2D imaging FMCW.
* **Baro:** industrial digital barometer (temp-comp), fused with ralt.
* **Marine depth (boats):** compact single-beam echosounder; charts for bathy TRN.
* **RF front-end for STN:** multi-channel SDR or dedicated 4G/5G/UWB receiver; 2–4 RF chains preferred for multi-band capture.

---

# Architecture (unchanged core; new aids slot in)

**UKF state now:** `p, v, q, b_g, b_a, (optional wind)`
**Extend for STN:** add `clock bias b` and `clock drift db`.
**Measurements (already aligned with your TRN work):**

* TRN: `z = (h_dem(p) + AGL_meas) - h_msl ≈ 0` (you already implemented residual/χ²/slope-gate).
* Baro: `z = h_baro - h_msl` (low-rate clamp + temp bias).
* STN TDOA: `z_ij = (||p−s_i|| − ||p−s_j||)/c + (b_i−b_j)` with **receiver clock bias in state**; RAIM/χ² gating.
* STN Doppler (if available): line-of-sight range-rate residuals.
* Radar/LiDAR odom: SE(3) relative pose with 6×6 cov, into the graph; UKF soft-reset at 1–2 Hz (you already have the reconciliation pattern).

---

# What we **keep** from your current build

* Steps **1–6** exactly as is (tooling, logging, UKF, IMU, replay).
* **TRN manager + DEM service** you just proved in logs (great health & acceptance).
* Binary log schema; extend with `stn_obs`, `baro`, `odom_radar`.

---

# What we **add** (surgical)

1. **Baro driver + UKF meas.** (tiny): clamp vertical; cross-check ralt for integrity.
2. **STNManager** (mirrors TRNManager): beacon catalog, TDOA/Doppler ingest, χ²/RAIM, clock states.
3. **mmWave odometry** (toggle): basic ICP/scan-matcher or power-image correlator; publish Δpose+cov.
4. **Marine module** (toggle): depth-chart TRN and shoreline matching from marine radar (if on boats).
5. **Integrity layer** (you already scaffolded): rolling NEES/NIS; per-modality scores; modes (NOMINAL / TRN_ONLY / STN_ONLY / DEGRADED).

---

# 10-week execution (uses your codebase; camera-free)

Each item has two sentences and a **Test**.

1. **Barometer integration (Week 1)**
   Implement baro driver + UKF residual with temp/offset calibration and slow bias state. Fuse with ralt for vertical consistency.
   **Test:** 10-min static: baro-DEM-ralt residuals have σ < 2 m; altitude step detected in <2 s.

2. **STN beacon catalog + loader (Week 1)**
   Add `configs/stn/beacons.yaml` (id, lat/lon/alt, band, bias σ). Loader computes local geometry + visibility.
   **Test:** unit test: 4-beacon geometry → GDOP < 6; bad entries rejected.

3. **STNObs ingest + timing (Week 2)**
   Define `STNObs{t_meas, id, type(TDOA/Doppler), value, σ, SNR}` and a lock-free ring buffer. Add OCXO health and t_meas/t_recv separation.
   **Test:** replay with synthetic RF → queue jitter < 3 ms; clock bias estimate converges < 5 s.

4. **STN UKF adapter + RAIM (Week 2)**
   Add TDOA residuals with receiver `b, db` in state; implement subset RAIM to drop outliers.
   **Test:** 1 outlier in 5 → RAIM excludes it; position error improves vs INS-only by ≥3× in urban sim.

5. **mmWave radar odometry (Week 3–4, optional but powerful)**
   Start with frame-to-frame 2D scan matching (NDT/ICP) on range-Doppler power image; publish Δpose+cov at 10–20 Hz.
   **Test:** warehouse/road loop: drift ≤1%/distance; dropouts do not destabilize UKF due to cov modeling.

6. **Fixed-lag smoother (Week 4)**
   5–10 s window with IMU preintegration + TRN pseudo-range + STN + radar-odom; soft-reset UKF at 1–2 Hz.
   **Test:** smoother beats UKF-only drift by ≥2× on dataset; resets pass Mahalanobis check.

7. **Marine depth TRN (Week 5, platform toggle)**
   Correlate depth profile vs chart (along-track TERCOM) with slope gating and tidal offset.
   **Test:** coastal leg 10 km: CEP90 < 40 m with depth + INS.

8. **Integrity & modes (Week 5–6)**
   Compute NEES/NIS per stream; expose integrity S∈[0,1] and mode (NOMINAL/TRN_ONLY/STN_ONLY/DEGRADED).
   **Test:** injected RF jamming → STN integrity drops, mode flips to TRN_ONLY with hysteresis; logs show transitions.

9. **Fieldable configs + hardening (Week 6)**
   Three YAMLs: `air_trn.yaml`, `air_trn_stn.yaml`, `marine_coastal.yaml`; tune χ² gates and process noise presets.
   **Test:** cold start < 20 s to nominal; no overruns at 200 Hz; binlogs < 1 GB per 30 min.

10. **Ground & low-risk flight trials (Week 7–10)**
    Car loops (hilly/flat) for TRN vs TRN+STN; then low-AGL flights on rolling terrain with ralt + STN as available.
    **Test (go/no-go):** **CEP90 ≤ 50 m** on ≥30-min legs in at least one of (hilly air, urban ground, coastal boat); integrity ≥0.8 for ≥80% time.

---

# Why this wins (military + business)

* **Survivability:** Works at night, in dust/fog, under jamming—**no camera dependency**.
* **Modularity:** Any aid can be toggled; you can ship **TRN-only today**, add STN/odom tomorrow.
* **COTS-friendly:** All sensors/parts are **buyable now**; no exotic physics required.
* **Coverage:** TRN excels over terrain/coast; STN excels in flat/urban; radar-odom bridges feature-poor stretches.
* **Investor story:** Clear milestones to **sub-50 m** with defensible IP (sensor fusion, integrity, RF-geometrics), and a path to maritime/coastal markets.

---

# Drop-in config skeletons

**Air – TRN-only (ship today):**

```yaml
trn: { enabled: true, update_rate_hz: 10, chi2_gate_1d: 9.0, min_slope_deg: 0.2 }
baro: { enabled: true, sigma_m: 1.5 }
stn:  { enabled: false }
odom_radar: { enabled: false }
integrity: { enabled: true, nis_window: 200 }
```

**Air – TRN+STN (upgrade when RF ready):**

```yaml
trn: { enabled: true, update_rate_hz: 10, chi2_gate_1d: 9.0, min_slope_deg: 0.2 }
stn:
  enabled: true
  beacons_file: configs/stn/beacons.yaml
  min_beacons: 4
  sigma_tdoa_s: 2.0e-7   # ≈60 m equiv; tune with SNR
  use_doppler: true
  raim: { enabled: true, max_outliers: 1 }
baro: { enabled: true, sigma_m: 1.5 }
odom_radar: { enabled: true, rate_hz: 10 }
```

**Marine – Coastal depth TRN:**

```yaml
marine:
  depth_trn: { enabled: true, chi2_gate_1d: 9.0, tide_model: "offset", tide_offset_m: 0.5 }
  shoreline_match: { enabled: true }   # if marine radar available
trn: { enabled: false }
stn: { enabled: true }                  # coastal SoOP often present
```

---

# Risks & mitigations

* **Flat desert / open sea:** TRN weak → depend on STN & odometry; keep OCXO and multi-band RF to expand SoOP options.
* **RF denial:** RAIM/integrity isolates bad obs; mode falls back gracefully to TRN-only/odom-only with clear flags.
* **Magnetic/magnetometer pain:** Use cautiously; prefer gyro-compass for boats or leave mag off unless calibrated.
* **Clock discipline:** Without GNSS, use **OCXO** and estimate `b, db` in state; schedule periodic RF beacons/LEO Doppler when possible.

---

If you want, I can:

* Generate the **`STNManager` + UKF residual** stubs and a **sample `beacons.yaml`**,
* Add **baro measurement code**, and
* Drop **three ready-to-run configs** (air_TRN, air_TRN_STN, marine_coastal) you can commit and test immediately.

This keeps everything you built, adds only what we need, and gets you

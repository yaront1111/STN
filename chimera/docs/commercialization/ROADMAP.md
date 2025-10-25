# CHIMERA Commercialization Roadmap

**Version:** 1.0.0
**Target:** Production-ready commercial GPS-denied navigation stack
**Platform:** ROS2-focused, with PX4/MAVLink and standalone SDK support
**Timeline:** 90 days to v1.0.0-GA

---

## Executive Summary

CHIMERA has achieved technical validation (11.89m altitude accuracy on 60s real flight), but commercialization requires hardening across **7 critical tracks**:

1. **Robustness & Reliability** (P0) - Production-grade fault handling
2. **Integration & UX** (P0) - ROS2/PX4/SDK interfaces
3. **Validation & QA** (P0) - Automated testing and acceptance gates
4. **Security & Compliance** (P0/P1) - SBOM, licensing, safety
5. **Tooling & Operations** (P0) - Telemetry, diagnostics, debugging
6. **Documentation & Support** (P0) - Integration guides, troubleshooting
7. **Business & Packaging** (P0) - Licensing, SKUs, support plans

**Ship Criteria (v1.0.0-GA):**
- ✅ Altitude MAE ≤ 0.5m (LiDAR), ≤ 2.0m (baro-only)
- ✅ Horizontal drift ≤ 1 m/min (with OF), ≤ 3 m/min (without)
- ✅ Lock time ≤ 3s to first reliable AGL
- ✅ Recovery ≤ 2 keyframes after baro takeover trigger
- ✅ False takeover rate < 1 per 10 hours
- ✅ Latency: UKF < 0.5ms @ 400Hz, Smoother P95 < 30ms @ 5Hz
- ✅ Resource budget: < 25% CPU (4-core i7), < 300MB RAM
- ✅ All acceptance tests passing
- ✅ ROS2 node functional with nav2 integration
- ✅ PX4/MAVLink bridge validated
- ✅ Documentation complete (integration, calibration, troubleshooting)

---

## Current Status (v1.0.0-beta)

### ✅ Completed (Production-Ready)
- Core estimation (UKF + factor graph)
- NED coordinate system consistency
- Baro takeover mechanism
- Auto-detection (IMU units, quaternion convention, LiDAR units)
- Boot-time orientation hardening
- Chebyshev risk budgeting
- Real flight validation (60s, 11.89m final altitude)
- Contract monitoring
- Basic telemetry (JSONL)

### 🚧 In Progress
- Sensor health monitoring (partial - contract monitor exists)
- Thermal compensation (placeholder in UKF)
- Test suite (some tests failing - needs update)

### ❌ Not Started
- Time sync & clock drift handling
- Per-sensor health scores with mode switching
- Calibration persistence framework
- ROS2 node
- PX4/MAVLink bridge
- CI/CD automation
- Crash dumps and repro bundles
- Production telemetry schema
- HIL/simulation integration
- Security hardening
- Customer-facing documentation

---

## 90-Day Execution Plan

### **Phase 1: Weeks 1-3 - Foundation (P0 Core)**

**Goal:** Lock APIs, add robustness primitives, establish CI baseline

#### Robustness & Reliability
- [ ] Time sync architecture (monotonic timebase, skew estimator)
- [ ] Per-sensor health scoring system
  - LiDAR: plateau detection, range validity
  - Baro: noise level, drift rate
  - Mag: norm gating, interference detection
  - OF: texture quality, saturation checks
  - IMU: saturation flags, temperature monitoring
- [ ] Deterministic mode switching (health → altitude source selection)
- [ ] Watchdogs: processing deadlines, NaN guards, covariance clamps
- [ ] Resource budgeting: CPU/memory caps, load-shedding policies

#### Integration & UX
- [ ] Lock C++ API (stable headers, error codes, semantic versioning)
- [ ] Config schema v1 (typed, validated, override hierarchy)
- [ ] Parameter live reload (safe parameters only)

#### Validation & QA
- [ ] CI setup (GitHub Actions or Jenkins)
  - Build matrix: gcc/clang, Ubuntu 22.04/24.04, x86/ARM
  - Sanitizers: ASAN, UBSAN, TSAN
  - Static analysis: clang-tidy
- [ ] Unit test repair (fix test_factors.cpp, add coordinate system tests)
- [ ] Property tests (covariance PSD, orientation SO(3) invariants)

**Deliverables:**
- Stable API headers (v1.0.0-alpha)
- Health monitoring in telemetry
- CI running on PRs
- Unit test suite passing

---

### **Phase 2: Weeks 4-6 - Integration (P0 Deployment)**

**Goal:** Enable ROS2/PX4 deployment, calibration framework

#### Integration & UX
- [ ] **ROS2 Node (Primary)**
  - Package: `chimera_nav`
  - Topics: `/chimera/odometry` (nav_msgs/Odometry), `/chimera/diagnostics`
  - Parameters: sensor sigmas, thresholds, mode overrides
  - Launch files: single/multi-sensor configs
  - TF tree: `map → odom → base_link`
  - nav2 integration guide
- [ ] **PX4/MAVLink Bridge**
  - Publish: ODOMETRY, ATTITUDE, HEARTBEAT
  - Custom extension: CHIMERA_STATUS (health, mode, altitude source)
  - Parameter mapping: PX4 EKF settings
- [ ] Calibration framework
  - Factory: IMU scale/misalignment, mag hard/soft-iron, camera intrinsics, extrinsics
  - Field: mag recal, baro offset "tap to zero", OF sanity checks
  - Persistence: versioned JSON with checksums
  - Boot validation: reject stale/invalid calibs

#### Robustness & Reliability
- [ ] Thermal compensation implementation
  - IMU bias polynomials (gyro/accel vs temperature)
  - Baro altitude correction (temp vs pressure)
  - Load LUTs at boot, apply in UKF/factors

#### Validation & QA
- [ ] Dataset pack creation
  - 5+ curated scenarios (takeoff, cruise, landing, low texture, thermal variation)
  - Labeled ground truth (GNSS/mocap)
  - Checksums and manifests
  - Baseline acceptance report
- [ ] Acceptance test harness
  - Automated scoring: altitude MAE, drift, lock time, etc.
  - CSV/JSON output for trending
  - Pass/fail gates for CI

**Deliverables:**
- ROS2 package functional
- PX4 MAVLink bridge working
- Calibration tools (CLI)
- Dataset pack v1

---

### **Phase 3: Weeks 7-9 - Validation (P0 Quality)**

**Goal:** Comprehensive testing, fault injection, long-soak validation

#### Validation & QA
- [ ] Fault injection tests
  - Dropped OF frames (simulate camera occlusion)
  - LiDAR plateau (stuck reading)
  - Baro step (sudden pressure change)
  - Mag spike (interference)
  - IMU saturation
  - Clock skew
- [ ] Long-soak testing
  - \> 1 hour continuous operation
  - Memory leak detection (Valgrind)
  - CPU/latency percentile tracking
- [ ] Thermal sweep (bench testing)
  - -10°C to +60°C chamber
  - Bias drift characterization
  - Calibration validation
- [ ] HIL/Simulation integration
  - Gazebo/Ignition scenarios (grass, asphalt, water)
  - Monte-Carlo with sensor noise/stalls
  - Automated pass/fail scoring

#### Robustness & Reliability
- [ ] Deterministic resource limits
  - Enforce CPU caps (preempt low-priority work)
  - Memory pool allocations (no dynamic alloc in hot path)
  - Predictable latency (deadline monitors)
- [ ] Crash dump & repro bundles
  - Last-N sensor snapshots
  - State at failure
  - Deterministic replay from bundle

#### Tooling & Operations
- [ ] Telemetry schema v1 finalization
  - Versioned, typed schema (JSON Schema)
  - Health/mode/reason fields
  - Backward compatibility rules
- [ ] Dashboard tooling
  - Grafana export or CLI plotting
  - Real-time AGL/health/errors

**Deliverables:**
- Acceptance test report (all gates passing)
- Thermal characterization data
- Fault injection test suite
- Crash dump tooling

---

### **Phase 4: Weeks 10-12 - Hardening & Launch Prep (P0 Ship)**

**Goal:** Security, documentation, v1.0.0-GA preparation

#### Security & Compliance
- [ ] SBOM generation (dependencies pinned, licenses audited)
- [ ] Code signing (binaries, calibration blobs)
- [ ] Telemetry privacy controls (opt-in/opt-out, PII-free)
- [ ] Export compliance review
- [ ] Basic safety case (FMEA for altitude failures)

#### Documentation & Support
- [ ] Integration guide
  - Sensor wiring and timing
  - Extrinsics calibration
  - Example configs (fixed-wing, multirotor)
  - ROS2 launch files
  - PX4 parameter maps
- [ ] Calibration guide
  - Factory procedures
  - Field tools and workflows
  - Success criteria and validation
- [ ] Troubleshooting playbook
  - Common symptoms → causes → fixes
  - Diagnostic commands
  - Telemetry interpretation

#### Business & Packaging
- [ ] Licensing finalization
  - Commercial EULA
  - Evaluation license (30-day, feature-limited)
  - OSS attribution notices
- [ ] SKU definitions
  - Core: UKF + LiDAR + Baro + Mag
  - Pro: + Optical Flow + Airspeed + Wind
  - OEM: SDK + source + support
- [ ] Support plan
  - SLAs (P0: 24h, P1: 3 business days)
  - Issue tracker workflow
  - Release cadence (quarterly)
  - LTS branch policy (2 years)

#### Final Validation
- [ ] Full acceptance test matrix
- [ ] Performance benchmarks on target HW
- [ ] Beta customer feedback integration
- [ ] Release candidate testing

**Deliverables:**
- v1.0.0-rc1 (release candidate)
- Complete documentation suite
- SBOM and licenses
- Support playbook

---

### **Phase 5: Launch (Week 13+)**

- [ ] v1.0.0-GA release
- [ ] Public announcement (blog post, demo video)
- [ ] Customer onboarding (first 3 pilots)
- [ ] Issue tracker open
- [ ] Support rotation established

---

## Dependency Graph

```
Foundation (Weeks 1-3)
    ├──> Integration (Weeks 4-6)
    │       ├──> Validation (Weeks 7-9)
    │       └──> Hardening (Weeks 10-12)
    └──> Validation (Weeks 7-9)
            └──> Hardening (Weeks 10-12)
                    └──> Launch (Week 13)
```

**Critical Path:**
1. Sensor health monitoring → ROS2 node → Acceptance tests → GA
2. Calibration framework → Dataset pack → Thermal validation → GA

---

## Resource Allocation

### Engineering
- **Core Estimation Team** (2 engineers): Robustness, thermal compensation
- **Integration Team** (2 engineers): ROS2 node, PX4 bridge, SDK
- **QA Team** (1 engineer): Test harness, CI, datasets
- **DevOps** (1 engineer): Build system, CI/CD, tooling
- **Tech Writer** (0.5 FTE): Documentation, troubleshooting

### Hardware
- Development: 4x companion computers (Jetson/RPi/NUC)
- Test: 2x UAV platforms (multirotor + fixed-wing)
- Lab: Thermal chamber, sensor stimulation equipment

### Infrastructure
- CI: GitHub Actions (or Jenkins)
- Artifact storage: S3/GCS for datasets and binaries
- Monitoring: Grafana for telemetry visualization

---

## Risk Mitigation

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Thermal compensation fails validation | Medium | High | Early bench testing (Week 5), fallback to factory cal |
| ROS2 node performance issues | Low | Medium | Profile early (Week 4), optimize before dataset validation |
| PX4 integration breaks existing workflows | Medium | High | Feature flag, parallel testing, rollback plan |
| Dataset diversity insufficient | Medium | High | Source from beta customers, simulate edge cases |
| Acceptance criteria too strict | Low | High | Beta test with relaxed gates, tune based on data |
| SBOM reveals licensing conflicts | Low | Critical | Early audit (Week 2), replace problematic deps |

---

## Success Metrics

### Technical
- [ ] All acceptance tests pass
- [ ] Zero P0 bugs in issue tracker
- [ ] CPU/RAM within budget on all target HW
- [ ] \> 95% test coverage on critical paths

### Business
- [ ] 3 beta customers deployed successfully
- [ ] Support SLA met for first month
- [ ] Documentation rated > 4/5 by beta users
- [ ] Zero license compliance issues

### Operational
- [ ] CI runs < 15 minutes end-to-end
- [ ] Zero false-positive test failures
- [ ] Telemetry dashboard live and monitored

---

## Post-Launch (v1.1+)

### Enhancements (Prioritized by customer demand)
1. **On-device self-calibration** (camera extrinsics via observability)
2. **Mag interference mapping** (current-correlated bias)
3. **Visual/terrain priors** (DEM lookup in Altimeter factor)
4. **Multi-sensor redundancy** (stereo OF, dual LiDAR, dual-IMU)
5. **Formal safety case** (if targeting regulated operations)
6. **Cloud telemetry pipeline** (optional SaaS tier)

### Ecosystem
- **ROS1 bridge** (backward compatibility)
- **ArduPilot support** (MAVLink tuning)
- **Docker images** (easy deployment)
- **Python SDK** (research & prototyping)

---

## Appendices

See detailed specifications:
- [ROBUSTNESS.md](./ROBUSTNESS.md) - Robustness & reliability architecture
- [INTEGRATION.md](./INTEGRATION.md) - ROS2/PX4/SDK integration
- [VALIDATION.md](./VALIDATION.md) - QA strategy and acceptance criteria
- [SECURITY.md](./SECURITY.md) - Security & compliance
- [TOOLING.md](./TOOLING.md) - Operations tooling
- [DOCUMENTATION.md](./DOCUMENTATION.md) - Documentation structure
- [BUSINESS.md](./BUSINESS.md) - Business model and packaging

Implementation artifacts:
- [CI_CHECKLIST.yaml](../specs/CI_CHECKLIST.yaml) - CI enforcement gates
- [telemetry_schema_v1.json](../specs/telemetry_schema_v1.json) - Production telemetry
- [mavlink_mapping.md](../specs/mavlink_mapping.md) - PX4/MAVLink integration
- [ros2_architecture.md](../specs/ros2_architecture.md) - ROS2 node design

---

**Approved By:** [TBD]
**Last Updated:** 2025-01-25
**Next Review:** 2025-02-08 (End of Phase 1)

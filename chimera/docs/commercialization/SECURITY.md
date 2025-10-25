# Security & Compliance

**Track:** P0/P1
**Owner:** DevOps + Legal
**Timeline:** Weeks 10-12

---

## SBOM (Software Bill of Materials)

### Dependencies

| Component | Version | License | Usage |
|-----------|---------|---------|-------|
| GTSAM | 4.2.0 | BSD-3 | Factor graph optimization |
| Eigen3 | 3.4.0 | MPL-2.0 | Linear algebra |
| Boost | 1.74+ | BSL-1.0 | JSON parsing |
| yaml-cpp | 0.7+ | MIT | Config parsing |

**Generation:**
```bash
# Use SPDX format
cyclonedx-cli generate -i . -o sbom.json
```

### License Audit Checklist
- [ ] All third-party licenses documented
- [ ] No GPL conflicts (use LGPL/BSD/MIT only)
- [ ] OSS attribution notices in README
- [ ] Export control review (if shipping internationally)

---

## Code Signing

```bash
# Sign binaries
gpg --detach-sign --armor chimera_node_multi
gpg --detach-sign --armor libchimera_core.a

# Verify
gpg --verify chimera_node_multi.asc chimera_node_multi
```

**Calibration blobs:**
```bash
# SHA256 checksum in calibration file
sha256sum factory_cal.json > factory_cal.json.sha256
```

---

## Telemetry Privacy

### Opt-In/Opt-Out

```yaml
chimera_config:
  telemetry:
    enabled: true
    pii_filter: true  # Strip serial numbers, GPS coords
    cloud_upload: false  # Local-only by default
```

### PII-Free Telemetry
- No GNSS coordinates (if available)
- No serial numbers
- No user identifiable data
- Aggregate statistics only

---

## Safety Case (Basic FMEA)

| Hazard | Cause | Effect | Mitigation |
|--------|-------|--------|------------|
| Alt divergence > 50m | LiDAR stuck + baro drift | Incorrect altitude estimate | Baro takeover, contract monitor |
| Total sensor loss | All sensors fail | No state estimate | Failsafe to FC (trigger RTL/land) |
| CPU overload | Resource exhaustion | Delayed state updates | Load shedding, deadline monitors |

**Action:** Trigger FC failsafe via MAVLink HEARTBEAT loss or CHIMERA_STATUS.contract_ok=false

---

**Acceptance:**
- [ ] SBOM generated and audited
- [ ] Binaries signed
- [ ] Telemetry privacy documented
- [ ] Basic FMEA complete

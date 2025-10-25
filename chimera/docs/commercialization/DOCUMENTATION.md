# Documentation & Support

**Track:** P0
**Owner:** Tech Writer + Support Team
**Timeline:** Weeks 10-12

---

## Documentation Structure

```
docs/
├── README.md (overview, quick start)
├── integration/
│   ├── ros2_setup.md
│   ├── px4_setup.md
│   ├── sdk_api.md
│   └── sensor_wiring.md
├── calibration/
│   ├── factory_calibration.md
│   ├── field_calibration.md
│   └── thermal_characterization.md
├── troubleshooting/
│   ├── common_issues.md
│   ├── log_interpretation.md
│   └── diagnostic_commands.md
├── api_reference/
│   ├── core_classes.md
│   ├── config_schema.md
│   └── telemetry_schema.md
└── support/
    ├── faq.md
    ├── changelog.md
    └── migration_guides.md
```

---

## Integration Guide Template

### ROS2 Setup

```markdown
# CHIMERA ROS2 Integration

## Prerequisites
- ROS2 Humble/Iron
- Ubuntu 22.04+
- CHIMERA v1.0.0+

## Installation
\`\`\`bash
cd ~/ros2_ws/src
git clone https://github.com/your-org/chimera_nav.git
cd ~/ros2_ws
colcon build --packages-select chimera_nav
\`\`\`

## Launch
\`\`\`bash
ros2 launch chimera_nav chimera.launch.py
\`\`\`

## Verify
\`\`\`bash
ros2 topic echo /chimera/odometry
ros2 run tf2_tools view_frames.py
\`\`\`
```

---

## Calibration Guide Template

### Factory Calibration Procedure

1. **IMU Calibration** (6-position tumble test)
2. **Magnetometer Calibration** (sphere fit)
3. **Camera Intrinsics** (checkerboard)
4. **Extrinsics** (joint optimization)
5. **Thermal Characterization** (chamber sweep -10°C to +60°C)

---

## Troubleshooting Playbook

### Symptom: Altitude diverges

**Causes:**
1. LiDAR stuck → Check plateau detection
2. Baro drift → Check thermal compensation
3. Coordinate system mismatch → Check boot sanity check

**Diagnostics:**
```bash
grep "BOOT WARN" logs/*.log
grep "BARO_TAKEOVER" logs/*.log
tail -100 logs/chimera_multi.jsonl | jq '.sensor_health'
```

**Fixes:**
- If LiDAR stuck: Adjust `lidar_stuck_eps` parameter
- If baro drift: Update thermal model
- If coord mismatch: Re-run orientation boot

---

## Support Plan

### SLAs

| Priority | Response Time | Resolution Time |
|----------|---------------|-----------------|
| P0 (production down) | 4 hours | 24 hours |
| P1 (degraded) | 1 business day | 3 business days |
| P2 (minor) | 3 business days | 1 week |

### Issue Tracker Workflow

1. Customer opens issue (GitHub/JIRA)
2. Support triage (assign priority)
3. Engineering investigation
4. Resolution (patch/workaround/doc update)
5. Customer validation
6. Close with notes

---

**Acceptance:**
- [ ] All guides complete and reviewed
- [ ] Troubleshooting playbook tested
- [ ] Support workflow documented

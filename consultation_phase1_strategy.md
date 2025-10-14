# ORION Phase 1 Implementation Strategy - Gemini Consultation

## Current State

**Project:** ORION - Neural Navigation Stack for GPS-Denied UAV Navigation
**Status:** Empty repository (just started)
**Available Data:** SRTM DEM terrain data (4 tiles covering region N31-32, E034-035)

## Project Overview

ORION is an ambitious AI-powered GPS-denied navigation system combining:
- 8+ sensor streams (IMU, stereo camera, LiDAR, event camera, magnetometer, RF, etc.)
- Multi-modal SLAM (VIO, LiDAR SLAM, Magnetic SLAM, RF SLAM)
- Factor graph fusion (GTSAM)
- Vision Transformer terrain matching for global relocalization
- Target: <1m accuracy, 30+ min GPS-denied flight

**See full details in:** D:\projexts\STN\README-ORION.MD

## Phase 1 Goals (Months 1-4)

From README:
- ROS2 architecture setup
- Stereo VIO (OpenVINS)
- LiDAR SLAM (FAST-LIO2)
- IMU pre-integration
- Basic GTSAM factor graph
- Simple confidence estimation
- **Success: 10+ min flight, <5m drift**

## Critical Questions for Gemini

### 1. MINIMAL VIABLE SYSTEM (MVS)

**Context:** Empty repository, limited initial resources (likely 1-2 developers)

**Question 1a - What's the ABSOLUTE MINIMUM to start?**

For Phase 1, which sensor combination achieves goals with LEAST complexity:

**Option A: IMU + Stereo VIO only**
- Pros: Single odometry pipeline, well-understood, many open-source options
- Cons: Visual failures cause complete system loss

**Option B: IMU + LiDAR SLAM only**
- Pros: Lighting-invariant, no texture dependency
- Cons: Less rich, no visual loop closure initially

**Option C: IMU + Stereo VIO + LiDAR SLAM (full Phase 1)**
- Pros: Redundancy from day 1, better accuracy
- Cons: More complex fusion, longer dev time

**Which option should we pursue first? Can we do A or B first, then add the other?**

### 2. ARCHITECTURE DECISIONS

**Question 2a - Factor Graph from Day 1?**

Should we:
- **Path A:** Start with EKF/UKF fusion (simpler, faster to implement, then migrate to GTSAM later)
- **Path B:** Use GTSAM from day 1 (steeper learning curve, but no migration needed)

**Which is better for 1-2 developers with 4 months?**

**Question 2b - ROS2 Node Structure**

For Phase 1 with 2-3 odometry sources:
- **Monolithic:** Single node with all odometry + fusion?
- **Microservices:** Separate nodes per sensor/odometry + fusion node?

**What's the right balance for development speed vs modularity?**

### 3. IMPLEMENTATION SEQUENCE

**Question 3a - What order to build in?**

Starting from ZERO code, what's the critical path:

**Proposed Sequence:**
1. Set up ROS2 workspace and basic nodes
2. IMU integration and dead reckoning
3. Add Stereo VIO (OpenVINS) with IMU
4. Add LiDAR SLAM (FAST-LIO2)
5. Implement factor graph fusion (GTSAM)
6. Test in simulation (Gazebo)
7. Hardware integration
8. Field testing

**Is this order correct? What should we parallelize?**

**Question 3b - Simulation First or Hardware First?**

Should we:
- Develop in simulation (Gazebo + PX4 SITL) first, then port to hardware?
- Start with real hardware from the beginning?
- Hybrid approach?

### 4. TECHNOLOGY CHOICES

**Question 4a - OpenVINS vs Alternatives**

For Stereo VIO:
- **OpenVINS:** Mentioned in README, EKF-based
- **ORB-SLAM3:** More mature, proven, visual + inertial
- **Kimera:** MIT's VIO with semantic understanding
- **VINS-Fusion:** Popular, well-documented

**Which is best for Phase 1 given our goals?**

**Question 4b - FAST-LIO2 vs KISS-ICP**

For LiDAR SLAM:
- **FAST-LIO2:** High performance, mentioned in README
- **KISS-ICP:** Simpler, lightweight, easier to integrate

**Which for Phase 1?**

**Question 4c - GTSAM Learning Curve**

How long does it realistically take to:
- Learn GTSAM basics?
- Integrate IMU pre-integration?
- Add VIO and LiDAR factors?
- Get it working reliably?

**Is 4 months enough for first-time GTSAM users?**

### 5. HARDWARE CONSIDERATIONS

**Question 5a - Do we need expensive hardware for Phase 1?**

README lists:
- VectorNav VN-300 tactical IMU ($3,500)
- OAK-D Pro stereo ($349)
- Livox Mid-360 LiDAR ($599)
- Jetson Orin NX ($599)

**Can Phase 1 work with cheaper alternatives?**
- Consumer-grade IMU ($50-100)?
- Basic stereo camera ($100)?
- Cheaper LiDAR ($200-300)?

**Or do we need tactical-grade from the start?**

**Question 5b - Jetson Orin NX Necessary?**

For Phase 1 (no ViT, no event camera yet):
- Is Jetson Orin NX required?
- Could we use Jetson Nano/Xavier NX ($200-400)?
- Or develop on PC first, optimize later?

### 6. PRACTICAL REALITY CHECK

**Question 6a - 4 Months with 1-2 Developers**

Given:
- Empty repository right now
- Team: Likely 1-2 developers (not 8-10)
- No hardware yet (assumed)
- Experience level: Know UKF basics, learning advanced SLAM

**Realistic Phase 1 deliverables:**
- What CAN we achieve in 4 months?
- What should we CUT from Phase 1 to be realistic?
- What's a minimum demo that proves the concept?

**Question 6b - Should we build "ORION-Lite" first?**

Simpler version:
- Target: <5m accuracy (not <1m)
- Duration: 5-10 min flight (not 30+)
- Sensors: IMU + ONE odometry source (VIO or LiDAR)
- Fusion: EKF instead of factor graph?
- Timeline: 2-3 months instead of 4

**Is this a better starting point?**

### 7. STEP-BY-STEP PLAN REQUEST

**Question 7 - Give us a concrete 16-week plan**

Week-by-week breakdown for Phase 1:
- Week 1-2: ?
- Week 3-4: ?
- Week 5-8: ?
- Week 9-12: ?
- Week 13-16: ?

What should we actually BUILD in each period?

### 8. RISK ASSESSMENT

**Question 8 - What's most likely to fail?**

Which parts of Phase 1 are:
- **Low risk** (proven tech, clear path)?
- **Medium risk** (doable but challenging)?
- **High risk** (research-level, might not work)?

Where should we focus effort to de-risk?

## What We Need from You

**Brutally honest, practical guidance:**

1. **Minimal Viable System** - What's the simplest thing that proves the concept?
2. **Step-by-step implementation sequence** - What to build Week 1, 2, 3...
3. **Technology recommendations** - Specific libraries/tools for Phase 1
4. **Realistic timeline** - What's actually achievable in 4 months?
5. **De-scoping advice** - What to postpone to Phase 2?
6. **Hardware requirements** - Can we start cheaper and upgrade?
7. **Learning resources** - Best tutorials for GTSAM, OpenVINS, ROS2 navigation

## Constraints

- **Budget:** Limited initially (prefer <$2K for Phase 1 hardware if possible)
- **Team:** 1-2 developers
- **Timeline:** 4 months to meaningful demo
- **Experience:** Mid-level robotics, learning advanced SLAM
- **Risk tolerance:** Want to succeed, not fail attempting too much

## Desired Output

A practical, achievable plan that:
- Gets us from ZERO to WORKING demo in 4 months
- Proves core ORION concepts (multi-sensor fusion, GPS-denied nav)
- Sets foundation for Phases 2-5
- Doesn't overreach and fail

**What should we actually build first?**

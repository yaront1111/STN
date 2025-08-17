import numpy as np
import pandas as pd
from pathlib import Path

def simulate_straight_and_level(T=120.0, dt=0.01, speed_mps=50.0, seed=42, out_dir="data"):
    """
    Straight & level flight.
    IMPORTANT: accelerometer measures specific force, so Z must read -g.
    """
    rng = np.random.default_rng(seed)
    N = int(T/dt)
    t = np.arange(N) * dt
    g = 9.80665

    # Truth (NED): constant speed north, constant altitude
    vn = np.full(N, speed_mps); ve = np.zeros(N); vd = np.zeros(N)
    pn = np.cumsum(vn*dt);      pe = np.cumsum(ve*dt);      pdown = np.cumsum(vd*dt)

    # IMU (body aligned with NED)
    gyro = rng.normal(0, 0.001, size=(N, 3))
    acc  = np.zeros((N, 3))
    acc[:, 2] = -g                                  # <-- the fix
    acc += rng.normal(0, 0.02, size=acc.shape)      # noise

    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    pd.DataFrame({
        "t": t,
        "ax": acc[:, 0], "ay": acc[:, 1], "az": acc[:, 2],
        "gx": gyro[:, 0], "gy": gyro[:, 1], "gz": gyro[:, 2],
    }).to_csv(out / "sim_imu.csv", index=False)

    pd.DataFrame({
        "t": t,
        "pn": pn, "pe": pe, "pd": pdown,
        "vn": vn, "ve": ve, "vd": vd,
    }).to_csv(out / "sim_truth.csv", index=False)

    print("IMU written with az ≈ -g (specific force).")
    return out / "sim_imu.csv", out / "sim_truth.csv"

if __name__ == "__main__":
    simulate_straight_and_level()

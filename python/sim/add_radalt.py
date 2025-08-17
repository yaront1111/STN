import numpy as np, pandas as pd
from pathlib import Path
# Synthetic terrain must "match" the C++ one (same A/L, phases).
A1, L1 = 100.0, 2000.0
A2, L2 = 50.0, 800.0

def terrain_height(n, e):
    w1, w2 = 2*np.pi/L1, 2*np.pi/L2
    return A1*np.sin(w1*n)*np.cos(w1*e) + A2*np.cos(w2*n + 0.4)*np.sin(w2*e + 0.8)

def main(truth_csv="data/sim_truth.csv", out_csv="data/radalt.csv", sigma=0.5, seed=1):
    rng = np.random.default_rng(seed)
    truth = pd.read_csv(truth_csv)
    # altitude (Up) = -pd (since NED: down positive)
    alt = -truth["pd"].values
    terr = terrain_height(truth["pn"].values, truth["pe"].values)
    agl  = alt - terr + rng.normal(0.0, sigma, size=len(alt))  # radar altimeter
    pd.DataFrame({"t": truth["t"], "agl_m": agl}).to_csv(out_csv, index=False)
    print("Wrote", out_csv, "rows:", len(agl))

if __name__ == "__main__":
    main()
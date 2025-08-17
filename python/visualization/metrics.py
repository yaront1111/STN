import pandas as pd, numpy as np
from pathlib import Path

root = Path(__file__).resolve().parents[2]
truth = pd.read_csv(root/"data/sim_truth.csv")
est   = pd.read_csv(root/"data/run_output.csv")

N = min(len(truth), len(est))
truth, est = truth.iloc[:N].reset_index(drop=True), est.iloc[:N].reset_index(drop=True)

# Errors (NED)
e_n = est.pn - truth.pn
e_e = est.pe - truth.pe
e_d = est.pd - truth.pd
e_2d = np.sqrt(e_n**2 + e_e**2)
e_3d = np.sqrt(e_n**2 + e_e**2 + e_d**2)

def cep(vals, p):
    vals = np.sort(vals)
    return float(vals[int(p*len(vals))-1])

print(f"samples={N}, dt≈{(truth.t.iloc[1]-truth.t.iloc[0]):.3f}s, T={truth.t.iloc[-1]:.1f}s")
print(f"2D CEP50={cep(e_2d,0.50):.3f} m, CEP95={cep(e_2d,0.95):.3f} m")
print(f"3D RMSE={np.sqrt(np.mean(e_3d**2)):.3f} m")
print(f"Final 2D error={e_2d.iloc[-1]:.3f} m")
print(f"Final vertical error={e_d.iloc[-1]:.3f} m")
print(f"Max |vd|={np.max(np.abs(est.vd)):.3f} m/s, Mean |vd|={np.mean(np.abs(est.vd)):.3f} m/s")
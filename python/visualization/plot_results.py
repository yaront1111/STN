# python/visualization/plot_results.py
import argparse
from pathlib import Path

import pandas as pd
import plotly.graph_objects as go
import plotly.io as pio

def main(renderer: str = "browser"):
    # Force a sane renderer (prevents notebook/nbformat errors)
    pio.renderers.default = renderer

    root = Path(__file__).resolve().parents[2]
    truth_csv = root / "data/sim_truth.csv"
    run_csv   = root / "data/run_output.csv"

    # Read data
    truth = pd.read_csv(truth_csv)
    run   = pd.read_csv(run_csv)

    # Build figure
    fig = go.Figure()
    fig.add_trace(go.Scatter3d(
        x=truth['pe'], y=truth['pn'], z=-truth['pd'],
        mode='lines', name='Truth', line=dict(width=6)
    ))
    fig.add_trace(go.Scatter3d(
        x=run['pe'], y=run['pn'], z=-run['pd'],
        mode='lines', name='STN v0.1', line=dict(width=4)
    ))
    fig.update_layout(
        title="STN v0.1 — Trajectories",
        scene=dict(
            xaxis_title='East (m)',
            yaxis_title='North (m)',
            zaxis_title='Up (m)'
        )
    )
    fig.show()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--renderer", default="browser",
                    help="Plotly renderer (e.g., browser, vdom, png). Default: browser")
    args = ap.parse_args()
    main(args.renderer)

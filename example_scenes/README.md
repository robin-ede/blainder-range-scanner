# Example Scenes

Open any of these `.blend` files directly in Blender 5.0+ to inspect the scene
and run scans interactively via the BLAINDER panel.

| File | Description | Used to prove |
|---|---|---|
| `sonar_example.blend` | Simple sonar scene used in early development demos | Basic sonar pipeline |
| `straight_push_degraded.blend` | Dual-sensor scene (LiDAR above water + sonar below). Sensor path pushes straight through, with GNSS drift and sea-state degradation applied. | TO4 — baseline RMSE ≥ 2 m, post-fusion ≤ 0.15 m, ≥ 80% error reduction |
| `chesapeake_bay_300m_real_bathy.blend` | 300 m crop of the NOAA Chesapeake Bay M130 2017 bathymetric survey with a lawnmower scan path. Real seabed geometry, sensors positioned above and below the water surface. | TO3 / D2 — KS depth-distribution test against NOAA reference data |

## Running a scan

1. Open a scene in Blender.
2. Open the **BLAINDER** side panel (press `N` in the 3D viewport).
3. Configure scanner parameters or load a preset.
4. Press **Start Scan**.

## Running the full validation pipeline

See [`validation/REPRODUCING.md`](../validation/REPRODUCING.md) for the
command-line workflows that produced the committed evidence in `validation/`.

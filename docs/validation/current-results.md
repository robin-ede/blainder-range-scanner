# Current Validation Summary

This document summarizes what the repository currently demonstrates in practical
terms, without assuming that the reader knows the codebase.

> **Committed evidence lives in [`validation/`](../../validation/) at the repo
> root** — reference data, live Blender reports, and demo plots.
> [`validation/REPRODUCING.md`](../../validation/REPRODUCING.md) has
> step-by-step commands to regenerate every result from scratch.

## What We Are Trying To Prove

The project goal is to support a dual-sensor fusion pipeline for a company use
case involving above-water and below-water sensing.

The simulation/validation work is intended to show two things:

1. the synthetic data pipeline is realistic enough to be useful
2. the fusion and correction workflow behaves correctly on both synthetic scenes
   and real-world-derived geometry/data

## What Is Already Demonstrated

### 1. Synthetic validation pipeline works end-to-end

The repository now supports:

- fixed-scene validation
- pose sweep and noise sweep workflows
- animated path validation
- degraded runs with GNSS drift and sea-state corruption
- reference-based correction and blind-registration reporting
- runtime and memory reporting

These workflows produce JSON reports and CSV summaries with metrics such as RMSE,
P95, alignment quality, error reduction, and performance limits.

### 2. Real bathymetry mesh validation works

We can now:

- identify shallow NOAA survey candidates
- ingest processed bathymetry products (NetCDF now, BAG/GeoTIFF when available)
- crop a small real seabed patch
- convert that patch into a Blender mesh
- generate scan-ready Blender scenes from that mesh
- run the scanner/validation pipeline on real seabed morphology

This supports the claim that the mapping and fusion workflow is not limited to
toy geometry.

### 3. Real sounder point extraction works

We can now:

- extract real point clouds from Kongsberg `.all` survey files using the real
  GitHub `pyall` reader
- normalize those points into reusable `.csv` / `.npy` outputs
- build a local mesh patch from those real points
- rescan that patch synthetically for direct point-cloud comparison

This supports the stronger claim that the synthetic points can be compared to
real measured points, not only to raster surfaces.

## Most Important Current Results

### Animated real-bathymetry mesh validation

On the small Chesapeake-derived real bathymetry crop, the animated validation
workflow runs successfully and produces valid reports with acceptable runtime and
memory usage.

### Rainier real-point extraction

From a Rainier Kongsberg sample file we extracted:

- 88 pings
- 35,200 real points

This proves we have a working real-sounder ingestion path for validation.

### Rainier same-geometry synthetic comparison

Using a flatter `10 m x 10 m` Rainier patch derived from the real point cloud:

- real point count: 3,908
- synthetic point count: 36,790
- both clouds occupy the same local footprint (`~10 x 10 m`)
- real depth range: `-20.02` to `-15.84 m`
- synthetic depth range: `-19.79` to `-15.89 m`

Cloud-to-cloud comparison on the same geometry gives:

- symmetric nearest-neighbour mean distance: about `0.225 m`
- symmetric nearest-neighbour RMSE: about `0.321 m`

After density-matching the synthetic cloud to the real point count:

- real median XY spacing: about `0.051 m`
- synthetic median XY spacing: about `0.067 m`
- symmetric distance remains about `0.24 m` mean / `0.32 m` RMSE

## What These Results Mean

The results do **not** mean the synthetic pipeline is a perfect physical model
of a specific sonar instrument.

They **do** mean:

- the synthetic points reproduce the same patch footprint and similar depth
  envelope as the real sounder sample
- the synthetic cloud is denser and cleaner than the real sample
- the geometric mismatch between the real and synthetic clouds remains on the
  order of a few decimetres over a small real patch

That is a defensible validation argument for this project stage:

- real seabed morphology: validated
- real point-cloud extraction: validated
- synthetic-vs-real same-geometry comparison: demonstrated

## Current Limitations

- The real-point comparison is currently strongest for Kongsberg `.all` input.
- Chesapeake HYSWEEP/HSX survey files are still harder to parse directly.
- The Rainier point comparison currently uses an underwater rotating point
  sampler rather than a full side-scan waveform model.
- Processed depth-distribution KS tests against raster depth grids work as a
  plumbing check, but they are not yet the strongest realism argument by
  themselves.

## Recommended Interpretation For Project Documentation

The cleanest claim today is:

> The project validates the fusion pipeline on real seabed geometry and shows
> that synthetic underwater point sampling can be compared directly against real
> survey-derived sounder points on the same local geometry.

That is stronger than claiming full sensor-physics equivalence, and it is well
supported by the current repository outputs.

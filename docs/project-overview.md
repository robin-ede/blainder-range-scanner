# Project Overview

This repository is a Blender 5 fork of BLAINDER extended for a capstone project
that studies dual-sensor mapping with an above-water scanner and a below-water
scanner. The goal is not just to synthesize point clouds, but to validate that
the resulting data is useful for a fusion pipeline intended for real company
data.

## What The Project Proves

The validation work now supports two complementary arguments:

1. Real seabed geometry validation
   - Processed bathymetry (NetCDF/GeoTIFF/BAG when available) can be cropped,
     imported into Blender, scanned, and evaluated.
   - This shows that the fusion/reconstruction workflow operates on realistic
     underwater morphology rather than only synthetic primitives.

2. Real sounder point validation
   - Real Kongsberg `.all` survey files can be extracted into point clouds.
   - Real point clouds can be converted into a local mesh patch and rescanned
     synthetically for direct point-cloud comparison.
   - This supports the claim that the synthetic data resembles real sampled data
     rather than only matching a raster surface.

## Main Technical Areas

- Blender add-on and GUI in `range_scanner/`
- Validation/reporting/fusion code in `range_scanner/validation/`
- Repeatable runners and real-data tooling in `scripts/`
- Curated example scene in `example_scenes/sonar_example.blend`

## Current Validation Capabilities

- fixed-scene validation
- pose then noise sweeps
- animated path validation
- blind vs reference correction reporting
- runtime and memory reporting
- real bathymetry mesh validation
- real point-cloud extraction from Kongsberg `.all`

## Suggested Reading Order

1. `README.md`
2. `docs/repo-map.md`
3. `scripts/README.md`
4. `docs/validation/real-bathymetry.md`

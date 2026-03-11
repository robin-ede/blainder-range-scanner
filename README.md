# Range Scanner Simulation for Blender

> **Fork Notice:** This is a fork of [BLAINDER](https://github.com/ln-12/blainder-range-scanner) by Lorenzo Neumann, updated for Blender 5.0 as part of a capstone project at Texas A&M University.

This Blender add-on enables you to simulate lidar, sonar and time of flight scanners in your scene. Each point of the generated point cloud is labeled with the object or part id that was set before the simulation. The obtained data can be exported in various formats to use it in machnie learning tasks (see [Examples](#examples) section).

The paper can be found here: https://www.mdpi.com/1424-8220/21/6/2144

## Project Snapshot

This fork extends the original scanner with a validation workflow focused on dual-sensor mapping: above-water plus below-water scanning, alignment, degradation studies, fusion/reconstruction metrics, and real-bathymetry based validation.

If you are new to the repo, start here:

- `docs/project-overview.md` - what the capstone fork is trying to prove
- `docs/repo-map.md` - where the important code and scripts live
- `scripts/README.md` - which script entrypoints are canonical
- `docs/validation/real-bathymetry.md` - processed bathy + real point workflows
- `docs/validation/current-results.md` - plain-English summary of what is already demonstrated

Quick validation entrypoint:

```bash
python scripts/run_fixed_scene_validation_batch.py \
  --blender "/Applications/Blender.app/Contents/MacOS/Blender" \
  --scene example_scenes/sonar_example.blend \
  --config scripts/fixed_scene_validation.example.json
```

## Original scene

<img src="images/part_segmentation_scene.png" width="500">

The figure shows a group of chairs (left), a Blender camera object and a light source (right). The chair legs and seats are randomly colored differently to make it easier to distinguish them in the following images.

## 3D point clouds

<img src="images/part_segmentation_cloud.png" width="500">

In each of the four figures, a generated, three-dimensional point cloud can be seen. The different colors for each data point have different meanings: 
* top left: original color from object material
* top right: grey scale intensity representation
* bottom left: each color stands for one object category (blue = floor, red = chair)
* bottom right: each color represents one object subcategory (blue = floor, red/green = seats, orange/turquoise = legs)

Note: the left and middle chairs have the same colors because the subobjects were classified identically (see [Classification](#classification)).

Supported formats:
* [.hdf5](https://en.wikipedia.org/wiki/Hierarchical_Data_Format)
* [.csv](https://en.wikipedia.org/wiki/Comma-separated_values)
* [.ply](https://en.wikipedia.org/wiki/PLY_(file_format))
* [.las](https://en.wikipedia.org/wiki/LAS_file_format)

## 2D annotated images

<img src="images/part_segmentation.png" width="500">

In addition to the 3D points clouds, the add-on can also export 2D images.
* top left: the image rendered from Blenders engine
* top right: depthmap
* bottom left: segmented image
* bottom right: segmented image with bounding box annotations

Supported formats:
* .png
* [pascal voc object descriptions](http://host.robots.ox.ac.uk/pascal/VOC/)

<br /><br />

## Table of Contents
**[Installation](#installation)**<br>
**[Dependencies](#dependencies)**<br>
**[Usage (GUI)](#usage-gui)**<br>
**[Validation Workflows](#validation-workflows)**<br>
**[Visualization](#visualization)**<br>
**[Examples](#examples)**<br>
**[Development](#development)**<br>
**[About](#about)**<br>
**[License](#license)**<br>

<br /><br />

## Installation

This addon supports multiple Blender versions:
- **Blender 5.0+** (main branch) - Uses the new extension format
- **Blender 4.2 LTS** ([see here](https://github.com/ln-12/blainder-range-scanner/tree/blender_4.2_lts))
- **Blender 3.3 LTS** ([see here](https://github.com/ln-12/blainder-range-scanner/tree/blender_3.3_lts))

### Blender 5.0+ Installation

#### Step 1: Install Dependencies

Install the required Python packages into Blender's Python environment.

**macOS:**
```bash
/Applications/Blender.app/Contents/Resources/5.0/python/bin/python3.11 -m pip install PyYAML laspy h5py pypng pascal-voc-writer numpy
```

**Linux:**
```bash
/path/to/blender-5.0/5.0/python/bin/python3.11 -m pip install PyYAML laspy h5py pypng pascal-voc-writer numpy
```

**Windows (as administrator):**
```powershell
& "C:\Program Files\Blender Foundation\Blender 5.0\5.0\python\bin\python.exe" -m pip install PyYAML laspy h5py pypng pascal-voc-writer numpy
```

#### Step 2: Build the Extension

```bash
blender --command extension build --source-dir range_scanner --output-dir .
```

This creates a `range_scanner-1.0.0.zip` file.

#### Step 3: Install the Extension

In Blender:
1. Go to `Edit` → `Preferences` → `Get Extensions`
2. Click the dropdown arrow (top right) → `Install from Disk...`
3. Select the `range_scanner-1.0.0.zip` file
4. Enable the extension

### Blender 4.2 and Earlier

For older Blender versions, see the respective branches linked above. The installation process uses the legacy add-on format with `bl_info`.

### Docker (Headless Full Pipeline)

This repository now includes a Docker-based, headless validation environment for running the full scripted pipeline without relying on a host Blender install. The container is pinned to Blender `5.0.1`, matching the version installed during development.

Build the image from the repository root:

```bash
docker build --platform linux/amd64 -t blainder-range-scanner:blender-5.0.1 .
```

Run the canonical fixed-scene validation workflow:

```bash
docker run --rm \
  -v "$PWD":/workspace \
  blainder-range-scanner:blender-5.0.1 \
  run-fixed-scene \
  --scene /workspace/example_scenes/sonar_example.blend \
  --config /workspace/scripts/fixed_scene_validation.example.json
```

Run the higher-level scripted workflows the same way:

```bash
docker run --rm -v "$PWD":/workspace blainder-range-scanner:blender-5.0.1 \
  run-pose-noise --scene /workspace/generated_scenes/dual_sensor_scene_42.blend \
  --config /workspace/scripts/pose_then_noise_validation.example.json

docker run --rm -v "$PWD":/workspace blainder-range-scanner:blender-5.0.1 \
  run-animated-path --base-scene /workspace/generated_scenes/dual_sensor_scene_42.blend \
  --config /workspace/scripts/animated_path_validation.example.json
```

Generate PNG report visualizations inside the same container:

```bash
docker run --rm -v "$PWD":/workspace blainder-range-scanner:blender-5.0.1 \
  visualize /workspace/path/to/report.json
```

Notes:

- The container is intended for `linux/amd64` headless execution.
- On Apple Silicon hosts, `linux/amd64` runs under emulation because Blender
  only provides the Linux `x64` archive for this setup.
- Mount a host output directory or the whole repo so generated JSON, CSV, and
  PNG artifacts persist after the container exits.
- Blender-style relative paths such as `//output` resolve relative to the
  mounted `.blend` scene path inside the container.

<br /><br />

## Dependencies

The following Python packages are required (install via pip as shown in [Installation](#installation)):

* [PyYAML](https://github.com/yaml/pyyaml) - Configuration file loading
* [numpy](https://github.com/numpy/numpy) - Numerical operations
* [laspy](https://github.com/laspy/laspy) - LAS file export
* [h5py](https://github.com/h5py/h5py) - HDF5 file export
* [pascal_voc_writer](https://github.com/AndrewCarterUK/pascal-voc-writer) - Pascal VOC XML export
* [pypng](https://github.com/drj11/pypng) - PNG image export

<br /><br />

## Usage (GUI)

In Blenders 3D View, open the sidebar on the right (click on the little `<`) and select `Scanner`.

![alt text](images/open_sidebar.png)

Please note that not all of the following options are available for all scanner types.

<br />

### Gerneral settings

![alt text](images/scanner_panel_main.png)

#### Scanner object

Select the object in the scene which should act as range sensor. This object must be of type `camera`.

#### Join static meshes

If enabled, all static meshes in the scene are joined into one mesh prior to the simulation.

#### Generate point clouds

This operator starts the actual scanning process. You should set all parameters (see the following sections) before you hit the button. It is generally recommended to open the command window to see any warning or errors occuring during simulation.

<br />

### Presets

![alt text](images/scanner_panel_presets.png)

#### Scanner category / name

In this section you can select a predefined sensor. First, choose one of the categories `Lidar`, `Sonar` or `Time of flight`. Then you can select a specific sensor. 

#### Load preset

When pressing `Load preset`, all parameters are applied automatically.

<br />

### Scanner

![alt text](images/scanner_panel_scanner.png)

#### Scanner type

The scanner type lets you define the operation mode of the sensor. Depending on the selected type, you can further specify the characteristics of the sensor. 

#### Field of view (FOV) / Resolution

The fields of view define the area which is covered by the scan horizontally and vertically. The resolution indicates the angle between to measurements.

#### Rotation

In case of rotating sensors, the number of rotations per seconds is used to simulate correct measurements during animations.

<br />

In the case of the `sideScan` scanner type, you can set additional parameters ([more info](https://dosits.org/science/advanced-topics/sonar-equation/)) and define the water profile for this scene.

![alt text](images/scanner_panel_scanner_sidescan.png)

#### Water profile

The `water surface level` defines the z coordinate in your scene which is refered as a water depth of 0 meters. In the table below, you can fill in values for different water layers. Keep in mind to always start with a layer at 0m depth. This approach is used to quickly adjust the water level without the need to move the whole scene.

![alt text](images/water_surface.png)

Example: you set a water surface level of z = 10 and add three layers at a depth of 0m, 3m and 6m. This means there is a layer between 0-3 m, another one between 3-6 m and a last layers which starts at 6 m depth and is infinitely deep (until it hits the bottom). Related to the scene's z coordinate, this means that you have borders between the layers at z = 7 and z = 4. 

<br />

### Reflectivity

The minimum reflectivity needed to capture a reflected ray is approximated by the following model. At a distance of d<sub>min</sub> a reflectivity of r<sub>min</sub> is needed, while at d<sub>max</sub> the reflectivity needs to be greater than r<sub>max</sub> . Measurement below d<sub>min</sub> are captured as long as the reflectivity is >0. For distances above d<sub>max</sub> , no values are registered.

![alt text](images/reflectivity.png)

The following panel lets you define the parameters.

![alt text](images/scanner_panel_reflectivity.png)

You can set the minimum and maximum reflectivity for the scene's targets at given distances. 

The maximum reflection depth defines how often a ray can be reflected on surfaces before it gets discarded.

The reflectivity is defined by the material:

#### Diffuse material

![alt text](images/diffuse_material.png)

Diffuse material can be defined by changing the `Base Color` attribute of the `Princinpled BSDF` shader. The reflectivity is taken from the `alpha` parameter of a materials color.

#### Texture

![alt text](images/texture_material.png)

To use a texture, add an `Image texture` node and link it to the input of `Base Color`. 

#### Glass

![alt text](images/glass_material.png)

To model glass objects, simply use the `Glass BSDF` shade and set the correct index of refraction with the `IOR` attribute.

#### Mirror

![alt text](images/mirror_material.png)

To simulate a fully reflecting surface, you can set the `Metallic` attribute of the `Princinpled BSDF` shader to 1.0.

<br />

### Classification

Objects can beclassified in the two following ways:

#### Via custom properties

<img src="images/category_custom_property.png" width="600">

Select an object to add a custom property `categoryID` to set the main category (here: chair) and `partID` to set the subcategory (here: legs/plate). If no `categoryID` is provided, the object name is used as the category name instead. If no `partID` is given, the material name is used (see below).

#### Via different materials

<img src="images/category_material.png" width="600">

The main category has to be set like explained above via `categoryID`. To differentiante parts within a single object, you can select the faces in edit mode and assign a specific material (here: leg/plate). Each subobject with the same material is treated as one category, even if they belong to different objects.

<br />

### Animation

![alt text](images/scanner_panel_animation.png)

The settings in this panel correspond to the values inside Blenders `Output Properties` tab. You can define the range of frames, the number of skipped frames in each animation step and the number of frames per second (relevant for rotating scanners). Any techniques inside Blender to simulate motion and physics can be applied.

<br />

### Noise

![alt text](images/scanner_panel_noise.png)

#### Constant offset

The constant offsets are applied to each measurement without any variation. You can choose between an absolute offset which is the same for each distance or a relative offset as percentage of the distance.

#### Random offset

To simulate random errors during the measurement, you can specify the distribution with the given parameters.

<br />

### Weather simulation

#### Rain

![alt text](images/scanner_panel_rain.png)

To simulate rain, just set the amount of rain in millimeters per hour (see [this paper](https://doi.org/10.3390/electronics8010089)).

#### Dust

![alt text](images/scanner_panel_dust.png)

For dust simulation, you can set the parameters to define a dust cloud starting at a given distance and with a given length (see [this paper](https://doi.org/10.1002/rob.21701)).

<br />

### Visualization

![alt text](images/scanner_panel_visualization.png)

If this setting is enabled, the generated point cloud is added the to Blender scene.

<br />

### Export

![alt text](images/scanner_panel_export.png)

#### Raw data

This add-on can output the generated point clouds as [.hdf5](https://en.wikipedia.org/wiki/Hierarchical_Data_Format), [.csv](https://en.wikipedia.org/wiki/Comma-separated_values), [.ply](https://en.wikipedia.org/wiki/PLY_(file_format)) and [.las](https://en.wikipedia.org/wiki/LAS_file_format) files.

The option `Export single frames` defines if each animation frame should be exported in a separat file or if all steps are exported into a single file.

#### Iages

In the case of `time of flight` sensors, you can furthermore export the rendered image along with a segemented image (including [pascal voc object descriptions](http://host.robots.ox.ac.uk/pascal/VOC/)) and a depthmap. You can specify the value range for the depthmap. All depth values at the minimum are white, whereas values at or above the maximum value appear black. Color values in-between are linearly interpolated.



<br />

### DEBUG

These options are only meant for debugging the add-on. Use them with caution as adding output/line to the process can lead to significant perfomance issues!

<br /><br />

## Validation Workflows

For command-line and scripted use, the repository now centers on the validation
runner scripts in `scripts/` rather than the older long-form inline API examples.
See `scripts/README.md` for the script map and `docs/repo-map.md` for where the
 major code paths live.

### Fixed-scene validation experiments

For repeatable multi-sensor validation runs without the GUI, this fork also exposes a direct Python helper:

```python
import bpy
import range_scanner

result = range_scanner.ui.user_interface.scan_multi_sensor(
    bpy.context,
    primary_config={
        "scannerObject": "Camera",
        "scannerType": "rotating",
        "fovX": 30.0,
        "fovY": 40.0,
        "xStepDegree": 0.2,
        "yStepDegree": 0.33,
        "rotationsPerSecond": 20.0,
        "reflectivityLower": 0.0,
        "distanceLower": 0.0,
        "reflectivityUpper": 0.0,
        "distanceUpper": 99999.9,
        "maxReflectionDepth": 10,
    },
    secondary_config={
        "scannerObject": "Camera.001",
        "scannerType": "sideScan",
        "maxDistance": 100.0,
        "fovSonar": 135.0,
        "sonarStepDegree": 0.25,
        "sonarMode3D": True,
        "sonarKeepRotation": False,
        "sourceLevel": 220.0,
        "noiseLevel": 63.0,
        "directivityIndex": 20.0,
        "processingGain": 10.0,
        "receptionThreshold": 10.0,
        "simulateWaterProfile": False,
        "surfaceHeight": 10.0,
    },
    common_config={
        "dataFilePath": "//output",
        "dataFileName": "baseline_clean",
        "addMesh": False,
        "exportCSV": False,
        "exportLAS": False,
        "exportHDF": False,
        "exportPLY": False,
        "enableAnimation": False,
        "frameStart": 1,
        "frameEnd": 1,
        "frameStep": 1,
        "frameRate": 1,
        "addNoise": False,
        "addConstantNoise": False,
        "noiseType": "gaussian",
        "mu": 0.0,
        "sigma": 0.01,
        "noiseAbsoluteOffset": 0.0,
        "noiseRelativeOffset": 0.0,
        "simulateRain": False,
        "rainfallRate": 0.0,
        "debugLines": False,
        "debugOutput": False,
        "outputProgress": True,
        "measureTime": False,
        "singleRay": False,
        "joinMeshes": False,
    },
    water_profile=[],
)

print(result["report_path"])
```

For batch experiments on one fixed `.blend` scene, use `scripts/run_fixed_scene_validation_batch.py` with `scripts/fixed_scene_validation.example.json`, `scripts/fixed_scene_noise_sweep.example.json`, or `scripts/fixed_scene_pose_sweep.example.json` as a template. The batch runner launches a fresh Blender process per trial so destructive scene mutations do not leak between runs.

```bash
python3 scripts/run_fixed_scene_validation_batch.py \
  --blender blender \
  --scene path/to/fixed_scene.blend \
  --config scripts/fixed_scene_noise_sweep.example.json
```

After the batch finishes, it writes:

* one per-trial validation report for each run
* one batch summary JSON
* one batch summary CSV

The summary files are written into the configured output directory by default, unless you pass `--summary-dir`.

Pose sweeps are supported with `primary_pose_defaults`, `secondary_pose_defaults`, `primary_pose`, and `secondary_pose` entries in the batch config. Each pose can include:

* `location`: `[x, y, z]`
* `rotation_euler_rad`: `[rx, ry, rz]`
* `rotation_euler_deg`: `[rx, ry, rz]`
* `look_at`: `[x, y, z]`

If you want the sensors to keep aiming at a shared target while moving between trials, set `look_at` in the pose defaults and vary only `location` per trial.

To automate a full validation pass that first selects the best static pose and then runs a noise sweep on that pose, use `scripts/run_pose_then_noise_validation.py` with `scripts/pose_then_noise_validation.example.json`:

```bash
python3 scripts/run_pose_then_noise_validation.py \
  --blender "/Applications/Blender.app/Contents/MacOS/Blender" \
  --scene generated_scenes/dual_sensor_scene_42.blend \
  --config scripts/pose_then_noise_validation.example.json
```

By default, the combined workflow ranks pose trials by `point_count`, keeps the top pose, runs the configured noise sweep for it, and writes a combined CSV/JSON summary.

For more stable validation runs, you can set `common_defaults.noiseSeed` in the noise sweep config. That makes repeated runs with the same scene/config produce repeatable measurement-noise realizations.

Pose selection also supports a `composite` strategy with weighted terms like `point_count`, `sensor_balance`, `rmse`, and `p95`. This is useful when you do not want the selector to over-favor lidar-heavy poses solely because they produce more total hits.

You can also scale this workflow across multiple seeded scenes with `scripts/run_multi_seed_pose_then_noise_validation.py` and `scripts/multi_seed_pose_then_noise_validation.example.json`:

```bash
python3 scripts/run_multi_seed_pose_then_noise_validation.py \
  --blender "/Applications/Blender.app/Contents/MacOS/Blender" \
  --config scripts/multi_seed_pose_then_noise_validation.example.json
```

This multi-seed runner can generate the seeded scenes automatically, run the pose-then-noise workflow for each seed, and write an aggregate CSV/JSON summary across scenes.

If you want to move toward mapping-style validation, the next step is to animate the two sensors deterministically and evaluate the accumulated point cloud over the full frame range. You can create an animated scene from an existing seeded scene with:

```bash
"/Applications/Blender.app/Contents/MacOS/Blender" generated_scenes/dual_sensor_scene_42.blend \
  --background \
  --python scripts/create_animated_validation_scene.py -- \
  --output-scene generated_scenes/dual_sensor_scene_42_animated.blend \
  --frame-start 1 \
  --frame-end 20
```

Then run accumulated animated validation with:

```bash
python3 scripts/run_fixed_scene_validation_batch.py \
  --blender "/Applications/Blender.app/Contents/MacOS/Blender" \
  --scene generated_scenes/dual_sensor_scene_42_animated.blend \
  --config scripts/animated_accumulated_validation.example.json
```

When `enableAnimation` is true, the current validation report treats all captured points across the configured frame range as one accumulated pre-fusion map and compares that combined result against the ground-truth surface.

To compare several deterministic animation paths, use `scripts/run_animated_path_validation.py` with `scripts/animated_path_validation.example.json`:

```bash
python3 scripts/run_animated_path_validation.py \
  --blender "/Applications/Blender.app/Contents/MacOS/Blender" \
  --base-scene generated_scenes/dual_sensor_scene_42.blend \
  --config scripts/animated_path_validation.example.json
```

This creates one animated `.blend` per path, runs accumulated validation on each path, and writes an aggregate CSV/JSON summary so you can compare mapping-oriented trajectories directly.

Waypoint-based paths are supported, so you can model more traditional coverage trajectories such as a lawnmower or snake pattern across a rectangular area. In `scripts/animated_path_validation.example.json`, the `lawnmower_cover` path uses `primary_waypoints` and `secondary_waypoints` to sweep back and forth across the scene while keeping both sensors aimed at a common target region.

### Seeded random dual-sensor scenes

If you do not already have a scene that works for both sensors, generate one with:

```bash
blender --background --python scripts/generate_seeded_dual_sensor_scene.py -- \
  --seed 42 \
  --output-dir generated_scenes
```

This creates:

* `generated_scenes/dual_sensor_scene_42.blend`
* `generated_scenes/dual_sensor_scene_42.metadata.json`

The generated scene includes:

* `Camera` for the primary sensor
* `Camera.001` for the secondary sensor
* above-water and underwater randomized mesh targets with materials
* a water surface at `z = 0.0`

To generate several seeded scenes at once:

```bash
python3 scripts/generate_seeded_dual_sensor_batch.py \
  --blender blender \
  --output-dir generated_scenes \
  --seeds 42 43 44
```

Then run your fixed-scene validation batch against one of the generated `.blend` files.

<br /><br />

## Visualization

All generated data can be shown inside Blender by enabling the `Add datapoint mesh` option inside the `Visualization` submenu. It is also possible to visualize the data as rendered, segmented/labeled and depth images (see [Export](#export)). 

To render .las files the tool [CloudCompare](https://www.cloudcompare.org/) can be used.

You can further use [Potree Desktop](https://github.com/potree/PotreeDesktop) to visualize the raw data. The generated .las files can be converted automatically by dragging it into the window or manually by using the [Potree Converter](https://github.com/potree/PotreeConverter):

```
 .\path\to\potree\PotreeConverter.exe .\path\to\data\data.las -o .\output_directory
```

This will generate a `cloud.js` file which you can drag and drop inside the Potree viewer. 

<br /><br />

## Examples

See [`example_scenes/`](./example_scenes/).

The curated example scene for this fork is `example_scenes/sonar_example.blend`.
The older legacy example scenes were removed to keep the repository focused on
the current validation and dual-sensor workflow.

<br /><br />

## Development

This add-on is developed using Visual Studio Code and the Blender extension [blender_vscode](https://github.com/JacquesLucke/blender_vscode).

To run the add-on in debug mode, use the extension and start the addon from there.

If you want to have autocomplete features, consider installing the [fake-bpy-module](https://github.com/nutti/fake-bpy-module) package.

Feel free to fork, modify and improve our work! We would also appreciate to receive contributions in for of pull requests. For that it would be a good idea to open an issue with your idea.

<br /><br />

## About

### Original Project

This add-on was originally developed by Lorenzo Neumann at [TU Bergakademie Freiberg](https://tu-freiberg.de/fakult1/inf).

Master thesis: *Lorenzo Neumann. "Generation of 3D training data for AI applications by simulation of ranging methods in virtual environments", 2020*.

Paper: Reitmann, S.; Neumann, L.; Jung, B. BLAINDER—A Blender AI Add-On for Generation of Semantically Labeled Depth-Sensing Data. Sensors 2021, 21, 2144. https://doi.org/10.3390/s21062144

### This Fork

Updated for Blender 5.0 by Robin Ede at [Texas A&M University](https://www.tamu.edu/) as part of a capstone group project.

Changes in this fork:
- Migrated from legacy `bl_info` addon format to Blender 5.0 extension format (`blender_manifest.toml`)
- Removed deprecated material API calls (`material.diffuse_color`, `material.metallic`)
- Simplified version checks for Blender 5.0+ compatibility
- Updated installation process for the new extension system

<br /><br />

## License

Copyright (C) 2020-2024 Lorenzo Neumann (original)
Copyright (C) 2026 Robin Ede (Blender 5.0 updates)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see https://www.gnu.org/licenses/.

<br />

A brief summary of this license can be found here: https://tldrlegal.com/license/gnu-general-public-license-v3-(gpl-3)
<br />

Commercial license: If you want to use this software without complying with the conditions of the GPL-3.0 license, you can get a custom license. If you wish to obtain such a license, please feel free to contact me at <a href="mailto:lorenzo.neumann@informatik.tu-freiberg.de?subject=[BLAINDER] License request">lorenzo.neumann@informatik.tu-freiberg.de</a> or via an issue.

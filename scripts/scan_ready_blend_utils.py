import sys
from pathlib import Path

import bpy

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import range_scanner


SONAR_EXAMPLE_CONFIG = {
    "activeScanner": "primary",
    "common": {
        "addMesh": True,
        "addNoise": False,
        "joinMeshes": False,
        "enableAnimation": True,
        "frameStart": 1,
        "frameEnd": 300,
        "frameStep": 1,
        "frameRate": 10.0,
        "dataFilePath": "",
        "dataFileName": "",
    },
    "primary": {
        "scannerObject": "camera",
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
        "fovX": 120.0,
        "fovY": 50.0,
        "xStepDegree": 1.0,
        "yStepDegree": 2.0,
        "rotationsPerSecond": 20.0,
    },
    "secondary": {
        "scannerObject": "camera.001",
        "scannerType": "rotating",
        "fovX": 30.0,
        "fovY": 90.0,
        "xStepDegree": 1.0,
        "yStepDegree": 1.0,
        "rotationsPerSecond": 10.0,
        "maxDistance": 100.0,
        "fovSonar": 45.0,
        "sonarStepDegree": 1.0,
        "sonarMode3D": False,
        "sonarKeepRotation": False,
        "sourceLevel": 200.0,
        "noiseLevel": 50.0,
        "directivityIndex": 20.0,
        "processingGain": 10.0,
        "receptionThreshold": 10.0,
        "simulateWaterProfile": False,
        "surfaceHeight": 10.0,
    },
}


def ensure_range_scanner_registered():
    if not hasattr(bpy.types.Scene, "scannerProperties"):
        range_scanner.register()


def resolve_scene_object(scene, object_name, field_name):
    obj = scene.objects.get(object_name)
    if obj is None:
        raise ValueError(
            "Object '%s' for %s was not found in the scene." % (object_name, field_name)
        )
    return obj


def apply_property_values(properties, values, suffix=""):
    for key, value in values.items():
        property_name = key + suffix
        if property_name in ("scannerObject", "scannerObject2"):
            continue
        if not hasattr(properties, property_name):
            raise ValueError("Unknown scanner property '%s'." % property_name)
        setattr(properties, property_name, value)


def replace_water_profile(scene, water_profile):
    while len(scene.custom) > 0:
        scene.custom.remove(0)

    for layer in water_profile or []:
        item = scene.custom.add()
        item.depth = layer["depth"]
        item.speed = layer["speed"]
        item.density = layer["density"]


def water_surface_height(scene, fallback=0.0):
    water = scene.objects.get("WaterSurface")
    if water is None:
        return fallback
    return float(water.matrix_world.translation.z)


def default_output_settings(scene):
    blend_name = Path(bpy.data.filepath).stem or scene.name or "scan"
    safe_name = blend_name.replace(" ", "_")
    return {
        "dataFilePath": "//output/%s" % safe_name,
        "dataFileName": safe_name,
    }


def build_dual_sensor_config(
    primary_object_name="Camera",
    secondary_object_name="Camera.001",
    frame_start=1,
    frame_end=1,
    frame_step=1,
    frame_rate=10.0,
    surface_height=0.0,
    enable_animation=False,
    add_noise=True,
):
    return {
        "activeScanner": "primary",
        "common": {
            "addMesh": True,
            "addNoise": add_noise,
            "noiseType": "gaussian",
            "mu": 0.0,
            "sigma": 0.03,
            "joinMeshes": False,
            "enableAnimation": enable_animation,
            "frameStart": frame_start,
            "frameEnd": frame_end,
            "frameStep": frame_step,
            "frameRate": frame_rate,
            "dataFilePath": "",
            "dataFileName": "",
            "exportCSV": False,
            "exportLAS": False,
            "exportHDF": False,
            "exportPLY": False,
            "exportSingleFrames": False,
            "outputProgress": True,
            "measureTime": False,
            "debugLines": False,
            "debugOutput": False,
            "singleRay": False,
        },
        "primary": {
            "scannerObject": primary_object_name,
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
        "secondary": {
            "scannerObject": secondary_object_name,
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
            "surfaceHeight": surface_height,
        },
        "water_profile": [],
    }


def example_scene_config(scene, blend_path):
    blend_name = Path(blend_path).name
    if blend_name == "sonar_example.blend":
        return SONAR_EXAMPLE_CONFIG
    if blend_name == "straight_push_degraded.blend":
        return build_dual_sensor_config(
            frame_end=max(scene.frame_end, 30),
            frame_rate=10.0,
            surface_height=water_surface_height(scene, 0.0),
            enable_animation=True,
            add_noise=True,
        )
    if blend_name == "chesapeake_bay_300m_real_bathy.blend":
        cfg = build_dual_sensor_config(
            frame_end=max(scene.frame_end, 236),
            frame_rate=10.0,
            surface_height=water_surface_height(scene, 0.0),
            enable_animation=True,
            add_noise=True,
        )
        # Chesapeake is sonar-only validation — promote sonar to primary so
        # pressing "Start Scan" runs the sonar directly without a sensor swap.
        cfg["primary"] = cfg.pop("secondary")
        cfg["primary"]["maxDistance"] = 350.0
        return cfg
    raise ValueError("No scan-ready preset is defined for %s" % blend_name)


def apply_scan_ready_config(scene, config):
    ensure_range_scanner_registered()

    properties = scene.scannerProperties
    common_values = dict(config.get("common", {}))
    primary_values = dict(config["primary"])
    secondary_values = dict(config.get("secondary", {}))

    primary_object_name = primary_values.pop("scannerObject")
    properties.scannerObject = resolve_scene_object(
        scene, primary_object_name, "primary scannerObject"
    )

    secondary_object_name = secondary_values.pop("scannerObject", None)
    if secondary_object_name is None:
        properties.scannerObject2 = None
        properties.scannerType2 = "static"
    else:
        properties.scannerObject2 = resolve_scene_object(
            scene, secondary_object_name, "secondary scannerObject"
        )
        properties.scannerType2 = secondary_values["scannerType"]

    properties.scannerType = primary_values["scannerType"]
    properties.activeScanner = config.get("activeScanner", "primary")

    if not common_values.get("dataFilePath"):
        common_values["dataFilePath"] = default_output_settings(scene)["dataFilePath"]
    if not common_values.get("dataFileName"):
        common_values["dataFileName"] = default_output_settings(scene)["dataFileName"]

    apply_property_values(properties, common_values)
    apply_property_values(properties, primary_values)
    if properties.scannerObject2 is not None:
        apply_property_values(properties, secondary_values, "2")

    replace_water_profile(scene, config.get("water_profile", []))

    if hasattr(properties, "frameStart"):
        scene.frame_start = properties.frameStart
    if hasattr(properties, "frameEnd"):
        scene.frame_end = properties.frameEnd
    if hasattr(properties, "frameRate"):
        scene.render.fps = max(1, int(round(properties.frameRate)))
    scene.camera = properties.scannerObject
    scene.frame_set(scene.frame_start)

    return properties

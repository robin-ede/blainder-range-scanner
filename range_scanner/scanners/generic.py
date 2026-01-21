import bpy
import sys
import bmesh
from mathutils.bvhtree import BVHTree
import numpy as np
from . import hit_info
import os
import time

from enum import Enum
ScannerType = Enum('ScannerType', 'static rotating sideScan')

from . import lidar
from . import sonar
from ..export import exporter
from .. import material_helper

SWAPPABLE_PROPERTIES = [
    'fovX', 'fovY', 'xStepDegree', 'yStepDegree', 'rotationsPerSecond',
    'resolutionX', 'resolutionY', 'resolutionPercentage',
    'fovSonar', 'sonarStepDegree', 'sonarMode3D', 'sonarKeepRotation',
    'sourceLevel', 'noiseLevel', 'directivityIndex', 'processingGain',
    'receptionThreshold', 'maxDistance', 'simulateWaterProfile', 'surfaceHeight',
    'reflectivityLower', 'distanceLower', 'reflectivityUpper', 'distanceUpper', 'maxReflectionDepth',
    'addNoise', 'mu', 'sigma', 'noiseType', 'noiseAbsoluteOffset', 'noiseRelativeOffset',
    'simulateRain', 'rainfallRate', 'simulateDust', 'particleRadius', 'particlesPcm', 'dustCloudStart', 'dustCloudLength'
]

class TemporaryPropertyOverride:
    def __init__(self, properties, property_names, suffix='2'):
        self.properties = properties
        self.property_names = property_names
        self.suffix = suffix
        self.original_values = {}

    def __enter__(self):
        for prop in self.property_names:
            secondary_prop = prop + self.suffix
            if hasattr(self.properties, prop) and hasattr(self.properties, secondary_prop):
                self.original_values[prop] = getattr(self.properties, prop)
                setattr(self.properties, prop, getattr(self.properties, secondary_prop))
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        for prop, value in self.original_values.items():
            setattr(self.properties, prop, value)

# source: https://blender.stackexchange.com/a/30739/95167
def updateProgress(job_title, progress):
    length = 20 # modify this to change the length
    block = int(round(length*progress))
    msg = "\r{0}: [{1}] {2}%".format(job_title, "#"*block + "-"*(length-block), round(progress*100, 2))
    if progress >= 1: msg += " DONE\r\n"
    sys.stdout.write(msg)
    sys.stdout.flush()

def addLine(v1, v2):
    mesh = bpy.data.meshes.new(name='mesh object')

    bm = bmesh.new()
    v1 = bm.verts.new(v1)  # add a new vert
    v2 = bm.verts.new(v2)  # add a new vert
    
    bm.edges.new((v1, v2))

    # make the bmesh the object's mesh
    bm.to_mesh(mesh)  
    bm.free()  # always do this when finished
        
    # We're done setting up the mesh values, update mesh object and 
    # let Blender do some checks on it
    mesh.update()
    mesh.validate()

    # Create Object whose Object Data is our new mesh
    obj = bpy.data.objects.new('ray', mesh)

    # Add *Object* to the scene, not the mesh
    scene = bpy.context.scene
    scene.collection.objects.link(obj)


def getTargetIndices(targets, debugOutput):
    # we need indices to store which point belongs to which object
    # there are to types of indices: 
    #   TOP LEVEL: the category of the top level object in our scene, e.g. chair, table
    #   CHILD LEVEL: the category of the child object, e.g. leg, plate

    # for that we iterate over all "groupID" custom properties and put them in the dictionary
    categoryIDs = {}
    partIDs = {}

    categoryIndex = 0
    partIndex = 0

    for target in targets:
        if not "categoryID" in target:
            # the custom property is not set, fallback is the targets name
            if debugOutput:
                print("WARNING: no categoryID given for target %s! Using name instead." % target.name)  
            target["categoryID"] = target.name

        categoryID = target["categoryID"]

        # only add the index if this group is not already in our dictionary
        if not categoryID in categoryIDs:
            categoryIDs[categoryID] = categoryIndex
            categoryIndex += 1


        if not "partID" in target:
            # the custom property is not set
            # in this case, the fallback is the material index for the given point on the mesh
            if debugOutput:
                print("WARNING: no partID given for target %s! Using material instead." % target.name)  
            
        else:
            partID = target["partID"]

            # only add the index if this group is not already in our dictionary
            if not partID in partIDs:
                partIDs[partID] = partIndex
                partIndex += 1
       

    # add all materials to dictionary
    for material in bpy.data.materials:
        materialName = material.name

        if not materialName in partIDs:
            partIDs[materialName] = partIndex
            partIndex += 1

    return (categoryIDs, partIDs)

def addMeshToScene(name, values, useNoiseLocation):
    # Create new mesh to store all measurements as points
    mesh = bpy.data.meshes.new(name='created mesh')
    bm = bmesh.new()        

    # iterate over all possible hits
    if useNoiseLocation:
        for hit in values:                
            bm.verts.new((hit.noiseLocation.x, hit.noiseLocation.y, hit.noiseLocation.z))
    else:
        for hit in values:                
            bm.verts.new((hit.location.x, hit.location.y, hit.location.z))

    # make the bmesh the object's mesh
    bm.to_mesh(mesh)  
    bm.free()  # always do this when finished
        
    # We're done setting up the mesh values, update mesh object and 
    # let Blender do some checks on it
    mesh.update()
    mesh.validate()

    # Create Object whose Object Data is our new mesh
    obj = bpy.data.objects.new(name, mesh)

    # Add *Object* to the scene, not the mesh
    scene = bpy.context.scene
    scene.collection.objects.link(obj)

def getClosestHit(targets, trees, origin, direction, maxRange, debugOutput, debugLines):
    closestLocation = None
    closestFaceNormal = None
    closestFaceIndex = None
    closestDistance = maxRange
    closestTarget = None

    # iterate over all targets to find the closest hit
    for target in targets:
        if debugOutput:
            print("Scanning target ", target.name, "...")

        # perform the actual ray casting
        # see: https://docs.blender.org/api/current/mathutils.bvhtree.html#mathutils.bvhtree.BVHTree.ray_cast
        #      https://github.com/blender/blender/blob/master/source/blender/blenlib/BLI_kdopbvh.h#L81
        location, faceNormal, faceIndex, distance = trees[target][0].ray_cast(origin, direction, closestDistance)

        # we use the current closest distance as maximum range, because we don't need to consider geometry which 
        # is further away than the current closest hit

        # if there was a hit and it is closer to the origin, update closest hit
        # but we need a workaround for rounding errors:
        # sometimes when we fire a ray from an object, that ray immediately hits that
        # same object again, so we just ignore it
        if distance is not None and distance < closestDistance:
            if debugOutput:
                print("Old hit ", closestLocation, closestFaceNormal, closestFaceIndex, closestDistance, closestTarget)
                print("New hit ", location, faceNormal, faceIndex, distance, target)

            closestLocation = location
            closestFaceNormal = faceNormal
            closestFaceIndex = faceIndex
            closestDistance = distance
            closestTarget = target
            
    if closestLocation is not None:
        if debugLines:
            addLine(origin, closestLocation)
        
        return hit_info.HitInfo(closestLocation, closestFaceNormal, closestFaceIndex, closestDistance, closestTarget)
    else:
        return None

# remove invalid characters
# source https://blender.stackexchange.com/a/104877
def removeInvalidCharatersFromFileName(name):  
    for char in " !@#$%^&*(){}:\";'[]<>,.\\/?":
        name = name.replace(char, '_')
    return name.lower().strip()

def startScan(context, properties, objectName):   
    if objectName is None:
        cleanedFileName = removeInvalidCharatersFromFileName(properties.dataFileName)
    else:
        cleanedFileName = removeInvalidCharatersFromFileName("%s_%s" % (properties.dataFileName, objectName))
    
    if not cleanedFileName == properties.dataFileName:
        print("WARNING: File name contains invalid characters. New file name: %s" % cleanedFileName)


    if properties.singleRay:
        allTargets = [properties.targetObject]
    else:
        allTargets = []
        for viewLayer in bpy.context.scene.view_layers:
            for obj in viewLayer.objects:
                # get all visible objects
                # filters:
                # - object has some kind of geometry
                # - is not excluded
                # - has a material set
                if obj.type == 'MESH' and \
                    obj.hide_get() == False and \
                    obj.active_material != None:
                        allTargets.append(obj)

    if properties.scannerObject == None:
        print("No scanner object selected!")
        return {'FINISHED'}   

    if properties.measureTime:
            startTime = time.time()

    targets = []
    materialMappings = {}

    for target in allTargets:
        # we need to know which material belongs to which face, get the mapping for each target
        if len(target.material_slots) == 0:
            if properties.debugOutput:
                print("No material set for object %s! Skipping..." % target.name)
            continue
        
        # Blender's modifiers can change an objetc's shape although it seems
        # already modified in the viewport
        # so wen need to apply all modifiers here
        context.view_layer.objects.active = target

        for modifier in target.modifiers:
            # Blender 5.0+ uses the simplified modifier_apply API
            bpy.ops.object.modifier_apply(modifier=modifier.name)

        try:
            targetMaterials = material_helper.getTargetMaterials(properties.debugOutput, target)
        except ValueError as e:
            print(e)
            print(f"The target object with name {target.name} will be ignored! ")
            continue

        targets.append(target)

        # get the face->material mappings for the current object
        targetMappings =  material_helper.getFaceMaterialMapping(target.data)
        
        materialMappings[target] = (targetMaterials, targetMappings)

    (categoryIDs, partIDs) = getTargetIndices(targets, properties.debugOutput)

    if properties.debugOutput:
        print("CategoryIDs ", categoryIDs)
        print("PartIDs ", partIDs)

    if properties.scannerType == ScannerType.sideScan.name:
        if properties.scannerObject.matrix_world.translation.z > properties.surfaceHeight:
            print("ERROR: Sensor is above water level!")
            return {'FINISHED'}

        if properties.simulateWaterProfile:
            # as the fill value (black) is a tuple, we need some special packing
            # see: https://stackoverflow.com/a/40711408/13440564
            value = np.empty((), dtype=object)
            value[()] = (0.0, 0.0, 0.0) 

            depthList = np.full(len(context.scene.custom.items()), value)

            for index, item in enumerate(context.scene.custom.items()):                
                # store all values in a new array
                # the depth is measured relative to the water surface level
                depthList[index] = (properties.surfaceHeight - item[1].depth, item[1].speed, item[1].density)
        else:
            depthList = []

        sonar.performScan(context, 
                    properties.scannerType, properties.scannerObject,
                    properties.maxDistance,
                    properties.fovSonar, properties.sonarStepDegree, properties.sonarMode3D, properties.sonarKeepRotation,
                    properties.sourceLevel, properties.noiseLevel, properties.directivityIndex, properties.processingGain, properties.receptionThreshold,   
                    properties.simulateWaterProfile, depthList,   
                    properties.addNoise, properties.noiseType, properties.mu, properties.sigma, properties.addConstantNoise, properties.noiseAbsoluteOffset, properties.noiseRelativeOffset,
                    properties.addMesh,
                    properties.exportLAS, properties.exportHDF, properties.exportCSV, properties.exportPLY, properties.exportSingleFrames,
                    properties.dataFilePath, cleanedFileName,
                    properties.debugLines, properties.debugOutput, properties.outputProgress, properties.measureTime, properties.singleRay, properties.destinationObject, properties.targetObject,
                    properties.enableAnimation, properties.frameStart, properties.frameEnd, properties.frameStep,
                    targets, materialMappings,
                    categoryIDs, partIDs)

    else:
        if properties.enableAnimation:
            # read the needed camera settings
            # alternative: get needed values from the main Blender GUI ('Output Properties' tab on the right)
            firstFrame = properties.frameStart  # bpy.context.scene.frame_start
            lastFrame = properties.frameEnd     # bpy.context.scene.frame_end
            frameStep = properties.frameStep    # bpy.context.scene.frame_step
            frameRate = properties.frameRate    # bpy.context.scene.render.fps / bpy.context.scene.render.fps_base

            # calculate the angle which the sensor covers in each frame
            angularFractionPerFrame = properties.rotationsPerSecond / frameRate * (properties.fovX)

            # more than 360° (one rotation) makes no sense, as we would compute some rays more than once
            if angularFractionPerFrame > 360.0:
                angularFractionPerFrame = 360.0

        else:
            firstFrame = bpy.context.scene.frame_current
            lastFrame = bpy.context.scene.frame_current
            frameStep = 1
            frameRate = 1

            angularFractionPerFrame = properties.fovX

        if properties.scannerType == ScannerType.rotating.name or properties.scannerType == ScannerType.sideScan.name:
            stepsX = properties.xStepDegree
            stepsY = properties.yStepDegree
        elif properties.scannerType == ScannerType.static.name:
            stepsX = int(properties.resolutionX * (properties.resolutionPercentage / 100))
            stepsY = int(properties.resolutionY * (properties.resolutionPercentage / 100))
        else:
            print("Unsupported scanner type %s!" % properties.scannerType)
            return {'FINISHED'}





        if properties.scannerType == ScannerType.rotating.name:
            # defining sensor properties
            # [-180, 180] degree
            xSteps = angularFractionPerFrame / stepsX + 1

            # [-90, 90] degree
            ySteps = properties.fovY / stepsY + 1

            totalNumberOfRays = int(xSteps) * int(ySteps)
        elif properties.scannerType == ScannerType.static.name:
            totalNumberOfRays = stepsX * stepsY
        else:
            print("ERROR: Unknown scanner type %s!" % properties.scannerType)
            return {'FINISHED'}

        frameRange = range(firstFrame, lastFrame + 1, frameStep)

        # array to store hit information
        # we don't know how many of our rays will actually hit an object, so we allocate
        # memory for the worst case of every ray hitting the scene
        # (TODO depending on the RAM usage, it might be a good idea to use some kind of caching/splitting)
        scannedValues = np.full(len(frameRange) * totalNumberOfRays, None, dtype=hit_info.HitInfo)

        startIndex = 0

        # graph needed for BVH tree
        depsgraph = context.evaluated_depsgraph_get()

        trees = {}

        for frameNumber in frameRange:
            print("Rendering frame %d..." % frameNumber)

            trees = generic.getBVHTrees(trees, targets, depsgraph)

            halfFOV = properties.fovX / 2.0

            # get the angle which the sensor needs to cover in the current frame
            if properties.enableAnimation and properties.scannerType == ScannerType.rotating.name:
                # if animation is enabled, only scan the area which is covered in the time of one frame
                intervalStart = -halfFOV + ((frameNumber - 1) * angularFractionPerFrame) % 360
                intervalEnd = intervalStart + angularFractionPerFrame
            else:
                # else, just scan from start to end
                intervalStart = -halfFOV
                intervalEnd = halfFOV

            if properties.enableAnimation:
                # set the current scene frame
                # IMPORTANT: don't use frame_current our the (internal) data might not
                # be updated before calculating the point data!
                bpy.context.scene.frame_set(frameNumber)

            numberOfHits = lidar.performScan(context, 
                                properties.scannerType, properties.scannerObject,
                                properties.reflectivityLower, properties.distanceLower, properties.reflectivityUpper, properties.distanceUpper, properties.maxReflectionDepth,
                                intervalStart, intervalEnd, properties.fovX, stepsX, properties.fovY, stepsY, properties.resolutionPercentage,
                                scannedValues, startIndex,
                                firstFrame, lastFrame, frameNumber, properties.rotationsPerSecond,
                                properties.addNoise, properties.noiseType, properties.mu, properties.sigma, properties.addConstantNoise, properties.noiseAbsoluteOffset, properties.noiseRelativeOffset,
                                properties.simulateRain, properties.rainfallRate,
                                properties.simulateDust, properties.particleRadius, properties.particlesPcm, properties.dustCloudLength, properties.dustCloudStart,
                                properties.addMesh and properties.exportSingleFrames,
                                properties.exportLAS and properties.exportSingleFrames, properties.exportHDF and properties.exportSingleFrames, properties.exportCSV and properties.exportSingleFrames, properties.exportPLY and properties.exportSingleFrames, 
                                properties.exportRenderedImage, properties.exportSegmentedImage, properties.exportPascalVoc, properties.exportDepthmap, properties.depthMinDistance, properties.depthMaxDistance, 
                                properties.dataFilePath, cleanedFileName,
                                properties.debugLines, properties.debugOutput, properties.outputProgress, properties.measureTime, properties.singleRay, properties.destinationObject, properties.targetObject,
                                targets, materialMappings,
                                categoryIDs, partIDs, trees, depsgraph)

            startIndex += numberOfHits

        print(f"Lidar scan produced {startIndex} hits")

        if not properties.exportSingleFrames:
            # we now have the final number of hits so we could shrink the array here
            # as explained here (https://stackoverflow.com/a/32398318/13440564), resizing
            # would cause a copy, so we slice the array instead
            slicedScannedValues = scannedValues[:startIndex]

            if properties.addMesh:
                addMeshToScene("real_values_frames_%d_to_%d" % (firstFrame, lastFrame), slicedScannedValues, False)

                if (properties.addNoise or properties.simulateRain):
                    addMeshToScene("noise_values_frames_%d_to_%d" % (firstFrame, lastFrame), slicedScannedValues, True)

            exportNoiseData = properties.addNoise or properties.simulateRain

            if len(slicedScannedValues) > 0:
                # setup exporter with our data
                if (properties.exportLAS) or (properties.exportHDF) or (properties.exportCSV) or (properties.exportPLY):
                    fileExporter = exporter.Exporter(properties.dataFilePath, "%s_frames_%d_to_%d" % (cleanedFileName, firstFrame, lastFrame), cleanedFileName, slicedScannedValues, targets, categoryIDs, partIDs, materialMappings, exportNoiseData, stepsX, stepsY)

                    print(fileExporter.fileName)

                    # export to each format
                    if properties.exportLAS:
                        fileExporter.exportLAS()

                    if properties.exportHDF:
                        fileExporter.exportHDF(fileNameExtra="_frames_%d_to_%d_merged" % (firstFrame, lastFrame))

                    if properties.exportCSV:
                        fileExporter.exportCSV()
                        
                    if properties.exportPLY:
                        fileExporter.exportPLY()
            else:
                print("No data to export!")
    if properties.measureTime:
        print("Scan time: %s s" % (time.time() - startTime))

def performMultiSensorScan(context, properties):
    """
    Perform a multi-sensor scan using both lidar and sonar sensors.
    First sensor is used for lidar (rotating/static), second for sonar (sideScan).
    Results are merged into a single CSV output file.
    """
    import time as time_module

    if properties.measureTime:
        startTime = time_module.time()

    # Store original settings
    originalScannerObject = properties.scannerObject
    originalScannerType = properties.scannerType

    # Collect all hits from both sensors
    allHits = []

    # Get targets and material mappings (shared between both scans)
    if properties.singleRay:
        allTargets = [properties.targetObject]
    else:
        allTargets = []
        for viewLayer in bpy.context.scene.view_layers:
            for obj in viewLayer.objects:
                if obj.type == 'MESH' and \
                    obj.hide_get() == False and \
                    obj.active_material != None:
                        allTargets.append(obj)

    targets = []
    materialMappings = {}

    for target in allTargets:
        if len(target.material_slots) == 0:
            continue

        context.view_layer.objects.active = target

        for modifier in target.modifiers:
            bpy.ops.object.modifier_apply(modifier=modifier.name)

        try:
            from .. import material_helper
            targetMaterials = material_helper.getTargetMaterials(properties.debugOutput, target)
        except ValueError as e:
            print(e)
            print(f"The target object with name {target.name} will be ignored! ")
            continue

        targets.append(target)
        targetMappings = material_helper.getFaceMaterialMapping(target.data)
        materialMappings[target] = (targetMaterials, targetMappings)

    (categoryIDs, partIDs) = getTargetIndices(targets, properties.debugOutput)

    print("=== Starting Multi-Sensor Scan ===")

    # Phase 1: Primary sensor scan
    print(f"\n--- Phase 1: Primary Sensor ({properties.scannerType}) ---")
    properties.scannerObject = originalScannerObject
    # scannerType is already set to primary's type

    if properties.scannerType == ScannerType.sideScan.name:
        primaryHits = runSonarScan(context, properties, targets, materialMappings, categoryIDs, partIDs)
        for hit in primaryHits: hit.sensor_id = "sonar"
    else:
        primaryHits = runLidarScan(context, properties, targets, materialMappings, categoryIDs, partIDs)
        for hit in primaryHits: hit.sensor_id = "lidar"
    allHits.extend(primaryHits)
    print(f"Primary scan complete: {len(primaryHits)} hits")

    # Phase 2: Secondary sensor scan
    secondaryType = properties.scannerType2
    print(f"\n--- Phase 2: Secondary Sensor ({secondaryType}) ---")

    # Swap to secondary scanner settings
    properties.scannerObject = properties.scannerObject2
    properties.scannerType = secondaryType

    with TemporaryPropertyOverride(properties, SWAPPABLE_PROPERTIES, '2'):
        if secondaryType == ScannerType.sideScan.name:
            secondaryHits = runSonarScan(context, properties, targets, materialMappings, categoryIDs, partIDs)
            for hit in secondaryHits: hit.sensor_id = "sonar"
        else:
            secondaryHits = runLidarScan(context, properties, targets, materialMappings, categoryIDs, partIDs)
            for hit in secondaryHits: hit.sensor_id = "lidar"
        allHits.extend(secondaryHits)
        print(f"Secondary scan complete: {len(secondaryHits)} hits")

    # Restore original settings
    properties.scannerObject = originalScannerObject
    properties.scannerType = originalScannerType

    # Phase 3: Merging Results and Creating Meshes
    print("\n--- Phase 3: Merging Results ---")
    print(f"Total hits from both sensors: {len(allHits)}")

    if len(allHits) > 0:
        # Convert to numpy array for export
        scannedValues = np.array(allHits, dtype=object)

        # Print summary for debugging
        sonar_count = sum(1 for h in allHits if h.sensor_id == "sonar")
        lidar_count = sum(1 for h in allHits if h.sensor_id == "lidar")
        print(f"Export summary: {sonar_count} sonar hits, {lidar_count} lidar hits")

        # Add meshes separately for each sensor
        if properties.addMesh:
            if len(primaryHits) > 0:
                primary_mesh_name = f"primary_{originalScannerType}_mesh"
                addMeshToScene(primary_mesh_name, np.array(primaryHits, dtype=hit_info.HitInfo), False)
                print(f"Created primary mesh: {primary_mesh_name} with {len(primaryHits)} points")
            
            if len(secondaryHits) > 0:
                secondary_mesh_name = f"secondary_{secondaryType}_mesh"
                addMeshToScene(secondary_mesh_name, np.array(secondaryHits, dtype=hit_info.HitInfo), False)
                print(f"Created secondary mesh: {secondary_mesh_name} with {len(secondaryHits)} points")

        # Export merged data
        cleanedFileName = removeInvalidCharatersFromFileName(properties.dataFileName + "_multi_sensor")
        exportNoiseData = properties.addNoise or properties.addConstantNoise

        if properties.exportLAS or properties.exportHDF or properties.exportCSV or properties.exportPLY:
            fileExporter = exporter.Exporter(
                properties.dataFilePath,
                cleanedFileName,
                cleanedFileName,
                scannedValues,
                targets,
                categoryIDs,
                partIDs,
                materialMappings,
                exportNoiseData,
                0, 0
            )

            if properties.exportLAS:
                fileExporter.exportLAS()

            if properties.exportHDF:
                fileExporter.exportHDF()

            if properties.exportCSV:
                fileExporter.exportCSV()

            if properties.exportPLY:
                fileExporter.exportPLY()
    else:
        print("No data to export!")

    if properties.measureTime:
        print(f"Multi-sensor scan time: {time_module.time() - startTime} s")

    print("=== Multi-Sensor Scan Complete ===")


def runLidarScan(context, properties, targets, materialMappings, categoryIDs, partIDs):
    """Run a lidar scan and return the hits as a list."""
    depsgraph = context.evaluated_depsgraph_get()
    trees = {}

    if properties.enableAnimation:
        firstFrame = properties.frameStart
        lastFrame = properties.frameEnd
        frameStep = properties.frameStep
        frameRate = properties.frameRate
        angularFractionPerFrame = properties.rotationsPerSecond / frameRate * properties.fovX
        if angularFractionPerFrame > 360.0:
            angularFractionPerFrame = 360.0
    else:
        firstFrame = bpy.context.scene.frame_current
        lastFrame = bpy.context.scene.frame_current
        frameStep = 1
        frameRate = 1
        angularFractionPerFrame = properties.fovX

    if properties.scannerType == ScannerType.rotating.name:
        stepsX = properties.xStepDegree
        stepsY = properties.yStepDegree
        xSteps = angularFractionPerFrame / stepsX + 1
        ySteps = properties.fovY / stepsY + 1
        totalNumberOfRays = int(xSteps) * int(ySteps)
    else:  # static
        stepsX = int(properties.resolutionX * (properties.resolutionPercentage / 100))
        stepsY = int(properties.resolutionY * (properties.resolutionPercentage / 100))
        totalNumberOfRays = stepsX * stepsY

    frameRange = range(firstFrame, lastFrame + 1, frameStep)
    scannedValues = np.full(len(frameRange) * totalNumberOfRays, None, dtype=object)
    startIndex = 0

    for frameNumber in frameRange:
        trees = getBVHTrees(trees, targets, depsgraph)

        halfFOV = properties.fovX / 2.0
        if properties.enableAnimation and properties.scannerType == ScannerType.rotating.name:
            intervalStart = -halfFOV + ((frameNumber - 1) * angularFractionPerFrame) % 360
            intervalEnd = intervalStart + angularFractionPerFrame
        else:
            intervalStart = -halfFOV
            intervalEnd = halfFOV

        if properties.enableAnimation:
            bpy.context.scene.frame_set(frameNumber)

        numberOfHits = lidar.performScan(
            context,
            properties.scannerType, properties.scannerObject,
            properties.reflectivityLower, properties.distanceLower, properties.reflectivityUpper, properties.distanceUpper, properties.maxReflectionDepth,
            intervalStart, intervalEnd, properties.fovX, stepsX, properties.fovY, stepsY, properties.resolutionPercentage,
            scannedValues, startIndex,
            firstFrame, lastFrame, frameNumber, properties.rotationsPerSecond,
            properties.addNoise, properties.noiseType, properties.mu, properties.sigma, properties.addConstantNoise, properties.noiseAbsoluteOffset, properties.noiseRelativeOffset,
            properties.simulateRain, properties.rainfallRate,
            properties.simulateDust, properties.particleRadius, properties.particlesPcm, properties.dustCloudLength, properties.dustCloudStart,
            False,  # addMesh - we'll add mesh at the end
            False, False, False, False,  # exports - we'll export at the end
            False, False, False, False, 0, 0,  # image exports
            properties.dataFilePath, properties.dataFileName,
            properties.debugLines, properties.debugOutput, properties.outputProgress, properties.measureTime, properties.singleRay, properties.destinationObject, properties.targetObject,
            targets, materialMappings,
            categoryIDs, partIDs, trees, depsgraph
        )

        startIndex += numberOfHits

    # Return only the valid hits as a list
    return [hit for hit in scannedValues[:startIndex] if hit is not None]


def runSonarScan(context, properties, targets, materialMappings, categoryIDs, partIDs):
    """Run a sonar scan and return the hits as a list."""
    # Check sonar-specific requirements
    if properties.scannerObject.matrix_world.translation.z > properties.surfaceHeight:
        print("WARNING: Sonar sensor is above water level!")
        return []

    # Prepare water profile depth list
    if properties.simulateWaterProfile:
        value = np.empty((), dtype=object)
        value[()] = (0.0, 0.0, 0.0)
        depthList = np.full(len(context.scene.custom.items()), value)

        for index, item in enumerate(context.scene.custom.items()):
            depthList[index] = (properties.surfaceHeight - item[1].depth, item[1].speed, item[1].density)
    else:
        depthList = []

    # Run sonar scan and capture results directly
    return runSonarScanWithCapture(context, properties, targets, materialMappings, categoryIDs, partIDs, depthList)


def runSonarScanWithCapture(context, properties, targets, materialMappings, categoryIDs, partIDs, depthList):
    """Run sonar scan and capture the hits directly."""
    from mathutils import Vector, Quaternion
    from math import radians
    import math

    sensor = properties.scannerObject
    depsgraph = context.evaluated_depsgraph_get()

    xRange = np.array([-90, 90])
    ySteps = (properties.fovSonar / 2.0) / properties.sonarStepDegree + 1
    yRange = np.linspace(-89.999, -90 + (properties.fovSonar / 2.0), int(ySteps))

    if properties.enableAnimation:
        firstFrame = properties.frameStart
        lastFrame = properties.frameEnd
        frameStep = properties.frameStep
    else:
        firstFrame = bpy.context.scene.frame_current
        lastFrame = bpy.context.scene.frame_current
        frameStep = 1

    totalNumberOfRays = xRange.size * yRange.size * int((lastFrame - firstFrame + 1) / frameStep)
    scannedValues = np.full(totalNumberOfRays, None, dtype=object)

    trees = {}
    valueIndex = 0

    startLocation = sensor.matrix_world.translation.copy()

    bpy.context.scene.frame_set(firstFrame)

    # Progress tracking
    frameList = list(range(firstFrame, lastFrame + 1, frameStep))
    totalFrames = len(frameList)

    for frameIdx, frameNumber in enumerate(frameList):
        if properties.outputProgress and totalFrames > 1:
            updateProgress("Sonar scan", (frameIdx + 1) / totalFrames)

        bpy.context.scene.frame_set(frameNumber)
        trees = getBVHTrees(trees, targets, depsgraph)

        origin = sensor.matrix_world.translation
        sensorHeight = origin.z
        traveledDistance = math.sqrt((startLocation.x - origin.x)**2 + (startLocation.y - origin.y)**2 + (startLocation.z - origin.z)**2)

        for x in xRange:
            quatX = Quaternion((0.0, 1.0, 0.0), radians(x))

            for y in yRange:
                quatY = Quaternion((1.0, 0.0, 0.0), radians(y))
                vec = Vector((0.0, 0.0, -1.0))
                quatAll = quatX @ quatY
                vec.rotate(quatAll)
                vec.rotate(sensor.matrix_world.decompose()[1])

                destination = vec + sensor.matrix_world.translation
                direction = (destination - origin).normalized()

                closestHit = sonar.castRay(
                    targets, trees, origin, direction, properties.maxDistance,
                    materialMappings, depsgraph, properties.debugLines, properties.debugOutput,
                    properties.sourceLevel, properties.noiseLevel, properties.directivityIndex,
                    properties.processingGain, properties.receptionThreshold
                )

                if closestHit is not None:
                    if "partID" in closestHit.target:
                        partIDIndex = closestHit.target["partID"]
                    else:
                        partIDIndex = closestHit.target.material_slots[materialMappings[closestHit.target][closestHit.faceIndex]].name

                    closestHit.categoryID = categoryIDs[closestHit.target["categoryID"]]
                    closestHit.partID = partIDs[partIDIndex]

                    # Apply noise if enabled
                    noise = properties.noiseAbsoluteOffset + (closestHit.distance * properties.noiseRelativeOffset / 100.0)
                    noiseDistance = closestHit.distance + noise
                    noiseDirection = direction.normalized() * noiseDistance
                    noiseLocation = noiseDirection + origin

                    closestHit.noiseLocation = noiseLocation
                    closestHit.noiseDistance = noiseDistance

                    if not properties.sonarMode3D:
                        if properties.sonarKeepRotation:
                            closestHit.location = Vector((direction.x, direction.y, 0)).normalized() * closestHit.distance + origin
                        else:
                            if x > 0:
                                closestHit.location.x = -closestHit.distance
                            else:
                                closestHit.location.x = closestHit.distance
                            closestHit.location.y = traveledDistance
                            closestHit.location.z = startLocation.z

                    closestHit.sensor_id = "sonar"
                    scannedValues[valueIndex] = closestHit
                    valueIndex += 1

    return [hit for hit in scannedValues[:valueIndex] if hit is not None]


def getBVHTrees(trees, targets, depsgraph):
    for target in targets:
        # check if the target is already in the tree map
        if target in trees:
            # if so, get the old values
            (existingTree, matrix_world) = trees[target]

            # if the object did not change its world matrix, we
            # don't have to recompute the tree
            if matrix_world == target.matrix_world:
                continue

        # the easy way would be to use this function, but then we would have to transform all
        # coordinates into the object's local coordinate system
        #trees[target] = BVHTree.FromObject(target, depsgraph)
        
        # source: https://developer.blender.org/T57861
        bm = bmesh.new()
        bm.from_object(target, depsgraph=depsgraph)
        bm.transform(target.matrix_world)
        
        trees[target] = (BVHTree.FromBMesh(bm), target.matrix_world.copy())

        bm.free()  # always do this when finished

    return trees
# -*- coding: utf-8 -*-
# ***************************************************************************
# *   Copyright (c) 2018 Kresimir Tusek <kresimir.tusek@gmail.com>          *
# *   Copyright (c) 2019-2021 Schildkroet                                   *
# *                                                                         *
# *   This file is part of the FreeCAD CAx development system.              *
# *                                                                         *
# *   This library is free software; you can redistribute it and/or         *
# *   modify it under the terms of the GNU Library General Public           *
# *   License as published by the Free Software Foundation; either          *
# *   version 2 of the License, or (at your option) any later version.      *
# *                                                                         *
# *   This library  is distributed in the hope that it will be useful,      *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU Library General Public License for more details.                  *
# *                                                                         *
# *   You should have received a copy of the GNU Library General Public     *
# *   License along with this library; see the file COPYING.LIB. If not,    *
# *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
# *   Suite 330, Boston, MA  02111-1307, USA                                *
# *                                                                         *
# ***************************************************************************

import BOPTools
import Path
import Path.Op.Base as PathOp
import PathScripts.PathUtils as PathUtils
import FreeCAD
import time
import json
import math
import area
from functools import cmp_to_key
from PySide.QtCore import QT_TRANSLATE_NOOP

if FreeCAD.GuiUp:
    from pivy import coin
    import FreeCADGui

__doc__ = "Class and implementation of the Adaptive CAM operation."

# lazily loaded modules
from lazy_loader.lazy_loader import LazyLoader

Part = LazyLoader("Part", globals(), "Part")
TechDraw = LazyLoader("TechDraw", globals(), "TechDraw")
FeatureExtensions = LazyLoader("Path.Op.FeatureExtension", globals(), "Path.Op.FeatureExtension")
DraftGeomUtils = LazyLoader("DraftGeomUtils", globals(), "DraftGeomUtils")


if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


translate = FreeCAD.Qt.translate


def convertTo2d(pathArray):
    output = []
    for path in pathArray:
        pth2 = []
        for edge in path:
            for pt in edge:
                pth2.append([pt[0], pt[1]])
        output.append(pth2)
    return output


sceneGraph = None
scenePathNodes = []  # for scene cleanup afterwards
topZ = 10


def sceneDrawPath(path, color=(0, 0, 1)):
    coPoint = coin.SoCoordinate3()

    pts = []
    for pt in path:
        pts.append([pt[0], pt[1], topZ])

    coPoint.point.setValues(0, len(pts), pts)
    ma = coin.SoBaseColor()
    ma.rgb = color
    li = coin.SoLineSet()
    li.numVertices.setValue(len(pts))
    pathNode = coin.SoSeparator()
    pathNode.addChild(coPoint)
    pathNode.addChild(ma)
    pathNode.addChild(li)
    sceneGraph.addChild(pathNode)
    scenePathNodes.append(pathNode)  # for scene cleanup afterwards


def sceneClean():
    for n in scenePathNodes:
        sceneGraph.removeChild(n)

    del scenePathNodes[:]


def discretize(edge, flipDirection=False):
    pts = edge.discretize(Deflection=0.002)
    if flipDirection:
        pts.reverse()

    return pts


def CalcHelixConePoint(height, cur_z, radius, angle):
    x = ((height - cur_z) / height) * radius * math.cos(math.radians(angle) * cur_z)
    y = ((height - cur_z) / height) * radius * math.sin(math.radians(angle) * cur_z)
    z = cur_z

    return {"X": x, "Y": y, "Z": z}


def GenerateGCode(op, obj, adaptiveResults, helixDiameter):
    if not adaptiveResults or not adaptiveResults[0]["AdaptivePaths"]:
        return

    # minLiftDistance = op.tool.Diameter
    helixRadius = 0
    for region in adaptiveResults:
        p1 = region["HelixCenterPoint"]
        p2 = region["StartPoint"]
        r = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        if r > helixRadius:
            helixRadius = r

    stepDown = max(obj.StepDown.Value, 0.1)

    length = 2 * math.pi * helixRadius

    obj.HelixAngle = min(89, max(float(obj.HelixAngle), 1))
    obj.HelixConeAngle = max(float(obj.HelixConeAngle), 0)

    helixAngleRad = math.pi * float(obj.HelixAngle) / 180.0
    depthPerOneCircle = length * math.tan(helixAngleRad)
    # print("Helix circle depth: {}".format(depthPerOneCircle))

    stepUp = max(obj.LiftDistance.Value, 0)

    finish_step = obj.FinishDepth.Value if hasattr(obj, "FinishDepth") else 0.0
    if finish_step > stepDown:
        finish_step = stepDown

    # Seems like lx/ly/lz are the last x/y/z position prior to a move, used to
    # see if our next move changes one of them/if we need to add (eg) a Z move?
    # ml: this is dangerous because it'll hide all unused variables hence forward
    #     however, I don't know what lx and ly signify so I'll leave them for now
    # lx = adaptiveResults[0]["HelixCenterPoint"][0]
    # ly = adaptiveResults[0]["HelixCenterPoint"][1]
    lz = obj.StartDepth.Value

    # Ensure we're cutting top-down- note reverse sort y-x!
    # FIXME: Rethink this
    adaptiveResults = sorted(
        adaptiveResults, key=cmp_to_key(lambda x, y: y["TopDepth"] - x["TopDepth"])
    )

    for region in adaptiveResults:
        passStartDepth = region["TopDepth"]

        print(
            "Processing region with final depth %.3f, starting at %.3f"
            % (region["BottomDepth"], region["TopDepth"])
        )
        depth_params = PathUtils.depth_params(
            clearance_height=obj.ClearanceHeight.Value,
            safe_height=obj.SafeHeight.Value,
            start_depth=region["TopDepth"],
            step_down=stepDown,
            z_finish_step=finish_step,
            final_depth=region["BottomDepth"],
            user_depths=None,
        )

        for passEndDepth in depth_params.data:
            print("\tPass at %.3f" % (passEndDepth,))
            startAngle = math.atan2(
                region["StartPoint"][1] - region["HelixCenterPoint"][1],
                region["StartPoint"][0] - region["HelixCenterPoint"][0],
            )

            # lx = region["HelixCenterPoint"][0]
            # ly = region["HelixCenterPoint"][1]

            passDepth = passStartDepth - passEndDepth

            p1 = region["HelixCenterPoint"]
            p2 = region["StartPoint"]
            helixRadius = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            # print("\tHelix radius: %.3f" % (helixRadius,))
            # print("\tStart point: %s\tHelix center: %s" % (region["StartPoint"],region["HelixCenterPoint"]))

            # Helix ramp
            if helixRadius > 0.01:
                r = helixRadius - 0.01

                maxfi = passDepth / depthPerOneCircle * 2 * math.pi
                fi = 0
                offsetFi = -maxfi + startAngle - math.pi / 16

                helixStart = [
                    region["HelixCenterPoint"][0] + r * math.cos(offsetFi),
                    region["HelixCenterPoint"][1] + r * math.sin(offsetFi),
                ]

                op.commandlist.append(Path.Command("(Helix to depth: %f)" % passEndDepth))

                if obj.UseHelixArcs is False:
                    # rapid move to start point
                    op.commandlist.append(Path.Command("G0", {"Z": obj.ClearanceHeight.Value}))
                    op.commandlist.append(
                        Path.Command(
                            "G0",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": obj.ClearanceHeight.Value,
                            },
                        )
                    )

                    # rapid move to safe height
                    op.commandlist.append(
                        Path.Command(
                            "G0",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": obj.SafeHeight.Value,
                            },
                        )
                    )

                    # move to start depth
                    op.commandlist.append(
                        Path.Command(
                            "G1",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": passStartDepth,
                                "F": op.vertFeed,
                            },
                        )
                    )

                    if obj.HelixConeAngle == 0:
                        while fi < maxfi:
                            x = region["HelixCenterPoint"][0] + r * math.cos(fi + offsetFi)
                            y = region["HelixCenterPoint"][1] + r * math.sin(fi + offsetFi)
                            z = passStartDepth - fi / maxfi * (passStartDepth - passEndDepth)
                            op.commandlist.append(
                                Path.Command("G1", {"X": x, "Y": y, "Z": z, "F": op.vertFeed})
                            )
                            # lx = x
                            # ly = y
                            fi = fi + math.pi / 16

                        # one more circle at target depth to make sure center is cleared
                        maxfi = maxfi + 2 * math.pi
                        while fi < maxfi:
                            x = region["HelixCenterPoint"][0] + r * math.cos(fi + offsetFi)
                            y = region["HelixCenterPoint"][1] + r * math.sin(fi + offsetFi)
                            z = passEndDepth
                            op.commandlist.append(
                                Path.Command("G1", {"X": x, "Y": y, "Z": z, "F": op.horizFeed})
                            )
                            # lx = x
                            # ly = y
                            fi = fi + math.pi / 16

                    else:
                        # Cone
                        _HelixAngle = 360 - (float(obj.HelixAngle) * 4)

                        if obj.HelixConeAngle > 6:
                            obj.HelixConeAngle = 6

                        helixRadius *= 0.9

                        # Calculate everything
                        helix_height = passStartDepth - passEndDepth
                        r_extra = helix_height * math.tan(math.radians(obj.HelixConeAngle))
                        HelixTopRadius = helixRadius + r_extra
                        helix_full_height = HelixTopRadius * (
                            math.cos(math.radians(obj.HelixConeAngle))
                            / math.sin(math.radians(obj.HelixConeAngle))
                        )

                        # Start height
                        z = passStartDepth
                        i = 0

                        # Default step down
                        z_step = 0.05

                        # Bigger angle, smaller step down
                        if _HelixAngle > 120:
                            z_step = 0.025
                        if _HelixAngle > 240:
                            z_step = 0.015

                        p = None
                        # Calculate conical helix
                        while z >= passEndDepth:
                            if z < passEndDepth:
                                z = passEndDepth

                            p = CalcHelixConePoint(
                                helix_full_height, i, HelixTopRadius, _HelixAngle
                            )
                            op.commandlist.append(
                                Path.Command(
                                    "G1",
                                    {
                                        "X": p["X"] + region["HelixCenterPoint"][0],
                                        "Y": p["Y"] + region["HelixCenterPoint"][1],
                                        "Z": z,
                                        "F": op.vertFeed,
                                    },
                                )
                            )
                            z = z - z_step
                            i = i + z_step

                        # Calculate some stuff for arcs at bottom
                        p["X"] = p["X"] + region["HelixCenterPoint"][0]
                        p["Y"] = p["Y"] + region["HelixCenterPoint"][1]
                        x_m = region["HelixCenterPoint"][0] - p["X"] + region["HelixCenterPoint"][0]
                        y_m = region["HelixCenterPoint"][1] - p["Y"] + region["HelixCenterPoint"][1]
                        i_off = (x_m - p["X"]) / 2
                        j_off = (y_m - p["Y"]) / 2

                        # One more circle at target depth to make sure center is cleared
                        op.commandlist.append(
                            Path.Command(
                                "G3",
                                {
                                    "X": x_m,
                                    "Y": y_m,
                                    "Z": passEndDepth,
                                    "I": i_off,
                                    "J": j_off,
                                    "F": op.horizFeed,
                                },
                            )
                        )
                        op.commandlist.append(
                            Path.Command(
                                "G3",
                                {
                                    "X": p["X"],
                                    "Y": p["Y"],
                                    "Z": passEndDepth,
                                    "I": -i_off,
                                    "J": -j_off,
                                    "F": op.horizFeed,
                                },
                            )
                        )

                else:
                    # Use arcs for helix - no conical shape support
                    helixStart = [
                        region["HelixCenterPoint"][0] + r,
                        region["HelixCenterPoint"][1],
                    ]

                    # rapid move to start point
                    op.commandlist.append(Path.Command("G0", {"Z": obj.ClearanceHeight.Value}))
                    op.commandlist.append(
                        Path.Command(
                            "G0",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": obj.ClearanceHeight.Value,
                            },
                        )
                    )

                    # rapid move to safe height
                    op.commandlist.append(
                        Path.Command(
                            "G0",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": obj.SafeHeight.Value,
                            },
                        )
                    )

                    # move to start depth
                    op.commandlist.append(
                        Path.Command(
                            "G1",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": passStartDepth,
                                "F": op.vertFeed,
                            },
                        )
                    )

                    x = region["HelixCenterPoint"][0] + r
                    y = region["HelixCenterPoint"][1]

                    curDep = passStartDepth
                    while curDep > (passEndDepth + depthPerOneCircle):
                        op.commandlist.append(
                            Path.Command(
                                "G2",
                                {
                                    "X": x - (2 * r),
                                    "Y": y,
                                    "Z": curDep - (depthPerOneCircle / 2),
                                    "I": -r,
                                    "F": op.vertFeed,
                                },
                            )
                        )
                        op.commandlist.append(
                            Path.Command(
                                "G2",
                                {
                                    "X": x,
                                    "Y": y,
                                    "Z": curDep - depthPerOneCircle,
                                    "I": r,
                                    "F": op.vertFeed,
                                },
                            )
                        )
                        curDep = curDep - depthPerOneCircle

                    lastStep = curDep - passEndDepth
                    if lastStep > (depthPerOneCircle / 2):
                        op.commandlist.append(
                            Path.Command(
                                "G2",
                                {
                                    "X": x - (2 * r),
                                    "Y": y,
                                    "Z": curDep - (lastStep / 2),
                                    "I": -r,
                                    "F": op.vertFeed,
                                },
                            )
                        )
                        op.commandlist.append(
                            Path.Command(
                                "G2",
                                {
                                    "X": x,
                                    "Y": y,
                                    "Z": passEndDepth,
                                    "I": r,
                                    "F": op.vertFeed,
                                },
                            )
                        )
                    else:
                        op.commandlist.append(
                            Path.Command(
                                "G2",
                                {
                                    "X": x - (2 * r),
                                    "Y": y,
                                    "Z": passEndDepth,
                                    "I": -r,
                                    "F": op.vertFeed,
                                },
                            )
                        )
                        op.commandlist.append(
                            Path.Command(
                                "G1",
                                {"X": x, "Y": y, "Z": passEndDepth, "F": op.vertFeed},
                            )
                        )

                    # one more circle at target depth to make sure center is cleared
                    op.commandlist.append(
                        Path.Command(
                            "G2",
                            {
                                "X": x - (2 * r),
                                "Y": y,
                                "Z": passEndDepth,
                                "I": -r,
                                "F": op.horizFeed,
                            },
                        )
                    )
                    op.commandlist.append(
                        Path.Command(
                            "G2",
                            {
                                "X": x,
                                "Y": y,
                                "Z": passEndDepth,
                                "I": r,
                                "F": op.horizFeed,
                            },
                        )
                    )
                    # lx = x
                    # ly = y

            else:  # no helix entry
                # rapid move to clearance height
                op.commandlist.append(Path.Command("G0", {"Z": obj.ClearanceHeight.Value}))
                op.commandlist.append(
                    Path.Command(
                        "G0",
                        {
                            "X": region["StartPoint"][0],
                            "Y": region["StartPoint"][1],
                            "Z": obj.ClearanceHeight.Value,
                        },
                    )
                )
                # straight plunge to target depth
                op.commandlist.append(
                    Path.Command(
                        "G1",
                        {
                            "X": region["StartPoint"][0],
                            "Y": region["StartPoint"][1],
                            "Z": passEndDepth,
                            "F": op.vertFeed,
                        },
                    )
                )

            lz = passEndDepth
            z = obj.ClearanceHeight.Value
            op.commandlist.append(Path.Command("(Adaptive - depth: %f)" % passEndDepth))

            # add adaptive paths
            for pth in region["AdaptivePaths"]:
                motionType = pth[0]  # [0] contains motion type

                for pt in pth[1]:  # [1] contains list of points
                    x = pt[0]
                    y = pt[1]

                    # dist = math.sqrt((x-lx)*(x-lx) + (y-ly)*(y-ly))

                    if motionType == area.AdaptiveMotionType.Cutting:
                        z = passEndDepth
                        if z != lz:
                            op.commandlist.append(Path.Command("G1", {"Z": z, "F": op.vertFeed}))

                        op.commandlist.append(
                            Path.Command("G1", {"X": x, "Y": y, "F": op.horizFeed})
                        )

                    elif motionType == area.AdaptiveMotionType.LinkClear:
                        z = passEndDepth + stepUp
                        if z != lz:
                            op.commandlist.append(Path.Command("G0", {"Z": z}))

                        op.commandlist.append(Path.Command("G0", {"X": x, "Y": y}))

                    elif motionType == area.AdaptiveMotionType.LinkNotClear:
                        z = obj.ClearanceHeight.Value
                        if z != lz:
                            op.commandlist.append(Path.Command("G0", {"Z": z}))

                        op.commandlist.append(Path.Command("G0", {"X": x, "Y": y}))

                    # elif motionType == area.AdaptiveMotionType.LinkClearAtPrevPass:
                    #     if lx!=x or ly!=y:
                    #         op.commandlist.append(Path.Command("G0", { "X": lx, "Y":ly, "Z":passStartDepth+stepUp}))
                    #     op.commandlist.append(Path.Command("G0", { "X": x, "Y":y, "Z":passStartDepth+stepUp}))

                    # lx = x
                    # ly = y
                    lz = z

            # return to safe height in this Z pass
            z = obj.ClearanceHeight.Value
            if z != lz:
                op.commandlist.append(Path.Command("G0", {"Z": z}))

            lz = z

            passStartDepth = passEndDepth

            # return to safe height in this Z pass
            z = obj.ClearanceHeight.Value
            if z != lz:
                op.commandlist.append(Path.Command("G0", {"Z": z}))

            lz = z

    z = obj.ClearanceHeight.Value
    if z != lz:
        op.commandlist.append(Path.Command("G0", {"Z": z}))


# functions for user-area-limited and automagic behavior. Maybe a parameter.
def Execute(op, obj):
    global sceneGraph
    global topZ

    if FreeCAD.GuiUp:
        sceneGraph = FreeCADGui.ActiveDocument.ActiveView.getSceneGraph()

    Path.Log.info("*** Adaptive toolpath processing started...\n")

    # hide old toolpaths during recalculation
    obj.Path = Path.Path("(Calculating...)")

    if FreeCAD.GuiUp:
        # store old visibility state
        job = op.getJob(obj)
        oldObjVisibility = obj.ViewObject.Visibility
        oldJobVisibility = job.ViewObject.Visibility

        obj.ViewObject.Visibility = False
        job.ViewObject.Visibility = False

        FreeCADGui.updateGui()

    try:
        helixDiameter = obj.HelixDiameterLimit.Value
        topZ = op.stock.Shape.BoundBox.ZMax
        obj.Stopped = False
        obj.StopProcessing = False
        if obj.Tolerance < 0.001:
            obj.Tolerance = 0.001

        # No need to calculate stock multiple times
        # FIXME: Slice the stock at every depth too!
        outer_wire = TechDraw.findShapeOutline(op.stock.Shape, 1, FreeCAD.Vector(0, 0, 1))
        stockPaths = [[discretize(outer_wire)]]

        stockPath2d = convertTo2d(stockPaths)

        outsidePathArray2dDepthTuples = [
            (dtop, dbot, convertTo2d(pathArray)) for dtop, dbot, pathArray in op.outsidePathArray
        ]

        insidePathArray2dDepthTuples = [
            (dtop, dbot, convertTo2d(pathArray)) for dtop, dbot, pathArray in op.insidePathArray
        ]

        outsideOpType = area.AdaptiveOperationType.ClearingOutside
        insideOpType = area.AdaptiveOperationType.ClearingInside

        keepToolDownRatio = 3.0
        if hasattr(obj, "KeepToolDownRatio"):
            keepToolDownRatio = float(obj.KeepToolDownRatio)

        # FIXME: This is... kinda nonsense if we're doing adaptive roughing?
        # FIXME: Respect user settings if NOT doing adaptive roughing!
        outsideInputStateObject = {
            "tool": float(op.tool.Diameter),
            "tolerance": float(obj.Tolerance),
            "geometry": outsidePathArray2dDepthTuples,
            "stockGeometry": stockPath2d,
            "stepover": float(obj.StepOver),
            "effectiveHelixDiameter": float(helixDiameter),
            "operationType": "Clearing",
            "side": "Outside",
            "forceInsideOut": obj.ForceInsideOut,
            "finishingProfile": obj.FinishingProfile,
            "keepToolDownRatio": keepToolDownRatio,
            "stockToLeave": float(obj.StockToLeave),
        }

        insideInputStateObject = {
            "tool": float(op.tool.Diameter),
            "tolerance": float(obj.Tolerance),
            "geometry": insidePathArray2dDepthTuples,
            "stockGeometry": stockPath2d,
            "stepover": float(obj.StepOver),
            "effectiveHelixDiameter": float(helixDiameter),
            "operationType": "Clearing",
            "side": "Inside",
            "forceInsideOut": obj.ForceInsideOut,
            "finishingProfile": obj.FinishingProfile,
            "keepToolDownRatio": keepToolDownRatio,
            "stockToLeave": float(obj.StockToLeave),
        }

        inputStateObject = [outsideInputStateObject, insideInputStateObject]

        inputStateChanged = False
        adaptiveResults = None

        # If we have a valid... path? Something. Generated, make that
        # tentatively the output
        if obj.AdaptiveOutputState:
            adaptiveResults = obj.AdaptiveOutputState

        # If ANYTHING in our input-cutting parameters, cutting regions,
        # etc.- changes, force recalculating
        if json.dumps(obj.AdaptiveInputState) != json.dumps(inputStateObject):
            inputStateChanged = True
            adaptiveResults = None

        # progress callback fn, if return true it will stop processing
        def progressFn(tpaths):
            if FreeCAD.GuiUp:
                for (
                    path
                ) in tpaths:  # path[0] contains the MotionType, #path[1] contains list of points
                    if path[0] == area.AdaptiveMotionType.Cutting:
                        sceneDrawPath(path[1], (0, 0, 1))

                    else:
                        sceneDrawPath(path[1], (1, 0, 1))

                FreeCADGui.updateGui()

            return obj.StopProcessing

        start = time.time()

        if inputStateChanged or adaptiveResults is None:
            # NOTE: Seem to need to create a new a2d for each area when we're
            # stepping down depths like this. If we don't, it will keep history
            # from the last region we did.
            results = list()

            # Outside paths
            for dtop, dbot, path2d in outsidePathArray2dDepthTuples:
                a2d = area.Adaptive2d()
                a2d.stepOverFactor = 0.01 * obj.StepOver
                a2d.toolDiameter = float(op.tool.Diameter)
                a2d.helixRampDiameter = helixDiameter
                a2d.keepToolDownDistRatio = keepToolDownRatio
                a2d.stockToLeave = float(obj.StockToLeave)
                a2d.tolerance = float(obj.Tolerance)
                a2d.forceInsideOut = obj.ForceInsideOut
                a2d.finishingProfile = obj.FinishingProfile
                a2d.opType = outsideOpType

                results.append((dtop, dbot, a2d.Execute(stockPath2d, path2d, progressFn)))

            # Inside paths
            for dtop, dbot, path2d in insidePathArray2dDepthTuples:
                a2d = area.Adaptive2d()
                a2d.stepOverFactor = 0.01 * obj.StepOver
                a2d.toolDiameter = float(op.tool.Diameter)
                a2d.helixRampDiameter = helixDiameter
                a2d.keepToolDownDistRatio = keepToolDownRatio
                a2d.stockToLeave = float(obj.StockToLeave)
                a2d.tolerance = float(obj.Tolerance)
                a2d.forceInsideOut = obj.ForceInsideOut
                a2d.finishingProfile = obj.FinishingProfile
                a2d.opType = insideOpType

                results.append((dtop, dbot, a2d.Execute(stockPath2d, path2d, progressFn)))

            # need to convert results to python object to be JSON serializable
            adaptiveResults = []
            for dtop, dbot, areas in results:
                for result in areas:
                    adaptiveResults.append(
                        {
                            "HelixCenterPoint": result.HelixCenterPoint,
                            "StartPoint": result.StartPoint,
                            "AdaptivePaths": result.AdaptivePaths,
                            "ReturnMotionType": result.ReturnMotionType,
                            "TopDepth": dtop,
                            "BottomDepth": dbot,
                        }
                    )

        # GENERATE
        GenerateGCode(op, obj, adaptiveResults, helixDiameter)

        if not obj.StopProcessing:
            Path.Log.info("*** Done. Elapsed time: %f sec\n\n" % (time.time() - start))
            obj.AdaptiveOutputState = adaptiveResults
            obj.AdaptiveInputState = inputStateObject

        else:
            Path.Log.info("*** Processing cancelled (after: %f sec).\n\n" % (time.time() - start))

    finally:
        if FreeCAD.GuiUp:
            obj.ViewObject.Visibility = oldObjVisibility
            job.ViewObject.Visibility = oldJobVisibility
            sceneClean()


def _get_solid_projection(shp, z, projFace):
    """_get_solid_projection(shp, z, proj_bb)
    Calculates a shape obtained by slicing shp at the height z, then projecting
    the solids above that height onto a region of proj_face, and creating a
    simplified face
    """
    bb = shp.BoundBox
    projdir = FreeCAD.Vector(0, 0, 1)

    above_faces = []

    # Find all faces above the machining depth. This is used to mask future
    # interior cuts, and the outer wire is used as the external wire
    bb_cut_top = Part.makeBox(
        bb.XLength,
        bb.YLength,
        max(bb.ZLength, bb.ZLength - z),
        FreeCAD.Vector(bb.XMin, bb.YMin, z),
    )
    above_solids = shp.common(bb_cut_top).Solids
    for s in above_solids:
        for f in s.Faces:
            # If you don't remove vertical faces, the vertical face of (eg) a
            # cylindrical hole will create a new face that cancels out the hole
            # left in the top/bottom surfaces, masking off regions that
            # should be accessible.
            if Path.Geom.isVertical(f):
                continue
            above_faces += [
                Part.makeFace(
                    [projFace.makeParallelProjection(w, projdir).Wires[0] for w in f.Wires]
                )
            ]
    # for s in above_solids

    # fuse and refine into one face, then remove extra edges
    if above_faces:
        above_refined = above_faces[0].fuse(above_faces[1:]).removeSplitter()
    else:
        # Make a dummy shape if we don't have anything actually above
        above_refined = Part.Shape()

    return above_refined


def _working_edge_helper_roughing(op, obj, depths):
    # Final calculated regions
    inside_regions = list()
    outside_regions = list()

    stock_bb = op.stock.Shape.BoundBox

    # FIXME: How to handle multiple shapes? Wrap entire thing in a "for m in op.model"?
    shp = op.model[0].Shape

    # Create bounding box, move it to the depth we're cutting, and keep the
    # portion above that to apply as a mask to the area specified for machining-
    # can't machine something with material above it.
    bb = shp.BoundBox

    # Make a face to project onto
    # NOTE: Use 0 as the height, since that's what TechDraw.findShapeOutline
    # uses, which we use to find the machining boundary, and the actual depth
    # is tracked separately.
    projface = Path.Geom.makeBoundBoxFace(stock_bb, zHeight=0)
    projdir = FreeCAD.Vector(0, 0, 1)

    lastdepth = obj.StartDepth.Value

    for depth in depths:
        above_refined = _get_solid_projection(shp, depth, projface)

        # Create appropriate tuples and add to list_working_edge_helper_roughing
        # Outside is based on the outer wire of the above_faces
        # Insides are based on the remaining "below" regions, masked by the
        # "above"- if something is above an area, we can't machine it in 2.5D

        # Outside: Take the outer wire of the above faces
        # NOTE: Exactly one entry per depth (not necessarily one depth entry per
        # stepdown, however), which is a LIST of the wires we're staying outside
        # NOTE: Do this FIRST- if any inside regions share enough of an edge
        # with an outside region for a tool to get through, we want to skip them
        # for the current stepdown
        regions = [TechDraw.findShapeOutline(k, 1, projdir) for k in above_refined.Faces]
        # If this region exists in our list, it has to be the last entry, due to
        # proceeding in order and having only one per depth. If it's already
        # there, replace with the new, deeper depth, else add new
        match = outside_regions and len(outside_regions[-1][2]) == len(regions)
        if match:
            prev = outside_regions[-1][2]
            for r1, r2 in zip(regions, prev):
                # FIXME: Smarter way to do this than a full cut operation?
                if r1.cut(r2).Wires:
                    match = False
                    break

        if match:
            outside_regions[-1] = (outside_regions[-1][0], depth, regions)
        else:
            outside_regions.append((lastdepth, depth, regions))

        # Inside:
        # Take outline of entire model as our baseline desired machining region
        # FIXME: Can move the first part of this outside the loop!
        below_faces = list()
        # Places to machine are anywhere in the selected boundary
        below_wires = [TechDraw.findShapeOutline(shp, 1, projdir)]

        # Also want to remove any overlapping areas machined from the
        # outside. ONLY do this if we didn't explicitly select a region
        outsideface = projface.cut([Part.Face(x) for x in outside_regions[-1][2]])
        # below_faces = [f.cut(outsideface) for f in below_faces]
        below_faces = [Part.Face(w).cut(outsideface) for w in below_wires]

        if below_faces:
            below_fusion = below_faces[0].fuse(below_faces[1:])
            below_refined = below_fusion.removeSplitter()
            # Remove the overhangs from the desired region to cut
            final_cut = below_refined.cut(above_refined)
        else:
            # Make a dummy shape if we don't have anything actually below
            final_cut = Part.Shape()

        # Split up into individual faces if any are disjoint, then update
        # inside_regions- either by adding a new entry OR by updating the depth
        # of an existing entry
        for f in final_cut.Faces:
            # Brut-force search all existing regions to see if any are the same
            newtop = lastdepth
            for dtop, dbot, df in inside_regions:
                # FIXME: Smarter way to do this than a full cut operation?
                if not df.cut(f).Wires:
                    print("DELETING FACE AT DEPTH %.3f" % (depth,))
                    newtop = dtop
                    inside_regions.remove((dtop, dbot, df))
                    break

            # Can add new region now- may or may not have removed an identical
            # one previously, don't care
            inside_regions.append((newtop, depth, f))

        # Update the last depth step
        lastdepth = depth
    # end for depth

    return inside_regions, outside_regions


def _working_edge_helper_manual(op, obj, depths):
    # Final calculated regions
    inside_regions = list()
    outside_regions = list()

    # User selections, with extensions
    selected_regions = list()
    selected_edges = list()

    # Get extensions and identify faces to avoid
    extensions = FeatureExtensions.getExtensions(obj)
    avoidFeatures = [e for e in extensions if e.avoid]

    # Similarly, expand selected regions with extensions
    for ext in extensions:
        if not ext.avoid:
            if wire := ext.getWire():
                selected_regions += [f for f in ext.getExtensionFaces(wire)]

    # Easy boolean to decide if we're machining the inside or outside region
    isOutside = obj.Side == "Outside"

    for base, subs in obj.Base:
        for sub in subs:
            element = base.Shape.getElement(sub)
            if sub.startswith("Face") and sub not in avoidFeatures:
                shape = Part.Face(element.OuterWire) if obj.UseOutline else element
                selected_regions.append(shape)
            elif sub.startswith("Edge"):
                selected_edges.append(element)
    # Efor

    stock_bb = op.stock.Shape.BoundBox

    # FIXME: How to handle multiple shapes? Wrap entire thing in a "for m in op.model"?
    shp = op.model[0].Shape

    # Create bounding box, move it to the depth we're cutting, and keep the
    # portion above that to apply as a mask to the area specified for machining-
    # can't machine something with material above it.
    bb = shp.BoundBox

    # Make a face to project onto
    # NOTE: Use 0 as the height, since that's what TechDraw.findShapeOutline
    # uses, which we use to find the machining boundary, and the actual depth
    # is tracked separately.
    projface = Path.Geom.makeBoundBoxFace(stock_bb, zHeight=0)
    projdir = FreeCAD.Vector(0, 0, 1)

    # Merge all of the selected regions into a single shape
    selected_regions_merged = list()
    for f in selected_regions:
        selected_regions_merged += [
            Part.makeFace([projface.makeParallelProjection(w, projdir).Wires[0] for w in f.Wires])
        ]
    # Places to machine are anywhere in the selected boundary
    if selected_edges:
        edgeWires = DraftGeomUtils.findWires(selected_edges)
        for ew in edgeWires:
            selected_regions_merged += [
                Part.makeFace(projface.makeParallelProjection(ew, projdir).Wires[0])
            ]

    # FIXME: Think we still need this 'if' in case of no closed wires?
    if selected_regions_merged:
        selected_fusion = selected_regions_merged[0].fuse(selected_regions_merged[1:])
        selected_refined = selected_fusion.removeSplitter()
    else:
        # Make a dummy shape if we don't actually have anything
        selected_refined = Part.Shape()

    lastdepth = obj.StartDepth.Value

    for depth in depths:
        above_refined = _get_solid_projection(shp, depth, projface)

        # Create appropriate tuples and add to list
        if isOutside:
            # Outside is based on the outer wire of the above_faces
            # Insides are based on the remaining "below" regions, masked by the
            # "above"- if something is above an area, we can't machine it in 2.5D

            # Outside: Take the outer wire of the above faces, added to selected
            # edges and regions
            # NOTE: Exactly one entry per depth (not necessarily one depth entry per
            # stepdown, however), which is a LIST of the wires we're staying outside
            # NOTE: Do this FIRST- if any inside regions share enough of an edge
            # with an outside region for a tool to get through, we want to skip them
            # for the current stepdown
            # regions = [TechDraw.findShapeOutline(k, 1, projdir) for k in above_refined.Faces]

            # FIXME: This gives weird artifacts
            # regions = [TechDraw.findShapeOutline(k, 1, projdir) for k in above_refined.Faces]
            # regions += [TechDraw.findShapeOutline(k, 1, projdir) for k in selected_refined.Faces]
            # ^^^

            # This actually seems to work, but...
            # FIXME: We don't really need this to be a list- simplify checks below!
            final_cut = selected_refined.fuse(above_refined).removeSplitter()
            regions = [TechDraw.findShapeOutline(final_cut, 1, projdir)]

            # THIS vvv
            # final_cut = selected_refined.fuse(above_refined).removeSplitter()
            # regions = [TechDraw.findShapeOutline(k, 1, projdir) for k in final_cut.Faces]
            # FIXME: This ^^^ doesn't quite work- ends up cutting some inside areas above the selected faces. Looks like removeSplitter isn't merging shared internal faces?

            # If this region exists in our list, it has to be the last entry, due to
            # proceeding in order and having only one per depth. If it's already
            # there, replace with the new, deeper depth, else add new
            match = outside_regions and len(outside_regions[-1][2]) == len(regions)
            if match:
                prev = outside_regions[-1][2]
                for r1, r2 in zip(regions, prev):
                    # FIXME: Smarter way to do this than a full cut operation?
                    if r1.cut(r2).Wires:
                        match = False
                        break

            if match:
                outside_regions[-1] = (outside_regions[-1][0], depth, regions)
            else:
                outside_regions.append((lastdepth, depth, regions))

        # Inside
        # For every area selected by the user, project to a plane
        else:
            final_cut = selected_refined.cut(above_refined)

            # Split up into individual faces if any are disjoint, then update
            # inside_regions- either by adding a new entry OR by updating the depth
            # of an existing entry
            for f in final_cut.Faces:
                # Brute-force search all existing regions to see if any are the same
                newtop = lastdepth
                for dtop, dbot, df in inside_regions:
                    # FIXME: Smarter way to do this than a full cut operation?
                    if not df.cut(f).Wires:
                        print("DELETING FACE AT DEPTH %.3f" % (depth,))
                        newtop = dtop
                        inside_regions.remove((dtop, dbot, df))
                        break

                # Can add new region now- may or may not have removed an identical
                # one previously, don't care
                inside_regions.append((newtop, depth, f))

        # Update the last depth step
        lastdepth = depth
    # end for depth

    return inside_regions, outside_regions


def _get_working_edges(op, obj):
    """_get_working_edges(op, obj)...
    Compile all working edges from the Base Geometry selection (obj.Base)
    for the current operation.
    Additional modifications to selected region(face), such as extensions,
    should be placed within this function.
    This version will return two lists- one for outside edges and one for inside
    edges. Each list will be FIXME TBD (topdepth, bottomdepth, edges) tuples.
    """

    stock_bb = op.stock.Shape.BoundBox

    # Find depth steps, throwing out all depths above anywhere we might cut
    depth_params = PathUtils.depth_params(
        clearance_height=obj.ClearanceHeight.Value,
        safe_height=obj.SafeHeight.Value,
        start_depth=obj.StartDepth.Value,
        step_down=max(obj.StepDown.Value, 0.1),
        z_finish_step=0,
        final_depth=obj.FinalDepth.Value,
        user_depths=None,
    )

    depths = [d for d in depth_params.data if d <= stock_bb.ZMax]

    # If user specified edges, calculate the machining regions based on that
    # input. Otherwise, process entire model
    # List of (topdepth, bottomdepth, region) tuples
    # Inside regions may have multiple entries per depth; outside regions are
    # one per depth. In both cases, each unique region shall have exactly one
    # entry, at the lowest depth of that region.
    if obj.Base:
        inside_regions, outside_regions = _working_edge_helper_manual(op, obj, depths)
    else:
        inside_regions, outside_regions = _working_edge_helper_roughing(op, obj, depths)

    # FIXME: Somewhere in here would be the right place to reorder for depth-
    # first or region-first machining. Right now we always have a weird hybrid
    # where it's depth-first, until a region with multiple stepdowns is found,
    # which is then processed depth-first. This is complicated by our current
    # return type. Might need to change to a single list returned, where the
    # list is in proper cut order, which would then require each element to also
    # be tagged with whether it's an inside or outside cut
    # FIXME: If you do this, you'll recompute paths for regions- suboptimal
    # Almost want to handle BOTH here AND after making paths from regions (but
    # BEFORE generating the gcode -> helixes for each region)

    # Create discretized regions
    inside_discretized = list()
    for dtop, dbot, region in inside_regions:
        discretizedEdges = list()
        for w in region.Wires:
            for e in w.Edges:
                discretizedEdges.append([discretize(e)])
        inside_discretized.append((dtop, dbot, discretizedEdges))

    outside_discretized = list()
    for dtop, dbot, region in outside_regions:
        discretizedEdges = list()
        for a in region:
            for w in a.Wires:
                for e in w.Edges:
                    discretizedEdges.append([discretize(e)])
        outside_discretized.append((dtop, dbot, discretizedEdges))

    # Return found inside and outside regions/depths. Up to the caller to decide
    # which ones it cares about.
    return inside_discretized, outside_discretized


class PathAdaptive(PathOp.ObjectOp):
    def opFeatures(self, obj):
        """opFeatures(obj) ... returns the OR'ed list of features used and supported by the operation.
        The default implementation returns "FeatureTool | FeatureDepths | FeatureHeights | FeatureStartPoint"
        Should be overwritten by subclasses."""
        return (
            PathOp.FeatureTool
            | PathOp.FeatureBaseEdges
            | PathOp.FeatureDepths
            | PathOp.FeatureFinishDepth
            | PathOp.FeatureStepDown
            | PathOp.FeatureHeights
            | PathOp.FeatureBaseGeometry
            | PathOp.FeatureCoolant
            | PathOp.FeatureLocations
        )

    @classmethod
    def propertyEnumerations(self, dataType="data"):
        """helixOpPropertyEnumerations(dataType="data")... return property enumeration lists of specified dataType.
        Args:
            dataType = 'data', 'raw', 'translated'
        Notes:
        'data' is list of internal string literals used in code
        'raw' is list of (translated_text, data_string) tuples
        'translated' is list of translated string literals
        """

        # Enumeration lists for App::PropertyEnumeration properties
        enums = {
            "Side": [
                (translate("CAM_Adaptive", "Outside"), "Outside"),
                (translate("CAM_Adaptive", "Inside"), "Inside"),
            ],  # this is the direction that the profile runs
            "OperationType": [
                (translate("CAM_Adaptive", "Clearing"), "Clearing"),
                (translate("CAM_Adaptive", "Profiling"), "Profiling"),
            ],  # side of profile that cutter is on in relation to direction of profile
        }

        if dataType == "raw":
            return enums

        data = list()
        idx = 0 if dataType == "translated" else 1

        Path.Log.debug(enums)

        for k, v in enumerate(enums):
            data.append((v, [tup[idx] for tup in enums[v]]))
        Path.Log.debug(data)

        return data

    def initOperation(self, obj):
        """initOperation(obj) ... implement to create additional properties.
        Should be overwritten by subclasses."""
        obj.addProperty(
            "App::PropertyEnumeration",
            "Side",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Side of selected faces that tool should cut",
            ),
        )
        # obj.Side = [
        #     "Outside",
        #     "Inside",
        # ]  # side of profile that cutter is on in relation to direction of profile

        obj.addProperty(
            "App::PropertyEnumeration",
            "OperationType",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Type of adaptive operation",
            ),
        )
        # obj.OperationType = [
        #     "Clearing",
        #     "Profiling",
        # ]  # side of profile that cutter is on in relation to direction of profile

        obj.addProperty(
            "App::PropertyFloat",
            "Tolerance",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Influences accuracy and performance",
            ),
        )
        obj.addProperty(
            "App::PropertyPercent",
            "StepOver",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Percent of cutter diameter to step over on each pass",
            ),
        )
        obj.addProperty(
            "App::PropertyDistance",
            "LiftDistance",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Lift distance for rapid moves",
            ),
        )
        obj.addProperty(
            "App::PropertyDistance",
            "KeepToolDownRatio",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Max length of keep tool down path compared to direct distance between points",
            ),
        )
        obj.addProperty(
            "App::PropertyDistance",
            "StockToLeave",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "How much stock to leave (i.e. for finishing operation)",
            ),
        )
        obj.addProperty(
            "App::PropertyBool",
            "ForceInsideOut",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Force plunging into material inside and clearing towards the edges",
            ),
        )
        obj.addProperty(
            "App::PropertyBool",
            "FinishingProfile",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "To take a finishing profile path at the end",
            ),
        )
        obj.addProperty(
            "App::PropertyBool",
            "Stopped",
            "Adaptive",
            QT_TRANSLATE_NOOP("App::Property", "Stop processing"),
        )
        obj.setEditorMode("Stopped", 2)  # hide this property

        obj.addProperty(
            "App::PropertyBool",
            "StopProcessing",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Stop processing",
            ),
        )
        obj.setEditorMode("StopProcessing", 2)  # hide this property

        obj.addProperty(
            "App::PropertyBool",
            "UseHelixArcs",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Use Arcs (G2) for helix ramp",
            ),
        )

        obj.addProperty(
            "App::PropertyPythonObject",
            "AdaptiveInputState",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Internal input state",
            ),
        )
        obj.addProperty(
            "App::PropertyPythonObject",
            "AdaptiveOutputState",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Internal output state",
            ),
        )
        obj.setEditorMode("AdaptiveInputState", 2)  # hide this property
        obj.setEditorMode("AdaptiveOutputState", 2)  # hide this property
        obj.addProperty(
            "App::PropertyAngle",
            "HelixAngle",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Helix ramp entry angle (degrees)",
            ),
        )
        obj.addProperty(
            "App::PropertyAngle",
            "HelixConeAngle",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Helix cone angle (degrees)",
            ),
        )
        obj.addProperty(
            "App::PropertyLength",
            "HelixDiameterLimit",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Limit helix entry diameter, if limit larger than tool diameter or 0, tool diameter is used",
            ),
        )

        obj.addProperty(
            "App::PropertyBool",
            "UseOutline",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Uses the outline of the base geometry.",
            ),
        )

        obj.addProperty(
            "Part::PropertyPartShape",
            "removalshape",
            "Path",
            QT_TRANSLATE_NOOP("App::Property", ""),
        )

        for n in self.propertyEnumerations():
            setattr(obj, n[0], n[1])

        obj.setEditorMode("removalshape", 2)  # hide

        FeatureExtensions.initialize_properties(obj)

    def opSetDefaultValues(self, obj, job):
        obj.Side = "Inside"
        obj.OperationType = "Clearing"
        obj.Tolerance = 0.1
        obj.StepOver = 20
        obj.LiftDistance = 0
        # obj.ProcessHoles = True
        obj.ForceInsideOut = False
        obj.FinishingProfile = True
        obj.Stopped = False
        obj.StopProcessing = False
        obj.HelixAngle = 5
        obj.HelixConeAngle = 0
        obj.HelixDiameterLimit = 0.0
        obj.AdaptiveInputState = ""
        obj.AdaptiveOutputState = ""
        obj.StockToLeave = 0
        obj.KeepToolDownRatio = 3.0
        obj.UseHelixArcs = False
        obj.UseOutline = False
        FeatureExtensions.set_default_property_values(obj, job)

    def opExecute(self, obj):
        """opExecute(obj) ... called whenever the receiver needs to be recalculated.
        See documentation of execute() for a list of base functionality provided.
        Should be overwritten by subclasses."""

        # inside is a list of (max depth, region) tuples of pockets to machine,
        # including any selections made by the user and accounting for stock.
        # outside is the same, except for the outside of the model
        inside, outside = _get_working_edges(self, obj)

        self.insidePathArray = inside
        self.outsidePathArray = outside

        Execute(self, obj)

    def opOnDocumentRestored(self, obj):
        if not hasattr(obj, "HelixConeAngle"):
            obj.addProperty(
                "App::PropertyAngle",
                "HelixConeAngle",
                "Adaptive",
                "Helix cone angle (degrees)",
            )

        if not hasattr(obj, "UseOutline"):
            obj.addProperty(
                "App::PropertyBool",
                "UseOutline",
                "Adaptive",
                "Uses the outline of the base geometry.",
            )

        if not hasattr(obj, "removalshape"):
            obj.addProperty("Part::PropertyPartShape", "removalshape", "Path", "")
        obj.setEditorMode("removalshape", 2)  # hide

        FeatureExtensions.initialize_properties(obj)


# Eclass


def SetupProperties():
    setup = [
        "Side",
        "OperationType",
        "Tolerance",
        "StepOver",
        "LiftDistance",
        "KeepToolDownRatio",
        "StockToLeave",
        "ForceInsideOut",
        "FinishingProfile",
        "Stopped",
        "StopProcessing",
        "UseHelixArcs",
        "AdaptiveInputState",
        "AdaptiveOutputState",
        "HelixAngle",
        "HelixConeAngle",
        "HelixDiameterLimit",
        "UseOutline",
    ]
    return setup


def Create(name, obj=None, parentJob=None):
    """Create(name) ... Creates and returns a Adaptive operation."""
    if obj is None:
        obj = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", name)
    obj.Proxy = PathAdaptive(obj, name, parentJob)
    return obj

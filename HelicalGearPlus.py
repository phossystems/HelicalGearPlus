# Author Nico Schlueter 2020
#
# Released under the MIT license. See License.txt for full license information.
#
# Description-Generates straight, helical and herringbone external, internal and rack gears
# as well as non-enveloping worms and worm gears
#
# Parts (mostly helical gear calculatiom) was taken from Ross Korsky's Helical gear generator
# Parts (mostly some of the Involute code) was taken from AutoDesks' Fusion 360 SpurGear sample script.
# The primary source used to produce this add-in was http://qtcgears.com/tools/catalogs/PDF_Q420/Tech.pdf

import adsk.core, adsk.fusion, traceback
import math

# Global set of event handlers to keep them referenced for the duration of the command
_handlers = []

# Caches last gear for 
lastGear = None
lastInput = ""

COMMAND_ID = "helicalGearPlus"
COMMAND_NAME = "Helical Gear+"
COMMAND_TOOLTIP = "Generates Helical Gears"
TOOLBAR_PANELS = ["SolidCreatePanel"]

# Initial persistence Dict
pers = {
    'DDType': "External Gear",
    'DDStandard': "Normal",
    'VIHelixAngle': 0.5235987755982988,
    'VIPressureAngle': 0.3490658503988659,
    'VIModule': 0.3,
    'ISTeeth': 16,
    'VIBacklash': 0.0,
    'VIWidth': 1.0,
    'VIHeight': 0.8,
    'VILength': 10.0,
    'VIDiameter': 8.0,
    'BVHerringbone': False,
    'BVPreview': False,
    'VIAddendum': 1,
    'VIDedendum': 1.25}


class Involute:
    def __init__(self, gear):
        self.gear = gear

    def draw(self, sketch, z_shift=0, rotation=0, involutePointCount=10):
        # Calculate points along the involute curve.
        origin_point = adsk.core.Point3D.create(0, 0, z_shift)
        involutePoints = []
        key_points = []

        if self.gear.base_diameter >= self.gear.root_diameter:
            involute_from_rad = self.gear.base_diameter / 2.0
        else:
            involute_from_rad = self.gear.root_diameter / 2
        radiusStep = (self.gear.outside_diameter / 2 - involute_from_rad) / (involutePointCount - 1)
        involuteIntersectionRadius = involute_from_rad
        for i in range(0, involutePointCount):
            newPoint = self._involute_point(self.gear.base_diameter / 2.0, involuteIntersectionRadius, z_shift)
            involutePoints.append(newPoint)
            involuteIntersectionRadius = involuteIntersectionRadius + radiusStep

        # Determine the angle between the X axis and a line between the origin of the curve
        # and the intersection point between the involute and the pitch diameter circle.
        pitchInvolutePoint = self._involute_point(self.gear.base_diameter / 2.0, self.gear.pitch_diameter / 2.0,
                                                  z_shift)
        pitchPointAngle = math.atan2(pitchInvolutePoint.y, pitchInvolutePoint.x)

        # Rotate the involute so the intersection point lies on the x axis.
        rotateAngle = -((self.gear.tooth_arc_angle / 4) + pitchPointAngle - (self.gear.backlash_angle / 4))
        cosAngle = math.cos(rotateAngle)
        sinAngle = math.sin(rotateAngle)
        for i in range(0, involutePointCount):
            x = involutePoints[i].x
            y = involutePoints[i].y
            involutePoints[i].x = x * cosAngle - y * sinAngle
            involutePoints[i].y = x * sinAngle + y * cosAngle

        # Create a new set of points with a negated y.  This effectively mirrors the original
        # points about the X axis.
        involute2Points = []
        for i in range(0, involutePointCount):
            involute2Points.append(adsk.core.Point3D.create(involutePoints[i].x, -involutePoints[i].y, z_shift))

        # Rotate involute
        if rotation:
            cosAngle = math.cos(rotation)
            sinAngle = math.sin(rotation)
            for i in range(0, involutePointCount):
                x = involutePoints[i].x
                y = involutePoints[i].y
                involutePoints[i].x = x * cosAngle - y * sinAngle
                involutePoints[i].y = x * sinAngle + y * cosAngle
                x = involute2Points[i].x
                y = involute2Points[i].y
                involute2Points[i].x = x * cosAngle - y * sinAngle
                involute2Points[i].y = x * sinAngle + y * cosAngle

        curve1Angle = math.atan2(involutePoints[0].y, involutePoints[0].x)
        curve2Angle = math.atan2(involute2Points[0].y, involute2Points[0].x)
        if curve2Angle < curve1Angle:
            curve2Angle += math.pi * 2

        # Create and load an object collection with the points.
        # Add the involute points for the second spline to an ObjectCollection.
        pointSet1 = adsk.core.ObjectCollection.create()
        pointSet2 = adsk.core.ObjectCollection.create()
        for i in range(0, involutePointCount):
            pointSet1.add(involutePoints[i])
            pointSet2.add(involute2Points[i])

        mid_index = int(pointSet1.count / 2)
        key_points.append(pointSet1.item(0))
        key_points.append(pointSet2.item(0))
        key_points.append(pointSet1.item(mid_index))
        key_points.append(pointSet2.item(mid_index))

        # Create splines.
        spline1 = sketch.sketchCurves.sketchFittedSplines.add(pointSet1)
        spline2 = sketch.sketchCurves.sketchFittedSplines.add(pointSet2)
        oc = adsk.core.ObjectCollection.create()
        oc.add(spline2)
        (_, _, cross_points) = spline1.intersections(oc)
        assert len(cross_points) == 0 or len(cross_points) == 1, 'Failed to compute a valid involute profile!'
        if len(cross_points) == 1:
            # involute splines cross, clip the tooth
            # clip = spline1.endSketchPoint.geometry.copy()
            # spline1 = spline1.trim(spline2.endSketchPoint.geometry).item(0)
            # spline2 = spline2.trim(clip).item(0)
            key_points.append(cross_points[0])
        else:
            # Draw the tip of the tooth - connect the splines
            if self.gear.tooth_count >= 100:
                sketch.sketchCurves.sketchLines.addByTwoPoints(spline1.endSketchPoint, spline2.endSketchPoint)
                key_points.append(spline1.endSketchPoint.geometry)
                key_points.append(spline2.endSketchPoint.geometry)
            else:
                tip_curve1Angle = math.atan2(involutePoints[-1].y, involutePoints[-1].x)
                tip_curve2Angle = math.atan2(involute2Points[-1].y, involute2Points[-1].x)
                if tip_curve2Angle < tip_curve1Angle:
                    tip_curve2Angle += math.pi * 2
                tip_rad = origin_point.distanceTo(involutePoints[-1])
                tip_arc = sketch.sketchCurves.sketchArcs.addByCenterStartSweep(
                    origin_point,
                    adsk.core.Point3D.create(math.cos(tip_curve1Angle) * tip_rad,
                                             math.sin(tip_curve1Angle) * tip_rad,
                                             z_shift),
                    tip_curve2Angle - tip_curve1Angle)
                key_points.append(tip_arc.startSketchPoint.geometry)
                key_points.append(adsk.core.Point3D.create(tip_rad, 0, z_shift))
                key_points.append(tip_arc.endSketchPoint.geometry)

        # Draw root circle
        # root_circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(origin_point, self.gear.root_diameter/2)
        root_arc = sketch.sketchCurves.sketchArcs.addByCenterStartSweep(
            origin_point,
            adsk.core.Point3D.create(math.cos(curve1Angle) * (self.gear.root_diameter / 2 - 0.01),
                                     math.sin(curve1Angle) * (self.gear.root_diameter / 2 - 0.01),
                                     z_shift),
            curve2Angle - curve1Angle)

        # if the offset tooth profile crosses the offset circle then trim it, else connect the offset tooth to the circle
        oc = adsk.core.ObjectCollection.create()
        oc.add(spline1)
        if True:
            if root_arc.intersections(oc)[1].count > 0:
                spline1 = spline1.trim(origin_point).item(0)
                spline2 = spline2.trim(origin_point).item(0)
                root_arc.trim(root_arc.startSketchPoint.geometry)
                root_arc.trim(root_arc.endSketchPoint.geometry)
            else:
                sketch.sketchCurves.sketchLines.addByTwoPoints(origin_point, spline1.startSketchPoint).trim(
                    origin_point)
                sketch.sketchCurves.sketchLines.addByTwoPoints(origin_point, spline2.startSketchPoint).trim(
                    origin_point)
        else:
            if root_arc.intersections(oc)[1].count > 0:
                spline1 = spline1.trim(origin_point).item(0)
                spline2 = spline2.trim(origin_point).item(0)
            root_arc.deleteMe()
            sketch.sketchCurves.sketchLines.addByTwoPoints(origin_point, spline1.startSketchPoint)
            sketch.sketchCurves.sketchLines.addByTwoPoints(origin_point, spline2.startSketchPoint)

    # Calculate points along an involute curve.
    def _involute_point(self, baseCircleRadius, distFromCenterToInvolutePoint, z_shift):
        l = math.sqrt(
            distFromCenterToInvolutePoint * distFromCenterToInvolutePoint - baseCircleRadius * baseCircleRadius)
        alpha = l / baseCircleRadius
        theta = alpha - math.acos(baseCircleRadius / distFromCenterToInvolutePoint)
        x = distFromCenterToInvolutePoint * math.cos(theta)
        y = distFromCenterToInvolutePoint * math.sin(theta)
        return adsk.core.Point3D.create(x, y, z_shift)


class HelicalGear:
    def __init__(self):
        pass

    @property
    def is_undercut_requried(self):
        return self.virtual_teeth < self.critcal_virtual_tooth_count

    @property
    def backlash_angle(self):
        """The backlash is split between both sides of this and (an assumed) mateing gear - each side of a tooth will be narrowed by 1/4 this value."""
        return 2 * self.backlash / self.pitch_diameter if self.pitch_diameter > 0 else 0

    @property
    def tooth_arc_angle(self):
        """Arc angle of a single tooth."""
        return 2 * math.pi / self.tooth_count if self.tooth_count > 0 else 0

    @property
    def tip_pressure_angle(self):
        """Pressure angle at the tip of the tooth."""
        return math.acos(self.base_diameter / self.outside_diameter)

    @property
    def involute_a(self):
        """Involute at nominal pressure angle."""
        return math.tan(self.pressure_angle) - self.pressure_angle

    @property
    def involute_aa(self):
        """Involute at tip pressure angle."""
        return math.tan(self.tip_pressure_angle) - self.tip_pressure_angle

    @property
    def profile_shift_coefficient(self):
        """Profile shift coefficient without undercut."""
        return 1 - (self.tooth_count / 2) * math.pow(math.sin(self.pressure_angle), 2)

    @property
    def top_land_angle(self):
        """Top land is the (sometimes flat) surface of the top of a gear tooth.
        DOES NOT APPEAR TO PRODUCE THE CORRECT VALUE."""
        return (math.pi / (2 * self.tooth_count)) + (
                (2 * self.profile_shift_coefficient * math.tan(self.pressure_angle)) / self.tooth_count) + (
                       self.involute_a - self.involute_aa)

    @property
    def top_land_thickness(self):
        """Top land is the (sometimes flat) surface of the top of a gear tooth.
        DOES NOT APPEAR TO PRODUCE THE CORRECT VALUE."""
        return math.radians(self.top_land_angle) * self.outside_diameter

    @property
    def critcal_virtual_tooth_count(self):
        q = math.pow(math.sin(self.normal_pressure_angle), 2)
        return 2 / q if q != 0 else float('inf')

    @property
    def is_invalid(self):
        if (self.width <= 0):
            return "Width too low"
        if (math.radians(-90) > self.helix_angle):
            return "Helix angle too low"
        if (math.radians(90) < self.helix_angle):
            return "Helix angle too high"
        if (self.module <= 0):
            return "Module to low"
        if (self.addendum <= 0):
            return "Addendum too low"
        if (self.whole_depth <= 0):
            return "Dedendum too low"
        if (self.pressure_angle < 0):
            return "Pressure angle too low"
        if (self.pressure_angle > math.radians(80)):
            return "Pressure angle too high"
        if (self.normal_pressure_angle < 0):
            return "Pressure angle too low"
        if (self.normal_pressure_angle > math.radians(80)):
            return "Pressure angle too high"
        if (self.tooth_count <= 0):
            return "Too few teeth"
        if (abs(self.backlash_angle) / 4 >= self.tooth_arc_angle / 8):
            return "Backlash too high"
        if (self.internal_outside_diameter):
            if (self.internal_outside_diameter <= self.outside_diameter):
                return "Outside diameter too low"
        if (self.circular_pitch <= 0):
            return "Invalid: circular_pitch"
        if (self.base_diameter <= 0):
            return "Invalid Gear"
        if (self.pitch_diameter <= 0):
            return "Invalid Gear"
        if (self.root_diameter <= 0.03):
            return "Invalid Gear"
        if (self.outside_diameter <= 0):
            return "Invalid Gear"
        return False

    @property
    def vertical_loop_seperation(self):
        return math.tan(math.radians(90) + self.helix_angle) * self.pitch_diameter * math.pi

    # returns the number of turns for a given distance
    def t_for(self, displacement):
        return displacement / (math.tan(math.radians(90) + self.helix_angle) * (self.pitch_diameter / 2))

    def __str__(self):
        str = ''
        str += '\n'
        str += 'root diameter..............:  {0:.3f} mm\n'.format(self.root_diameter * 10)
        str += 'base diameter.............:  {0:.3f} mm\n'.format(self.base_diameter * 10)
        str += 'pitch diameter............:  {0:.3f} mm\n'.format(self.pitch_diameter * 10)
        str += 'outside diameter.........:  {0:.3f} mm\n'.format(self.outside_diameter * 10)
        str += '\n'
        str += 'module.......................:  {0:.3f} mm\n'.format(self.module * 10)
        str += 'normal module...........:  {0:.3f} mm\n'.format(self.normal_module * 10)
        str += 'pressure angle............:  {0:.3f} deg\n'.format(math.degrees(self.pressure_angle))
        str += 'normal pressure angle:  {0:.3f} deg\n'.format(math.degrees(self.normal_pressure_angle))
        str += '\n'
        if (self.helix_angle != 0):
            str += 'length per revolution..:  {0:.3f} mm\n'.format(abs(self.vertical_loop_seperation) * 10)
            str += '\n'
        return str

    @staticmethod
    def create_in_normal_system(tooth_count, normal_module, normal_pressure_angle, helix_angle, backlash=0, addendum=1,
                                dedendum=1.25, width=1, herringbone=False, internal_outside_diameter=None):
        tooth_count = tooth_count if tooth_count > 0 else 1
        # normal_module = normal_module if normal_module > 0 else 1e-10
        # normal_pressure_angle = normal_pressure_angle if 0 <= normal_pressure_angle < math.radians(90) else 0
        # helix_angle = helix_angle if math.radians(-90) < helix_angle < math.radians(90) else 0

        gear = HelicalGear()
        gear.backlash = backlash
        gear.helix_angle = helix_angle
        gear.tooth_count = tooth_count
        gear.width = width
        gear.herringbone = herringbone
        gear.internal_outside_diameter = internal_outside_diameter

        gear.normal_module = normal_module
        gear.normal_pressure_angle = normal_pressure_angle

        gear.normal_circular_pitch = gear.normal_module * math.pi
        cos_helix_angle = math.cos(helix_angle)
        gear.virtual_teeth = gear.tooth_count / math.pow(cos_helix_angle, 3)

        # Radial / Transverse figures
        gear.module = gear.normal_module / cos_helix_angle
        gear.pressure_angle = math.atan2(math.tan(gear.normal_pressure_angle), cos_helix_angle)
        gear.pitch_diameter = gear.module * gear.tooth_count
        gear.base_diameter = gear.pitch_diameter * math.cos(gear.pressure_angle)
        gear.addendum = addendum * gear.normal_module
        gear.whole_depth = (addendum + dedendum) * gear.normal_module
        gear.outside_diameter = gear.pitch_diameter + 2 * gear.addendum
        gear.root_diameter = gear.outside_diameter - 2 * gear.whole_depth
        gear.circular_pitch = gear.module * math.pi

        return gear

    @staticmethod
    def create_in_radial_system(tooth_count, radial_module, radial_pressure_angle, helix_angle, backlash=0, addendum=1,
                                dedendum=1.25, width=1, herringbone=False, internal_outside_diameter=None):
        tooth_count = tooth_count if tooth_count > 0 else 1
        radial_module = radial_module if radial_module > 0 else 1e-10
        radial_pressure_angle = radial_pressure_angle if 0 <= radial_pressure_angle < math.radians(90) else 0
        helix_angle = helix_angle if math.radians(-90) < helix_angle < math.radians(90) else 0

        gear = HelicalGear()
        gear.backlash = backlash
        gear.helix_angle = helix_angle
        gear.tooth_count = tooth_count
        gear.width = width
        gear.herringbone = herringbone
        gear.internal_outside_diameter = internal_outside_diameter

        gear.normal_module = radial_module * math.cos(gear.helix_angle)
        gear.normal_pressure_angle = math.atan(math.tan(radial_pressure_angle) * math.cos(gear.helix_angle))
        gear.normal_circular_pitch = gear.normal_module * math.pi

        cos_helix_angle = math.cos(helix_angle)
        gear.virtual_teeth = gear.tooth_count / math.pow(cos_helix_angle, 3)

        # Radial / Transverse figures
        gear.module = radial_module
        gear.pressure_angle = radial_pressure_angle
        gear.pitch_diameter = gear.module * gear.tooth_count
        gear.base_diameter = gear.pitch_diameter * math.cos(gear.pressure_angle)
        gear.addendum = gear.module
        gear.whole_depth = 2.25 * gear.module
        gear.outside_diameter = gear.pitch_diameter + 2 * gear.addendum
        gear.root_diameter = gear.outside_diameter - 2 * gear.whole_depth
        gear.circular_pitch = gear.module * math.pi

        return gear

    def model_gear(self, parent_component, same_as_last=False):
        # Storres a copy of the last gear generated to speed up regeneation of the same gear
        global lastGear 
        
        # The temporaryBRep manager is a tool for creating 3d geometry without the use of features
        # The word temporary referrs to the geometry being created being virtual, but It can easily be converted to actual geometry
        tbm = adsk.fusion.TemporaryBRepManager.get()
        # Create new component
        occurrence = parent_component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        component = occurrence.component
        component.name = 'Healical Gear ({0}{1}@{2:.2f} m={3})'.format(
            self.tooth_count,
            'L' if self.helix_angle < 0 else 'R',
            abs(math.degrees(self.helix_angle)),
            round(self.normal_module * 10, 4))

        # Creates BaseFeature if DesignType is parametric 
        if (parent_component.parentDesign.designType):
            base_feature = component.features.baseFeatures.add()
            base_feature.startEdit()
        else:
            base_feature = None

        if(not (same_as_last and lastGear)):

            # Creates sketch and draws tooth profile
            involute = Involute(self)
            
            # Creates profile on z=0 if herringbone and on bottom if not
            if(not self.herringbone):
                plane = adsk.core.Plane.create(adsk.core.Point3D.create(0, 0, -self.width/2),
                                        adsk.core.Vector3D.create(0,0,1))
                # Creates an object responsible for passing all required data to create a construction plane
                planeInput = component.constructionPlanes.createInput()
                # Sets the plane input by plane
                planeInput.setByPlane(plane)
                # Adds plain input to construction planes
                cPlane = component.constructionPlanes.add(planeInput)
                sketch = component.sketches.add(cPlane)
                cPlane.deleteMe()
                sketch.isComputeDeferred = True
                # Draws All Teeth
                # TODO: Optimize by copying instead of regenerating
                for i in range(self.tooth_count):
                    involute.draw(sketch, 0, (i / self.tooth_count) * 2 * math.pi)
                # Base Circle
                sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), self.root_diameter / 2)
            else:
                sketch = component.sketches.add(component.xYConstructionPlane)
                sketch.isComputeDeferred = True
                # Draws All Teeth
                # TODO: Optimize by copying instead of regenerating
                for i in range(self.tooth_count):
                    involute.draw(sketch, 0, (i / self.tooth_count) * 2 * math.pi)
                # Base Circle
                sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), self.root_diameter / 2)

            # Creates path line for sweep feature
            if (not self.herringbone):
                line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0),
                                                                    adsk.core.Point3D.create(0, 0, self.width))
            else: 
                line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0),
                                                                    adsk.core.Point3D.create(0, 0, self.width / 2))
                line2 = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0),
                                                                    adsk.core.Point3D.create(0, 0, -self.width / 2))

            # Reactivates sketch computation and puts all profules into an OC              
            sketch.isComputeDeferred = False
            profs = adsk.core.ObjectCollection.create()
            for prof in sketch.profiles:
                profs.add(prof)

            # Creates sweeep features
            if(not self.herringbone):
                path1 = component.features.createPath(line1)
                sweepInput = component.features.sweepFeatures.createInput(profs, path1,
                                                                        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
                sweepInput.twistAngle = adsk.core.ValueInput.createByReal(-self.t_for(self.width))
                if (base_feature):
                    sweepInput.targetBaseFeature = base_feature
                gearBody = sweepFeature = component.features.sweepFeatures.add(sweepInput).bodies.item(0)
            else:
                path1 = component.features.createPath(line1)
                sweepInput = component.features.sweepFeatures.createInput(profs, path1,
                                                                        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
                sweepInput.twistAngle = adsk.core.ValueInput.createByReal(-self.t_for(self.width / 2))
                if (base_feature):
                    sweepInput.targetBaseFeature = base_feature
                sweepFeature = component.features.sweepFeatures.add(sweepInput)

                path2 = component.features.createPath(line2)
                sweepInput = component.features.sweepFeatures.createInput(profs, path2,
                                                                        adsk.fusion.FeatureOperations.JoinFeatureOperation)
                sweepInput.twistAngle = adsk.core.ValueInput.createByReal(self.t_for(self.width / 2))
                if (base_feature):
                    sweepInput.targetBaseFeature = base_feature
                gearBody = sweepFeature = component.features.sweepFeatures.add(sweepInput).bodies.item(0)

            # "Inverts" internal Gears
            if (self.internal_outside_diameter):
                cyl = cylinder = tbm.createCylinderOrCone(adsk.core.Point3D.create(0, 0, -self.width / 2),
                                                        self.internal_outside_diameter / 2,
                                                        adsk.core.Point3D.create(0, 0, self.width / 2),
                                                        self.internal_outside_diameter / 2)
                tbm.booleanOperation(cyl, tbm.copy(gearBody), 0)
                # Deletes external gear
                gearBody.deleteMe()

                if (base_feature):
                    gearBody = component.bRepBodies.add(cyl, base_feature)
                else:
                    gearBody = component.bRepBodies.add(cyl)

            # Delete tooth sketch for performance
            sketch.deleteMe()

            # Draws pitch diameter
            pitch_diameter_sketch = component.sketches.add(component.xYConstructionPlane)
            pitch_diameter_sketch.name = "PD: {0:.3f}mm".format(self.pitch_diameter * 10)
            pitch_diameter_circle = pitch_diameter_sketch.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(0, 0, 0), self.pitch_diameter / 2)
            pitch_diameter_circle.isConstruction = True
            pitch_diameter_circle.isFixed = True

            # Storres a copy of the newly generated gear    
      
            lastGear = tbm.copy(gearBody)
        else:
            if(base_feature):
                component.bRepBodies.add(lastGear, base_feature)
            else:
                component.bRepBodies.add(lastGear)

        # Finishes BaseFeature if it exists
        if (base_feature):
            base_feature.finishEdit()

        return occurrence


class RackGear:

    def __init__(self):
        pass

    @staticmethod
    def create_in_normal_system(normal_module, normal_pressure_angle, helix_angle, herringbone, length, width, height,
                                backlash=0, addendum=1, dedendum=1.25):
        gear = RackGear()

        gear.normal_module = normal_module
        gear.normal_pressure_angle = normal_pressure_angle
        gear.helix_angle = helix_angle
        gear.herringbone = herringbone
        gear.length = length
        gear.width = width
        gear.height = height
        gear.backlash = backlash

        gear.addendum = addendum * gear.normal_module
        gear.dedendum = dedendum * gear.normal_module

        cos_helix_angle = math.cos(helix_angle)
        gear.module = gear.normal_module / cos_helix_angle
        gear.pressure_angle = math.atan2(math.tan(gear.normal_pressure_angle), cos_helix_angle)

        return gear

    @staticmethod
    def create_in_radial_system(radial_module, radial_pressure_angle, helix_angle, herringbone, length, width, height,
                                backlash=0, addendum=1, dedendum=1.25):
        gear = RackGear()

        gear.module = radial_module
        gear.pressure_angle = radial_pressure_angle
        gear.helix_angle = helix_angle
        gear.herringbone = herringbone
        gear.length = length
        gear.width = width
        gear.height = height
        gear.backlash = backlash

        gear.addendum = addendum * gear.module
        gear.dedendum = dedendum * gear.module

        cos_helix_angle = math.cos(helix_angle)
        gear.normal_module = gear.module * cos_helix_angle
        gear.normal_pressure_angle = math.atan(math.tan(radial_pressure_angle) * math.cos(gear.helix_angle))

        return gear

    def __str__(self):
        str = ''
        str += '\n'
        str += 'module.......................:  {0:.3f} mm\n'.format(self.module * 10)
        str += 'normal module...........:  {0:.3f} mm\n'.format(self.normal_module * 10)
        str += 'pressure angle............:  {0:.3f} deg\n'.format(math.degrees(self.pressure_angle))
        str += 'normal pressure angle:  {0:.3f} deg\n'.format(math.degrees(self.normal_pressure_angle))
        str += '\n'
        return str

    @property
    def is_invalid(self):
        if (self.length <= 0):
            return "Length too low"
        if (self.width <= 0):
            return "Width too low"
        if (self.height <= 0):
            return "Height too low"
        if (self.module <= 0):
            return "Module too low"
        if (self.addendum < 0):
            return "Addendum too low"
        if (self.dedendum < 0):
            return "Dedendum too low"
        if (self.addendum + self.dedendum <= 0):
            return "Addendum too low"
        if (not (0 < self.pressure_angle < math.radians(90))):
            return "Invalid pressure angle"
        if (not (math.radians(-90) < self.helix_angle < math.radians(90))):
            return "Invalid helix angle"
        # Not actually the limit but close enough
        if ((-3 * self.normal_module) > self.backlash):
            return "Backlash too low"
        if (self.backlash > (3 * self.normal_module)):
            return "Backlash too high"
        return False

    def rackLines(self, x, y, z, m, n, height, pAngle, hAngle, backlash, addendum, dedendum):
        strech = 1 / math.cos(hAngle)
        P = m * math.pi

        # Clamps addendum and dedendum
        addendum = min(addendum, (-(1 / 4) * (backlash - P) * (1 / math.tan(pAngle))) - 0.0001)
        dedendum = min(dedendum, -(1 / 4) * (-backlash - P) * (1 / math.tan(pAngle)) - 0.0001)
        dedendum = min(dedendum, height - 0.0001)

        lines = []

        for i in range(n):
            # Root
            lines.append(
                adsk.core.Line3D.create(adsk.core.Point3D.create(x + ((i * P)) * strech, y, z - dedendum),
                                        adsk.core.Point3D.create(x + ((i * P) + (P / 2) + backlash / 2 - (
                                                math.tan(pAngle) * 2 * dedendum)) * strech, y, z - dedendum))
            )
            # Left Edge
            lines.append(
                adsk.core.Line3D.create(adsk.core.Point3D.create(
                    x + ((i * P) + (P / 2) + backlash / 2 - (math.tan(pAngle) * 2 * dedendum)) * strech, y,
                    z - dedendum),
                    adsk.core.Point3D.create(x + ((i * P) + (P / 2) + backlash / 2 - (
                            math.tan(pAngle) * (dedendum - addendum))) * strech, y,
                                             z + addendum))
            )
            # Tip
            lines.append(
                adsk.core.Line3D.create(adsk.core.Point3D.create(
                    x + ((i * P) + (P / 2) + backlash / 2 - (math.tan(pAngle) * (dedendum - addendum))) * strech, y,
                    z + addendum),
                    adsk.core.Point3D.create(x + ((i * P) + P - (
                            math.tan(pAngle) * (dedendum + addendum))) * strech, y,
                                             z + addendum))
            )
            # Right Edge
            lines.append(
                adsk.core.Line3D.create(adsk.core.Point3D.create(
                    x + ((i * P) + P - (math.tan(pAngle) * (dedendum + addendum))) * strech, y,
                    z + addendum),
                    adsk.core.Point3D.create(x + ((i * P) + P) * strech, y, z - dedendum))
            )
            # Right Edge
        lines.append(
            adsk.core.Line3D.create(adsk.core.Point3D.create(x + (n * P) * strech, y, z - dedendum),
                                    adsk.core.Point3D.create(x + (n * P) * strech, y, z - height))
        )
        # Bottom Edge
        lines.append(
            adsk.core.Line3D.create(adsk.core.Point3D.create(x + (n * P) * strech, y, z - height),
                                    adsk.core.Point3D.create(x, y, z - height))
        )
        # Left Edge
        lines.append(
            adsk.core.Line3D.create(adsk.core.Point3D.create(x, y, z - height),
                                    adsk.core.Point3D.create(x, y, z - dedendum))
        )
        return lines

    def model_gear(self, parent_component, same_as_last=False):
        # Storres a copy of the last gear generated to speed up regeneation of the same gear
        global lastGear 

        # Create new component
        occurrence = parent_component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        component = occurrence.component
        component.name = 'Healical Rack ({0}mm {1}@{2:.2f} m={3})'.format(
            self.length * 10,
            'L' if self.helix_angle < 0 else 'R',
            abs(math.degrees(self.helix_angle)),
            round(self.normal_module * 10, 4))
        if (parent_component.parentDesign.designType):
            base_feature = component.features.baseFeatures.add()
            base_feature.startEdit()
        else:
            base_feature = None

        if(not (same_as_last and lastGear)):

            teeth = math.ceil(
                (self.length + 2 * math.tan(abs(self.helix_angle)) * self.width) / (self.normal_module * math.pi))
            # The temporaryBRep manager is a tool for creating 3d geometry without the use of features
            # The word temporary referrs to the geometry being created being virtual, but It can easily be converted to actual geometry
            tbm = adsk.fusion.TemporaryBRepManager.get()
            # Array to keep track of TempBRepBodies
            tempBRepBodies = []
            # Creates BRep wire object(s), representing edges in 3D space from an array of 3Dcurves
            if (self.herringbone):
                wireBody1, _ = tbm.createWireFromCurves(self.rackLines(
                    -self.length / 2 - (math.tan(abs(self.helix_angle)) + math.tan(self.helix_angle)) * self.width / 2,
                    -self.width / 2,
                    0,
                    self.normal_module, teeth, self.height, self.normal_pressure_angle, self.helix_angle,
                    self.backlash, self.addendum, self.dedendum
                ))
                wireBody2, _ = tbm.createWireFromCurves(self.rackLines(
                    -self.length / 2 - math.tan(abs(self.helix_angle)) * self.width / 2,
                    0,
                    0,
                    self.normal_module, teeth, self.height, self.normal_pressure_angle, self.helix_angle,
                    self.backlash, self.addendum,
                    self.dedendum
                ))
                wireBody3, _ = tbm.createWireFromCurves(self.rackLines(
                    -self.length / 2 - (math.tan(abs(self.helix_angle)) + math.tan(self.helix_angle)) * self.width / 2,
                    self.width / 2,
                    0,
                    self.normal_module, teeth, self.height, self.normal_pressure_angle, self.helix_angle,
                    self.backlash, self.addendum, self.dedendum
                ))
            else:
                wireBody1, _ = tbm.createWireFromCurves(self.rackLines(
                    -self.length / 2 - (math.tan(abs(self.helix_angle)) + math.tan(self.helix_angle)) * self.width,
                    -self.width / 2,
                    0,
                    self.normal_module, teeth, self.height, self.normal_pressure_angle, self.helix_angle,
                    self.backlash, self.addendum, self.dedendum
                ))
                wireBody2, _ = tbm.createWireFromCurves(self.rackLines(
                    -self.length / 2 - math.tan(abs(self.helix_angle)) * self.width,
                    self.width / 2,
                    0,
                    self.normal_module, teeth, self.height, self.normal_pressure_angle, self.helix_angle,
                    self.backlash, self.addendum,
                    self.dedendum
                ))

            # Creates the planar end caps.
            tempBRepBodies.append(tbm.createFaceFromPlanarWires([wireBody1]))
            if (self.herringbone):
                tempBRepBodies.append(tbm.createFaceFromPlanarWires([wireBody3]))
            else:
                tempBRepBodies.append(tbm.createFaceFromPlanarWires([wireBody2]))
            # Creates the ruled surface connectiong the two end caps
            tempBRepBodies.append(tbm.createRuledSurface(wireBody1.wires.item(0), wireBody2.wires.item(0)))
            if (self.herringbone):
                tempBRepBodies.append(tbm.createRuledSurface(wireBody2.wires.item(0), wireBody3.wires.item(0)))
            # Turns surfaces into real BRep so they can be boundary filled
            tools = adsk.core.ObjectCollection.create()
            for b in tempBRepBodies:
                if (base_feature):
                    tools.add(component.bRepBodies.add(b, base_feature))
                else:
                    tools.add(component.bRepBodies.add(b))
            # Boundary fills enclosed voulume
            boundaryFillInput = component.features.boundaryFillFeatures.createInput(tools,
                                                                                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            if base_feature:
                boundaryFillInput.targetBaseFeature = base_feature
            boundaryFillInput.bRepCells.item(0).isSelected = True
            body = component.features.boundaryFillFeatures.add(boundaryFillInput).bodies.item(0)
            # Creates a box to cut off angled ends
            obb = adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(0, 0, 0),
                                                        adsk.core.Vector3D.create(1, 0, 0),
                                                        adsk.core.Vector3D.create(0, 1, 0),
                                                        self.length, self.width * 2, (self.height + self.addendum) * 2)
            box = tbm.createBox(obb)
            tbm.booleanOperation(box, tbm.copy(body), 1)
            if (base_feature):
                gearBody = component.bRepBodies.add(box, base_feature)
            else:
                gearBody = component.bRepBodies.add(box)
            body.deleteMe()
            # Deletes tooling bodies
            for b in tools:
                b.deleteMe()
            
            # Adds "pitch diameter" line
            pitch_diameter_sketch = component.sketches.add(component.xYConstructionPlane)
            pitch_diameter_sketch.name = "Pitch Diameter Line"
            pitch_diameter_line = pitch_diameter_sketch.sketchCurves.sketchLines.addByTwoPoints(
                adsk.core.Point3D.create(-self.length / 2, 0, 0),
                adsk.core.Point3D.create(self.length / 2, 0, 0)
            )
            pitch_diameter_line.isFixed = True
            pitch_diameter_line.isConstruction = True

            # Storres a copy of the newly generated gear            
            lastGear = tbm.copy(gearBody)
        else:
            if(base_feature):
                component.bRepBodies.add(lastGear, base_feature)
            else:
                component.bRepBodies.add(lastGear)

        if (base_feature):
            base_feature.finishEdit()

        return occurrence


# Fires when the CommandDefinition gets executed.
# Responsible for adding commandInputs to the command &
# registering the other command handlers.
class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            # Get the command that was created.
            cmd = adsk.core.Command.cast(args.command)

            # Registers the CommandExecuteHandler
            onExecute = CommandExecuteHandler()
            cmd.execute.add(onExecute)
            _handlers.append(onExecute)

            # Registers the CommandExecutePreviewHandler
            onExecutePreview = CommandExecutePreviewHandler()
            cmd.executePreview.add(onExecutePreview)
            _handlers.append(onExecutePreview)

            # Registers the CommandInputChangedHandler
            onInputChanged = CommandInputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            _handlers.append(onInputChanged)

            # Registers the CommandDestryHandler
            onDestroy = CommandDestroyHandler()
            cmd.destroy.add(onDestroy)
            _handlers.append(onDestroy)

            # Registers the CommandValidateInputsEventHandler
            onValidate = CommandValidateInputsEventHandler()
            cmd.validateInputs.add(onValidate)
            _handlers.append(onValidate)

            # Get the CommandInputs collection associated with the command.
            inputs = cmd.commandInputs

            # Tabs
            tabSettings = inputs.addTabCommandInput("TabSettings", "Settings")
            tabAdvanced = inputs.addTabCommandInput("TabAdvanced", "Advanced")
            tabPosition = inputs.addTabCommandInput("TabPosition", "Position")
            tabProperties = inputs.addTabCommandInput("TabProperties", "Properties")

            # Setting command Inputs
            ddType = tabSettings.children.addDropDownCommandInput("DDType", "Type", 0)
            ddType.listItems.add("External Gear", pers['DDType'] == "External Gear", "resources/external")
            ddType.listItems.add("Internal Gear", pers['DDType'] == "Internal Gear", "resources/internal")
            ddType.listItems.add("Rack Gear", pers['DDType'] == "Rack Gear", "resources/rack")

            viModule = tabSettings.children.addValueInput("VIModule", "Module", "mm",
                                                          adsk.core.ValueInput.createByReal(pers['VIModule']))
            viModule.tooltip = "Module"
            viModule.tooltipDescription = "The module is the fundamental unit of size for a gear.\nMatching gears must have the same module."

            viHelixAngle = tabSettings.children.addValueInput("VIHelixAngle", "Helix Angle", "deg",
                                                              adsk.core.ValueInput.createByReal(pers['VIHelixAngle']))
            viHelixAngle.tooltip = "Helix Angle"
            viHelixAngle.tooltipDescription = "Angle of tooth twist.\n0 degrees produces a standard spur gear.\nHigh angles produce worm gears\nNegative angles produce left handed gears"
            viHelixAngle.toolClipFilename = 'resources/captions/HelixAngle.png'

            isTeeth = tabSettings.children.addIntegerSpinnerCommandInput("ISTeeth", "Teeth", 1, 99999, 1,
                                                                         pers['ISTeeth'])
            isTeeth.isVisible = pers['DDType'] != "Rack Gear"
            isTeeth.tooltip = "Number of Teeth"
            isTeeth.tooltipDescription = "The number of teeth a gear has.\nGears with higher helix angle can have less teeth.\nFor example mots worm gears have only one."

            viWidth = tabSettings.children.addValueInput("VIWidth", "Gear Width", "mm",
                                                         adsk.core.ValueInput.createByReal(pers['VIWidth']))
            viWidth.tooltip = "Gear Width"
            viWidth.tooltipDescription = "Represenets the width or thickness of a gear"

            viHeight = tabSettings.children.addValueInput("VIHeight", "Height", "mm",
                                                          adsk.core.ValueInput.createByReal(pers['VIHeight']))
            viHeight.tooltip = "Rack Height"
            viHeight.tooltipDescription = "Represents the distance from the bottom to the pitch diameter.\nDoes not include Addendum."
            viHeight.isVisible = pers['DDType'] == "Rack Gear"

            viLength = tabSettings.children.addValueInput("VILength", "Length", "mm",
                                                          adsk.core.ValueInput.createByReal(pers['VILength']))
            viLength.tooltip = "Rack Length"
            viLength.isVisible = pers['DDType'] == "Rack Gear"

            viDiameter = tabSettings.children.addValueInput("VIDiameter", "Outside Diameter", "mm",
                                                            adsk.core.ValueInput.createByReal(pers['VIDiameter']))
            viDiameter.tooltip = "Internal Gear Outside Diameter"
            viDiameter.isVisible = pers['DDType'] == "Internal Gear"

            bvHerringbone = tabSettings.children.addBoolValueInput("BVHerringbone", "Herringbone", True, "",
                                                                   pers['BVHerringbone'])
            bvHerringbone.toolClipFilename = 'resources/captions/Herringbone.png'
            bvHerringbone.tooltip = "Herringbone"
            bvHerringbone.tooltipDescription = "Generates gear as herringbone."

            bvPreview = tabSettings.children.addBoolValueInput("BVPreview", "Preview", True, "", pers['BVPreview'])
            bvPreview.tooltip = "Preview"
            bvPreview.tooltipDescription = "Generates a real-time preview of the gear.\nThis makes changes slower as the gear has to re-generate."

            tbWarning1 = tabSettings.children.addTextBoxCommandInput("TBWarning1", "", '', 2, True)

            # Advanced command inputs
            ddStandard = tabAdvanced.children.addDropDownCommandInput("DDStandard", "Standard", 0)
            ddStandard.listItems.add("Normal", pers['DDStandard'] == "Normal", "resources/normal")
            ddStandard.listItems.add("Radial", pers['DDStandard'] == "Radial", "resources/radial")
            ddStandard.toolClipFilename = 'resources/captions/NormalVsRadial.png'
            ddStandard.tooltipDescription = "Normal System: Pressure angle and module are defined relative to the normal of the tooth.\n\nRadial System: Pressure angle and module are defined relative to the plane of rotation."

            viPressureAngle = tabAdvanced.children.addValueInput("VIPressureAngle", "Pressure Angle", "deg",
                                                                 adsk.core.ValueInput.createByReal(
                                                                     pers['VIPressureAngle']))
            viPressureAngle.tooltip = "Pressure Angle"
            viPressureAngle.tooltipDescription = "Represent the angle of the line of contact.\nStandart values are: 20°, 14.5° "

            viBacklash = tabAdvanced.children.addValueInput("VIBacklash", "Backlash", "mm",
                                                            adsk.core.ValueInput.createByReal(pers['VIBacklash']))
            viBacklash.tooltip = "Backlash"
            viBacklash.tooltipDescription = "Represents the distance between two mating teeth at the correct spacing.\nThis value is halved as it should be distributed between both gears."

            viAddendum = tabAdvanced.children.addValueInput("VIAddendum", "Addendum", "",
                                                            adsk.core.ValueInput.createByReal(pers['VIAddendum']))
            viAddendum.tooltip = "Addendum"
            viAddendum.tooltipDescription = "Represents the factor that the tooth extends past the pitch diameter."

            viDedendum = tabAdvanced.children.addValueInput("VIDedendum", "Dedendum", "",
                                                            adsk.core.ValueInput.createByReal(pers['VIDedendum']))
            viDedendum.tooltip = "Dedendum"
            viDedendum.tooltipDescription = "Represents the factor that the root diameter is below the pitch diameter."

            tbWarning2 = tabAdvanced.children.addTextBoxCommandInput("TBWarning2", "", '', 2, True)

            # Position
            siOrigin = tabPosition.children.addSelectionInput("SIOrigin", "Center", "Select Gear Center")
            siOrigin.addSelectionFilter("ConstructionPoints")
            siOrigin.addSelectionFilter("SketchPoints")
            siOrigin.addSelectionFilter("Vertices")
            siOrigin.setSelectionLimits(0, 1)
            
            siPlane = tabPosition.children.addSelectionInput("SIPlane", "Plane", "Select Gear Plane")
            siPlane.addSelectionFilter("ConstructionPlanes")
            siPlane.addSelectionFilter("Profiles")
            siPlane.addSelectionFilter("Faces")
            #siPlane.addSelectionFilter("ConstructionLines")
            #siPlane.addSelectionFilter("SketchLines")
            #siPlane.addSelectionFilter("LinearEdges")
            siPlane.setSelectionLimits(0, 1)

            ddDirection = tabPosition.children.addDropDownCommandInput("DDDirection", "Direction", 0)
            ddDirection.listItems.add("Front", True, "resources/front")
            ddDirection.listItems.add("Center", False, "resources/center")
            ddDirection.listItems.add("Back", False, "resources/back")

            avRotation = tabPosition.children.addAngleValueCommandInput("AVRotation", "Rotation", adsk.core.ValueInput.createByReal(0))
            avRotation.isVisible = False

            dvOffset = tabPosition.children.addDistanceValueCommandInput("DVOffset", "Offset",adsk.core.ValueInput.createByReal(0))
            dvOffset.setManipulator(
                adsk.core.Point3D.create(0,0,0),
                adsk.core.Vector3D.create(0,0,1)
            )
            dvOffset.isVisible = False


            # Properties
            tbProperties = tabProperties.children.addTextBoxCommandInput("TBProperties", "", "", 5, True)

        except:
            print(traceback.format_exc())


# Fires when the User executes the Command
# Responsible for doing the changes to the document
class CommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            # Saves inputs to dict for persistence
            preserve_inputs(args.command.commandInputs, pers)

            gear = generate_gear(args.command.commandInputs).model_gear(adsk.core.Application.get().activeProduct.rootComponent)
                
            move_gear(gear, args.command.commandInputs)

        except:
            print(traceback.format_exc())


# Fires when the Command is being created or when Inputs are being changed
# Responsible for generating a preview of the output.
# Changes done here are temporary and will be cleaned up automatically.
class CommandExecutePreviewHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            if (args.command.commandInputs.itemById("BVPreview").value):
                preserve_inputs(args.command.commandInputs, pers)

                global lastInput

                reuse_gear = lastInput  in ["APITabBar", "SIPlane", "SIOrigin", "DDDirection", "AVRotation", "DVOffset"]
                
                gear = generate_gear(args.command.commandInputs).model_gear(adsk.core.Application.get().activeProduct.rootComponent, reuse_gear)
                
                move_gear(gear, args.command.commandInputs)
                
                args.isValidResult = True
            else:
                args.isValidResult = False

        except:
            print(traceback.format_exc())


# Fires when CommandInputs are changed or other parts of the UI are updated
# Responsible for turning the ok button on or off and allowing preview
class CommandValidateInputsEventHandler(adsk.core.ValidateInputsEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            is_invalid = generate_gear(args.inputs).is_invalid
            args.areInputsValid = not is_invalid
        except:
            print(traceback.format_exc())


# Fires when CommandInputs are changed
# Responsible for dynamically updating other Command Inputs
class CommandInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            global lastInput
            lastInput = args.input.id
            # Handles input visibillity based on gear type
            if (args.input.id == "DDType"):
                gearType = args.input.selectedItem.name
                args.inputs.itemById("ISTeeth").isVisible = gearType != "Rack Gear"
                args.inputs.itemById("VIHeight").isVisible = gearType == "Rack Gear"
                args.inputs.itemById("VILength").isVisible = gearType == "Rack Gear"
                args.inputs.itemById("VIDiameter").isVisible = gearType == "Internal Gear"
            # Updates Information
            if (args.inputs.itemById("TabProperties") and args.inputs.itemById("TabProperties").isActive):
                gear = generate_gear(args.inputs)
                tbProperties = args.inputs.itemById("TBProperties")
                tbProperties.numRows = len(str(gear).split('\n'))
                tbProperties.text = str(gear)
            # Updates Warning Message
            if (not args.input.id[:2] == "TB"):
                is_invalid = generate_gear(args.input.parentCommand.commandInputs).is_invalid
                if (is_invalid):
                    args.input.parentCommand.commandInputs.itemById(
                        "TBWarning1").formattedText = '<h3><font color="darkred">Error: {0}</font></h3>'.format(
                        is_invalid)
                    args.input.parentCommand.commandInputs.itemById(
                        "TBWarning2").formattedText = '<h3><font color="darkred">Error: {0}</font></h3>'.format(
                        is_invalid)
                else:
                    args.input.parentCommand.commandInputs.itemById("TBWarning1").formattedText = ''
                    args.input.parentCommand.commandInputs.itemById("TBWarning2").formattedText = ''
            # Hides Positioning Manipulators when inactive
            if(args.input.id == "APITabBar"):
                if (args.inputs.itemById("TabPosition") and args.inputs.itemById("TabPosition").isActive):
                    args.input.parentCommand.commandInputs.itemById("SIOrigin").isVisible = True
                    args.input.parentCommand.commandInputs.itemById("SIPlane").isVisible = True
                    args.input.parentCommand.commandInputs.itemById("DVOffset").isVisible = True
                    args.input.parentCommand.commandInputs.itemById("AVRotation").isVisible = True
                else:
                    args.input.parentCommand.commandInputs.itemById("SIOrigin").isVisible = False
                    args.input.parentCommand.commandInputs.itemById("SIPlane").isVisible = False
                    args.input.parentCommand.commandInputs.itemById("DVOffset").isVisible = False
                    args.input.parentCommand.commandInputs.itemById("AVRotation").isVisible = False
            # Update manipulators
            #if(args.input.id == "SIOrigin" or args.input.id == "SIPlane"):
                

        except:
            print(traceback.format_exc())


# Fires when the Command gets Destroyed regardless of success
# Responsible for cleaning up
class CommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            # TODO: Add Destroy stuff
            pass
        except:
            print(traceback.format_exc())


def preserve_inputs(commandInputs, pers):
    pers['DDType'] = commandInputs.itemById("DDType").selectedItem.name
    pers['DDStandard'] = commandInputs.itemById("DDStandard").selectedItem.name
    pers['VIHelixAngle'] = commandInputs.itemById("VIHelixAngle").value
    pers['VIPressureAngle'] = commandInputs.itemById("VIPressureAngle").value
    pers['VIModule'] = commandInputs.itemById("VIModule").value
    pers['ISTeeth'] = commandInputs.itemById("ISTeeth").value
    pers['VIBacklash'] = commandInputs.itemById("VIBacklash").value
    pers['VIWidth'] = commandInputs.itemById("VIWidth").value
    pers['VIHeight'] = commandInputs.itemById("VIHeight").value
    pers['VILength'] = commandInputs.itemById("VILength").value
    pers['VIDiameter'] = commandInputs.itemById("VIDiameter").value
    pers['BVHerringbone'] = commandInputs.itemById("BVHerringbone").value
    pers['VIAddendum'] = commandInputs.itemById("VIAddendum").value
    pers['VIDedendum'] = commandInputs.itemById("VIDedendum").value


def generate_gear(commandInputs):
    gearType = commandInputs.itemById("DDType").selectedItem.name
    standard = commandInputs.itemById("DDStandard").selectedItem.name

    if (gearType == "Rack Gear"):
        if (standard == "Normal"):
            gear = RackGear.create_in_normal_system(
                commandInputs.itemById("VIModule").value,
                commandInputs.itemById("VIPressureAngle").value,
                commandInputs.itemById("VIHelixAngle").value,
                commandInputs.itemById("BVHerringbone").value,
                commandInputs.itemById("VILength").value,
                commandInputs.itemById("VIWidth").value,
                commandInputs.itemById("VIHeight").value,
                commandInputs.itemById("VIBacklash").value,
                commandInputs.itemById("VIAddendum").value,
                commandInputs.itemById("VIDedendum").value
            )
        else:
            gear = RackGear.create_in_radial_system(
                commandInputs.itemById("VIModule").value,
                commandInputs.itemById("VIPressureAngle").value,
                commandInputs.itemById("VIHelixAngle").value,
                commandInputs.itemById("BVHerringbone").value,
                commandInputs.itemById("VILength").value,
                commandInputs.itemById("VIWidth").value,
                commandInputs.itemById("VIHeight").value,
                commandInputs.itemById("VIBacklash").value,
                commandInputs.itemById("VIAddendum").value,
                commandInputs.itemById("VIDedendum").value
            )
    else:
        if (gearType == "External Gear"):
            if (standard == "Normal"):
                gear = HelicalGear.create_in_normal_system(
                    commandInputs.itemById("ISTeeth").value,
                    commandInputs.itemById("VIModule").value,
                    commandInputs.itemById("VIPressureAngle").value,
                    commandInputs.itemById("VIHelixAngle").value,
                    commandInputs.itemById("VIBacklash").value,
                    commandInputs.itemById("VIAddendum").value,
                    commandInputs.itemById("VIDedendum").value,
                    commandInputs.itemById("VIWidth").value,
                    commandInputs.itemById("BVHerringbone").value
                )
            else:
                gear = HelicalGear.create_in_radial_system(
                    commandInputs.itemById("ISTeeth").value,
                    commandInputs.itemById("VIModule").value,
                    commandInputs.itemById("VIPressureAngle").value,
                    commandInputs.itemById("VIHelixAngle").value,
                    commandInputs.itemById("VIBacklash").value,
                    commandInputs.itemById("VIAddendum").value,
                    commandInputs.itemById("VIDedendum").value,
                    commandInputs.itemById("VIWidth").value,
                    commandInputs.itemById("BVHerringbone").value
                )
        else:
            if (standard == "Normal"):
                gear = HelicalGear.create_in_normal_system(
                    commandInputs.itemById("ISTeeth").value,
                    commandInputs.itemById("VIModule").value,
                    commandInputs.itemById("VIPressureAngle").value,
                    commandInputs.itemById("VIHelixAngle").value,
                    -commandInputs.itemById("VIBacklash").value,
                    commandInputs.itemById("VIDedendum").value,
                    commandInputs.itemById("VIAddendum").value,
                    commandInputs.itemById("VIWidth").value,
                    commandInputs.itemById("BVHerringbone").value,
                    commandInputs.itemById("VIDiameter").value
                )
            else:
                gear = HelicalGear.create_in_radial_system(
                    commandInputs.itemById("ISTeeth").value,
                    commandInputs.itemById("VIModule").value,
                    commandInputs.itemById("VIPressureAngle").value,
                    commandInputs.itemById("VIHelixAngle").value,
                    -commandInputs.itemById("VIBacklash").value,
                    commandInputs.itemById("VIDedendum").value,
                    commandInputs.itemById("VIAddendum").value,
                    commandInputs.itemById("VIWidth").value,
                    commandInputs.itemById("BVHerringbone").value,
                    commandInputs.itemById("VIDiameter").value
                )
    return gear


def move_gear(gear, commandInputs):

    side_offset = (0.5 - (commandInputs.itemById("DDDirection").selectedItem.index * 0.5)) * commandInputs.itemById("VIWidth").value


    if(commandInputs.itemById("SIOrigin").selectionCount):
        point = commandInputs.itemById("SIOrigin").selection(0).entity
        pointPrim = get_primitive_from_selection(point)

        # Both Plane and Origin selected, regular move
        if(commandInputs.itemById("SIPlane").selectionCount):
            plane = commandInputs.itemById("SIPlane").selection(0).entity
            planePrim = get_primitive_from_selection(plane)

        # Just sketch point selected, use sketch plane as plane
        elif(point.objectType == "adsk::fusion::SketchPoint"):
            planePrim = adsk.core.Plane.createUsingDirections(
                point.parentSketch.origin,
                point.parentSketch.xDirection,
                point.parentSketch.yDirection
            )

        # No useable plane selected
        else:
            planePrim = adsk.core.Plane.createUsingDirections(
                pointPrim,
                adsk.core.Vector3D.create(1,0,0),
                adsk.core.Vector3D.create(0,1,0)
            )
            
        gear.transform = move_matrix(
            project_point_on_plane(pointPrim, planePrim),
            planePrim.normal,
            commandInputs.itemById("AVRotation").value,
            commandInputs.itemById("DVOffset").value + side_offset
        )
    else:
        # No valid selection combination, no move just side & rotation
        gear.transform = move_matrix(
            adsk.core.Point3D.create(0,0,0),
            adsk.core.Vector3D.create(0,0,1),
            commandInputs.itemById("AVRotation").value,
            commandInputs.itemById("DVOffset").value + side_offset
        )


def move_matrix(position, direction, rotation, offset):
    mat = adsk.core.Matrix3D.create()

    p = adsk.core.Plane.create(position, direction)

    mat.setToAlignCoordinateSystems(
        adsk.core.Point3D.create(0, 0, -offset),
        adsk.core.Vector3D.create(math.cos(-rotation), math.sin(-rotation), 0),
        adsk.core.Vector3D.create(-math.sin(-rotation), math.cos(-rotation), 0),
        adsk.core.Vector3D.create(0, 0, 1),
        position,
        p.uDirection,
        p.vDirection,
        direction
    )

    return mat


def get_primitive_from_selection(selection):
    # Construction Plane
    if selection.objectType == "adsk::fusion::ConstructionPlane":
        # TODO: Coordinate in assembly context, world transform still required!
        return selection.geometry
    # Sketch Profile
    if selection.objectType == "adsk::fusion::Profile":
        return adsk.core.Plane.createUsingDirections(
            selection.parentSketch.origin,
            selection.parentSketch.xDirection,
            selection.parentSketch.yDirection
        )
    # BRepFace
    if selection.objectType == "adsk::fusion::BRepFace":
        _, normal = selection.evaluator.getNormalAtPoint(selection.pointOnFace)
        return adsk.core.Plane.create(
            selection.pointOnFace,
            normal
        )
    # Construction Axis
    if selection.objectType == "adsk::fusion::ConstructionAxis":
        # TODO: Coordinate in assembly context, world transform still required!
        return selection.geometry
    #TODO: BRep Edge, Sketch line
    
    # Construction Point
    if selection.objectType == "adsk::fusion::ConstructionPoint":
        # TODO: Coordinate in assembly context, world transform still required!
        return selection.geometry
    # Sketch Point
    if selection.objectType == "adsk::fusion::SketchPoint":
        return selection.worldGeometry
    # BRepVertex
    if selection.objectType == "adsk::fusion::BRepVertex":
        return selection.geometry

    
def project_point_on_plane(point, plane):
    origin_to_point = plane.origin.vectorTo(point)

    normal = plane.normal.copy()
    normal.normalize()
    dist_pt_to_pln = normal.dotProduct(origin_to_point)

    normal.scaleBy(-dist_pt_to_pln)

    pt_on_pln = point.copy()
    pt_on_pln.translateBy(normal)

    return pt_on_pln
    

def run(context):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        commandDefinitions = ui.commandDefinitions
        # check the command exists or not
        cmdDef = commandDefinitions.itemById(COMMAND_ID)
        if not cmdDef:
            cmdDef = commandDefinitions.addButtonDefinition(COMMAND_ID, COMMAND_NAME,
                                                            COMMAND_TOOLTIP, 'resources')
            cmdDef.tooltip = "Generates external, inrernal & rack gears of any helix angle.\nThis includes regular sput gears as well as worm gears."
            cmdDef.toolClipFilename = 'resources/captions/Gears.png'
        # Adds the commandDefinition to the toolbar
        for panel in TOOLBAR_PANELS:
            ui.allToolbarPanels.itemById(panel).controls.addCommand(cmdDef)

        onCommandCreated = CommandCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        _handlers.append(onCommandCreated)
    except:
        print(traceback.format_exc())


def stop(context):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # Removes the commandDefinition from the toolbar
        for panel in TOOLBAR_PANELS:
            p = ui.allToolbarPanels.itemById(panel).controls.itemById(COMMAND_ID)
            if p:
                p.deleteMe()

        # Deletes the commandDefinition
        ui.commandDefinitions.itemById(COMMAND_ID).deleteMe()
    except:
        print(traceback.format_exc())

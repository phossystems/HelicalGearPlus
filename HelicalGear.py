# Original Author-Ross Korsky 2016-2017
# Modified by Nico Schlueter
#
# Released under the MIT license. See License.txt for full license information.
#
# Description-Helical or "dry fixed" gears offer a refinement over spur gears. The leading edges of the teeth are not
# parallel to the axis of rotation, but are set at an angle. Since the gear is curved, this angling makes the tooth
# shape a segment of a helix. Helical gears can be meshed in parallel or crossed orientations.[From Wikipedia]
#
# Parts (mostly some of the Involute code) was taken from AutoDesks' Fusion 360 SpurGear sample script.
# The primary source used to produce this add-in was http://qtcgears.com/tools/catalogs/PDF_Q420/Tech.pdf

import math
import os

import adsk.cam
import adsk.core
import adsk.fusion

from .Backports.enum import Enum
from .Fission import fission


class GearStandards(Enum):
    normal_system = 1
    radial_system = 2
    sunderland = 3


class Handedness(Enum):
    left = -1
    right = 1


class Involute(object):
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
        (_, _, cross_points) = spline1.intersections(fission.ObjectCollectionFromList([spline2]))
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
                    fission.Point3D(math.cos(tip_curve1Angle) * tip_rad,
                                    math.sin(tip_curve1Angle) * tip_rad,
                                    z_shift),
                    tip_curve2Angle - tip_curve1Angle)
                key_points.append(tip_arc.startSketchPoint.geometry)
                key_points.append(fission.Point3D(tip_rad, 0, z_shift))
                key_points.append(tip_arc.endSketchPoint.geometry)

        # Draw root circle
        # root_circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(origin_point, self.gear.root_diameter/2)
        root_arc = sketch.sketchCurves.sketchArcs.addByCenterStartSweep(
            origin_point,
            fission.Point3D(math.cos(curve1Angle) * (self.gear.root_diameter / 2 - 0.01),
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


class HelicalGear(object):
    def __init__(self):
        pass

    @property
    def helix_angle(self):
        return self.__helix_angle

    @property
    def handedness(self):
        return self.__handedness

    @property
    def tooth_count(self):
        return self.__tooth_count

    @property
    def normal_module(self):
        return self.__normal_module

    @property
    def normal_pressure_angle(self):
        return self.__normal_pressure_angle

    @property
    def normal_circular_pitch(self):
        return self.__normal_circular_pitch

    @property
    def virtual_teeth(self):
        return self.__virtual_teeth

    @property
    def module(self):
        return self.__module

    @property
    def pressure_angle(self):
        return self.__pressure_angle

    @property
    def pitch_diameter(self):
        return self.__pitch_diameter

    @property
    def base_diameter(self):
        return self.__base_diameter

    @property
    def addendum(self):
        return self.__addendum

    @property
    def whole_depth(self):
        return self.__whole_depth

    @property
    def outside_diameter(self):
        return self.__outside_diameter

    @property
    def root_diameter(self):
        return self.__root_diameter

    @property
    def circular_pitch(self):
        return self.__circular_pitch

    @property
    def is_undercut_requried(self):
        return self.virtual_teeth < self.critcal_virtual_tooth_count

    @property
    def backlash(self):
        """The backlash is split between both sides of this and (an assumed) mateing gear - each side of a tooth will be narrowed by 1/4 this value."""
        return self.__backlash

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
    def is_valid(self):
        valid = self.circular_pitch > 0
        valid &= self.base_diameter > 0
        valid &= self.pitch_diameter > 0
        valid &= self.root_diameter > 0.03
        valid &= self.outside_diameter > 0
        valid &= self.whole_depth > 0
        valid &= self.addendum > 0
        valid &= self.pressure_angle >= 0
        valid &= self.pressure_angle < math.radians(90)
        valid &= self.module > 0
        valid &= self.tooth_count > 0
        valid &= self.helix_angle >= 0
        valid &= self.helix_angle < math.radians(90)
        valid &= abs(self.backlash_angle) / 4 < self.tooth_arc_angle / 8
        return valid

    @property
    def pitch_helix(self):
        return HelixCurve(
            self.pitch_diameter / 2,
            math.radians(90) - self.helix_angle if self.handedness is Handedness.right else math.radians(
                90) + self.helix_angle)

    def __str__(self):
        str = ''
        if self.is_undercut_requried:
            str += 'UNDERCUT REQUIRED\n'
            str += '-Critical Virtual Tooth Count: {0:.3f}\n'.format(self.critcal_virtual_tooth_count)
            str += '-Current Virtual Tooth Count.: {0:.3f}\n'.format(self.__virtual_teeth)
            str += '\n'

        str += 'helix angle..........: {0:.3f} deg\n'.format(math.degrees(self.__helix_angle))
        str += 'handedness...........: {0}\n'.format(self.__handedness)
        str += 'length per revolution: {0:.9f} mm\n'.format(
            abs(self.pitch_helix.vertical_loop_seperation) * 10 if self.__helix_angle != 0 else float('inf'))
        str += '\n'
        str += 'tooth count..........: {0}\n'.format(self.__tooth_count)
        str += 'module...............: {0:.3f} mm\n'.format(self.__module * 10)
        str += 'pressure angle.......: {0:.3f} deg\n'.format(math.degrees(self.__pressure_angle))
        str += 'tip pressure angle...: {0:.3f} deg\n'.format(math.degrees(self.tip_pressure_angle))
        str += 'circular pitch.......: {0:.3f} mm\n'.format(self.__circular_pitch * 10)
        str += 'addendum.............: {0:.3f} mm\n'.format(self.__addendum * 10)
        str += 'whole depth..........: {0:.3f} mm\n'.format(self.__whole_depth * 10)
        str += 'backlash.............: {0:.3f} mm\n'.format(self.__backlash * 10)
        str += '\n'
        str += 'virtual teeth........: {0:.3f}\n'.format(self.__virtual_teeth)
        str += 'normal module........: {0:.3f} mm\n'.format(self.__normal_module * 10)
        str += 'normal pressure angle: {0:.3f} deg\n'.format(math.degrees(self.__normal_pressure_angle))
        str += 'normal circular pitch: {0:.3f} mm\n'.format(self.__normal_circular_pitch * 10)
        str += '\n'
        str += 'root diameter........: {0:.3f} mm\n'.format(self.__root_diameter * 10)
        str += 'base diameter........: {0:.3f} mm\n'.format(self.__base_diameter * 10)
        str += 'pitch diameter.......: {0:.3f} mm\n'.format(self.__pitch_diameter * 10)
        str += 'outside diameter.....: {0:.3f} mm\n'.format(self.__outside_diameter * 10)
        str += '\n'
        str += 'tooth arc angle......: {0:.3f} deg\n'.format(math.degrees(self.tooth_arc_angle))
        str += 'profile shift coef...: {0:.6f}\n'.format(self.profile_shift_coefficient)
        str += 'involute a...........: {0:.6f} rad\n'.format(self.involute_a)
        str += 'involute aa..........: {0:.6f} rad\n'.format(self.involute_aa)
        #    str += 'top land angle.......: {0:.3f} deg\n'.format(math.degrees(self.top_land_angle))
        #    str += 'top land thickness...: {0:.3f} mm\n'.format(self.top_land_thickness * 10)
        return str

    @staticmethod
    def create_in_normal_system(tooth_count, normal_module, normal_pressure_angle, helix_angle, handedness, backlash=0):
        tooth_count = tooth_count if tooth_count > 0 else 1
        normal_module = normal_module if normal_module > 0 else 1e-10
        normal_pressure_angle = normal_pressure_angle if 0 <= normal_pressure_angle < math.radians(90) else 0
        helix_angle = helix_angle if 0 <= helix_angle < math.radians(90) else 0

        gear = HelicalGear()
        gear.__backlash = backlash
        gear.__helix_angle = helix_angle
        gear.__handedness = handedness
        gear.__tooth_count = tooth_count

        gear.__normal_module = normal_module
        gear.__normal_pressure_angle = normal_pressure_angle

        gear.__normal_circular_pitch = gear.__normal_module * math.pi
        cos_helix_angle = math.cos(helix_angle)
        gear.__virtual_teeth = gear.tooth_count / math.pow(cos_helix_angle, 3)

        # Radial / Transverse figures
        gear.__module = gear.__normal_module / cos_helix_angle
        gear.__pressure_angle = math.atan2(math.tan(gear.normal_pressure_angle), cos_helix_angle)
        gear.__pitch_diameter = gear.__module * gear.__tooth_count
        gear.__base_diameter = gear.__pitch_diameter * math.cos(gear.__pressure_angle)
        gear.__addendum = gear.__normal_module
        gear.__whole_depth = 2.25 * gear.__normal_module
        gear.__outside_diameter = gear.__pitch_diameter + 2 * gear.__addendum
        gear.__root_diameter = gear.__outside_diameter - 2 * gear.__whole_depth
        gear.__circular_pitch = gear.__module * math.pi

        return gear

    @staticmethod
    def create_in_radial_system(tooth_count, radial_module, radial_pressure_angle, helix_angle, handedness, backlash=0):
        tooth_count = tooth_count if tooth_count > 0 else 1
        radial_module = radial_module if radial_module > 0 else 1e-10
        radial_pressure_angle = radial_pressure_angle if 0 <= radial_pressure_angle < math.radians(90) else 0
        helix_angle = helix_angle if 0 <= helix_angle < math.radians(90) else 0

        gear = HelicalGear()
        gear.__backlash = backlash
        gear.__helix_angle = helix_angle
        gear.__handedness = handedness
        gear.__tooth_count = tooth_count

        gear.__normal_module = radial_module * math.cos(gear.__helix_angle)
        gear.__normal_pressure_angle = math.atan(math.tan(radial_pressure_angle) * math.cos(gear.__helix_angle))
        gear.__normal_circular_pitch = gear.__normal_module * math.pi

        cos_helix_angle = math.cos(helix_angle)
        gear.__virtual_teeth = gear.tooth_count / math.pow(cos_helix_angle, 3)

        # Radial / Transverse figures
        gear.__module = radial_module
        gear.__pressure_angle = radial_pressure_angle
        gear.__pitch_diameter = gear.__module * gear.__tooth_count
        gear.__base_diameter = gear.__pitch_diameter * math.cos(gear.__pressure_angle)
        gear.__addendum = gear.__module
        gear.__whole_depth = 2.25 * gear.__module
        gear.__outside_diameter = gear.__pitch_diameter + 2 * gear.__addendum
        gear.__root_diameter = gear.__outside_diameter - 2 * gear.__whole_depth
        gear.__circular_pitch = gear.__module * math.pi

        return gear

    @staticmethod
    def create_sunderland(tooth_count, radial_module, handedness, backlash=0):
        """The Sunderland machine is commonly used to make double helical, or herringbone, gears.

        The (radial) pressure angle is fixed at 20 degrees and the helix angle is fixed at 22.5 degrees and the tooth
        has a slightly lower profile."""
        tooth_count = tooth_count if tooth_count > 0 else 1
        radial_module = radial_module if radial_module > 0 else 1e-10

        gear = HelicalGear()
        gear.__backlash = backlash
        helix_angle = math.radians(22.5)
        radial_pressure_angle = math.radians(20)
        gear.__helix_angle = helix_angle
        gear.__handedness = handedness
        gear.__tooth_count = tooth_count

        gear.__normal_module = radial_module * math.cos(gear.__helix_angle)
        gear.__normal_pressure_angle = math.atan(math.tan(radial_pressure_angle) * math.cos(gear.__helix_angle))
        gear.__normal_circular_pitch = gear.__normal_module * math.pi

        cos_helix_angle = math.cos(helix_angle)
        gear.__virtual_teeth = gear.tooth_count / math.pow(cos_helix_angle, 3)

        # Radial / Transverse figures
        gear.__module = radial_module
        gear.__pressure_angle = radial_pressure_angle
        gear.__pitch_diameter = gear.__module * gear.__tooth_count
        gear.__base_diameter = gear.__pitch_diameter * math.cos(gear.__pressure_angle)
        gear.__addendum = 0.8796 * gear.__module
        gear.__whole_depth = 1.8849 * gear.__module
        gear.__outside_diameter = gear.__pitch_diameter + 2 * gear.__addendum
        gear.__root_diameter = gear.__outside_diameter - 2 * gear.__whole_depth
        gear.__circular_pitch = gear.__module * math.pi

        return gear


class HelixCurve(object):
    def __init__(self, radius, helix_angle, rotation=0):
        self.__radius = radius
        self.__helix_angle = helix_angle
        self.__rotation = rotation
        self.__cos_rotation = math.cos(rotation)
        self.__sin_rotation = math.sin(rotation)
        # self.__is_valid = (
        #  (self.__helix_angle >= math.radians(-89.9999) and self.__helix_angle <= math.radians(-0.0001))
        #  or (self.__helix_angle <= math.radians(89.9999) and self.__helix_angle >= math.radians(0.0001)))
        self.__is_valid = (
                self.__helix_angle <= math.radians(-0.0001) or
                self.__helix_angle >= math.radians(0.0001))

        self.__c = math.tan(self.__helix_angle) * self.__radius if self.__is_valid else None

    @property
    def rotation(self):
        return self.__rotation

    @rotation.setter
    def rotation(self, value):
        self.__rotation = value
        self.__cos_rotation = math.cos(value)
        self.__sin_rotation = math.sin(value)

    @property
    def is_valid(self):
        return self.__is_valid

    @property
    def vertical_loop_seperation(self):
        return self.__c * 2 * math.pi if self.__is_valid else None

    @property
    def helix_angle(self):
        return self.__helix_angle if self.__is_valid else None

    @property
    def radius(self):
        return self.__radius if self.__is_valid else None

    @property
    def curvature(self):
        r = self.__radius
        c = self.__c
        return r / (r * r + c * c) if self.__is_valid else None

    @property
    def torsion(self):
        r = self.__radius
        c = self.__c
        return c / (r * r + c * c) if self.__is_valid else None

    def t_for(self, displacement):
        return displacement / self.__c if self.__is_valid else None

    def angle_at_displacement(self, displacement):
        """Angle between 0 and 2PI."""
        t = self.t_for(displacement)
        a = t + self.rotation
        a %= 2 * math.pi
        return a

    def locus(self, t):
        r = self.__radius
        c = self.__c
        return math.sqrt(r * r + c * c) * t if self.__is_valid else None

    def get_point(self, t):
        """Gets a 3D point along the Helical path.

        Args:
          self - self
          t - number - one revolution of the helix spans t from 0 to 2PI.
        Returns:
          adsk.core.Point3D
        """
        x = self.__radius * math.cos(t)
        y = self.__radius * math.sin(t)
        z = self.__c * t
        if self.__rotation != 0:
            xr = x * self.__cos_rotation - y * self.__sin_rotation
            y = x * self.__sin_rotation + y * self.__cos_rotation
            x = xr
        return fission.Point3D(x, y, z)

    def get_points(self, from_t, to_t, steps=None, bookends=0):
        points = []
        t_range = to_t - from_t
        if not steps:
            steps = int(3 * t_range / math.pi * 2)
        if steps < 0:
            steps *= -1
        if steps < 3:
            steps = 3
        step = 1.0 / (steps - 1)

        for i in range(-bookends, steps + bookends):
            t = from_t + t_range * step * i
            points.append(self.get_point(t))
        return points

    def offset(self, distance):
        return self.project(self.radius + distance)

    def project(self, new_radius):
        r2 = new_radius
        assert r2 != 0
        theta = math.atan(self.__c / abs(r2))
        if r2 < 0:
            theta = -theta
        return HelixCurve(abs(r2), theta)


class HelicalGearAddin(fission.CommandBase):
    """Helical or "dry fixed" gears offer a refinement over spur gears.

    The leading edges of the teeth are not parallel to the axis of rotation, but are set at an angle.
    Since the gear is curved, this angling makes the tooth shape a segment of a helix.

    Helical gears can be meshed in parallel or crossed orientations.
    [From Wikipedia]
    """

    def __init__(self):
        super().__init__()
        self.design = fission.DesignUtils()
        self.last_gear_stat_text = ''

        self.pers = {
            'Pressure Angle': 0.3490658503988659,
            'Backlash': 0.0,
            'Gear Thickness': 1.0,
            'Teeth': 16,
            'Gear Standard': 'Normal System',
            'Handedness': 'Right',
            'Helix Angle': 0.5235987755982988,
            'Module': 0.3,
            'Base Feature': False,
            'Herringbone': False}  # NSC C2 Initial persistence Dict

    @property
    def is_repeatable(self):
        return True

    @property
    def command_name(self):
        return 'Helical Gear'

    def add_button(self):
        self.remove_button()
        button = super().add_button()
        create_panel = self.ui.allToolbarPanels.itemById('SolidCreatePanel')
        create_panel.controls.addCommand(button)
        button.isPromotedByDefault = True
        button.isPromoted = True
        return button

    def remove_button(self):
        button = self.ui.commandDefinitions.itemById(self.command_id)
        create_panel = self.ui.allToolbarPanels.itemById('SolidCreatePanel')
        button_control = create_panel.controls.itemById(self.command_id)
        if button:
            button.deleteMe()
        if button_control:
            button_control.deleteMe()

    def initialize_inputs(self, factory):
        self.gear_standard = factory.create_text_drop_down(
            'gear_standard',
            'Gear Standard',
            items=['Normal System', 'Radial System', 'Sunderland'],
            values=[GearStandards.normal_system, GearStandards.radial_system, GearStandards.sunderland],
            default=self.pers['Gear Standard'], persist=False,  # NSC C2
            on_change=self.gear_standard_changed,
            help_image='resources/captions/NormalVsRadial.png',
            description="""The true involute pitch and involute geometry of a helical gear is in the plane of rotation (Radial System). However, because of the nature of tooth generation with a rack-type hob, a single tool can generate helical gears at all helix angles as well as standard spur gears. However, this means the normal pitch is the common denominator, and usually is taken as a standard value (e.g. 14.5 deg or most commonly 20 deg). In other words if you plan to have your gear manufactured with a standard hob you will likely want to use the "Normal System" and a pressure Angle of 20 degrees.

Normal System: Pressure angle and module are defined relative to the normal of the tooth (i.e. defined as if the tooth was rotated by the helix angle). When defining a gear in the normal system changes to the Helix Angle will cause the gears diameter to change as well as the working thickness (and therefore the strength) of the tooth.

Radial System: Pressure angle and module are defined relative to the plane of rotation. When defining a gear in the radial system changes to the Helix Angle does NOT affect the gear diameter but it does change the "normal pressure angle" which may require custom tooling to have the gear manufactured (obviously this is not an issue if 3D printing the part).

Sunderland: The Sunderland machine is commonly used to make a double helical gear, or herringbone, gear. The radial pressure angle and helix angle are fixed at 20° and 22.5°, respectively.  The tooth profile of Sunderland gears is also slightly shorter (and therefore stronger) than equivalent gears defined in the radial system.""")
        self.handedness = factory.create_text_drop_down(
            'handedness',
            'Handedness',
            items=['Left', 'Right'],
            values=[Handedness.left, Handedness.right],
            default=self.pers['Handedness'], persist=False,  # NSC C2
            help_image='resources/captions/Handedness.png',
            description='Direction the tooth appears to lean when placed flat on a table. Helical gears of opposite hand operate on parallel shafts. Helical gears of the same hand operate at right angles.')
        # Both gears of a meshed pair must have the same helix angle and pressure able. \nHowever, the handedness (helix direction) must be opposite.
        self.helix_angle = factory.addValueInput(
            'helix_angle',
            'Helix Angle',
            self.pers['Helix Angle'], 'deg', persist=False,  # NSC C2
            on_validate=lambda i: self.gear_standard.eval() == GearStandards.sunderland or (
                    0 <= i.eval() < math.radians(88.0001)),
            help_image='resources/captions/HelixAngle.png',
            description='Angle of tooth twist. 0 degrees produces a standard spur gear.\nThe higher the helix angle the more twist the gear has.')
        self.pressure_angle = factory.addValueInput(
            'pressure_angle',
            'Pressure Angle',
            self.pers['Pressure Angle'], 'deg', persist=False,  # NSC C2
            on_validate=lambda i: self.gear_standard.eval() == GearStandards.sunderland or (
                    0 <= i.eval() <= math.radians(70.0001)),
            description='The most common value for pressure angle is 20°, the second most common is 14.5°. The pressure angle defines the angle of the line of action which is a common tangent between the two base circles of a pair of gears. The short of it is this: leave this value at 20 degrees until you have reason to do otherwise - but know that any pair of gears MUST have the same pressure angle.')
        self.module = factory.addValueInput(
            'module',
            'Module',
            self.pers['Module'], 'mm', persist=False,  # NSC C2
            on_validate=lambda i: i.eval() > 0,
            description='The module is the length of pitch diameter per tooth. Therefore m = d / z; where m is module, d is the pitch diameter of the gear, and z is the number of teeth.')

        self.tooth_count = factory.create_int_spinner(
            'tooth_count',
            'Teeth',
            self.pers['Teeth'],  # NSC C2
            min=1,
            max=10000, persist=False,  # NSC C2
            description='Number of teeth the gear has. The higher the helix angle, and to some extent pressure angle, are the fewer teeth the gear needs to have to avoid undercutting. It is possible to create a Helical gear with a single tooth given a high enough Helix Angle.\n\nCAUTION: due to performance reasons, do not make gears with several hundred teeth.')
        self.backlash = factory.addValueInput(
            'backlash',
            'Backlash',
            str(self.pers['Backlash']), 'mm', persist=False,
            description='[experimental] a positive value here causes each tooth to be slightly narrower than the ideal tooth. In the real world having a perfect tooth is not often desired, it is better to build in a little backlash to reduce friction, allow room for lubricant between teeth, and to prevent jamming.\n\nBacklash is allowed to also be negative which has no real world application I\'m aware of but may be useful for compensating for undersized teeth which were 3D printed, etc.\n\nThe backlash value is split between this gear and its theoretical mate.')
        self.gear_thickness = factory.addValueInput(
            'gear_thickness',
            'Gear Thickness',
            self.pers['Gear Thickness'], 'mm', persist=False,
            on_validate=lambda i: i.eval() > 0,
            description='How thick you want the gear to be. CAUTION: making a gear really thick can cause some serious performance issues. If you wish to make a gear where the teeth wrap around multiple times it is recommend to see the "length per revolution" field in the "Gear Parameters" readout and use that value for your gear thickness then copy/rectangular pattern the gear body to reach your desired length. This is something which may be addressed in a future release.')
        self.herringbone = factory.create_checkbox(
            'herringbone',
            'Herringbone',
            self.pers["Herringbone"],
            tooltip='Generate as herringbone gear when checked.',
            persist=False)
        self.full_preview = factory.create_checkbox(
            'full_preview',
            'Preview',
            False,
            tooltip='Generate a full preview when checked.',
            persist=False)
        self.base_feature = factory.create_checkbox(
            'base_feature',
            'Base Feature',
            self.pers['Base Feature'],
            tooltip='Generates as a base feature when checked. Slightly better performance when recomputing.',
            persist=False)

        self.error_message = factory.create_textbox('error_message', read_only=True, persist=False)
        self.error_message.isFullWidth = True
        self.set_error_message('')
        self.warn_message = factory.create_textbox('warn_message', read_only=True, persist=False)
        self.warn_message.isFullWidth = True
        self.set_warn_message('')

        factory.begin_group(
            'gear_parameters_group',
            'Gear Parameters',
            expanded=False)
        self.gear_stats = factory.create_textbox('gear_stats', read_only=True, persist=False)
        factory.close_group()

        # factory.additional_validator = self.on_validate
        # Trigger any change events we have setup - the values may have been restored from saved state
        self.gear_standard_changed(self.gear_standard)
        self.update_warnings()

    def __on_input_changed(self, _a) -> 'input_changed':
        args = adsk.core.InputChangedEventArgs.cast(_a)
        if args.input in (self.helix_angle,
                          self.pressure_angle,
                          self.module,
                          self.tooth_count,
                          self.gear_standard,
                          self.backlash,
                          self.gear_thickness):
            self.update_warnings()

    def update_warnings(self):
        gear = self.make_gear()
        warn_msg = 'Undercut Required - increase the number of teeth, helix angle, or pressure angle to avoid undercutting.' if gear.is_undercut_requried else ''
        warn_lines = 4
        err_lines = 4
        bad_fields = []
        loops = self.gear_thickness.eval() / gear.pitch_helix.vertical_loop_seperation
        if loops > 50:
            if warn_msg: warn_msg += '<br/>'
            warn_msg += 'Creating a gear where teeth wrap around more than 50 times may lead to Fusion becoming unresponsive while generating. Your settings will result in {0:.3f} wraps. Don\'t worry, it will generate eventually. '.format(
                loops)
            warn_lines += 6
        elif loops > 10:
            if warn_msg: warn_msg += '<br/>'
            warn_msg += ('Generating a gear where the teeth wrap around many times is performance intensive. '
                         + 'Your settings will result in {0:.3f} wraps. '
                         + 'Previewing is prevented above 25 wraps but clicking OK will generate the gear.').format(
                loops)
            warn_lines += 6
        if gear.tooth_count > 300:
            bad_fields.append((self.tooth_count.name,
                               '[WARN] creating a gear with over 300 teeth may lead to Fusion becoming unresponsive.'))
            err_lines += 2
        elif gear.tooth_count > 100:
            if warn_msg: warn_msg += '<br/>'
            warn_msg += 'Generating a gear with a high number of teeth has poor performance. Previewing will be limited to 150 teeth but clicking OK will generate the gear.'
            warn_lines += 4
        self.set_warn_message(warn_msg, warn_lines)

        if not self.helix_angle.validate():
            bad_fields.append((self.helix_angle.name, 'must be between 0 and 88 deg.'))
            err_lines += 1
        if not self.pressure_angle.validate():
            bad_fields.append((self.pressure_angle.name, 'must be between 0 and 70 deg.'))
            err_lines += 1
        if not self.module.validate():
            bad_fields.append((self.module.name, 'must be greater than 0.'))
            err_lines += 1
        if not (abs(gear.backlash_angle) / 4 < gear.tooth_arc_angle / 8):
            bad_fields.append((self.backlash.name,
                               'must be in the range +/- {0:.4f} mm.'.format(10 * gear.circular_pitch / 2 - 0.00005)))
            err_lines += 1

        if not bad_fields and not gear.is_valid:
            bad_fields.append(('Gear Generation Failed', 'try increasing the helix angle or the number of teeth.'))
            err_lines += 2
        if bad_fields:
            msg = '<b>Invalid Inputs:</b><br/>'
            msg += ''.join('<em>{0}</em> {1}<br/>'.format(i[0], i[1]) for i in bad_fields)
            msg += ''
            self.set_error_message(msg, err_lines)
        else:
            self.set_error_message('')

    # for some reason including this code in validation in any way leads to a crazy validate -> update -> validate loop
    def on_validate(self, _a):  # -> 'validate_inputs':
        args = adsk.core.ValidateInputsEventArgs.cast(_a)
        gear = self.make_gear()
        args.areInputsValid &= gear.is_valid

    def set_error_message(self, message, count_lines=0):
        formatted_msg = '<font color="darkred">' + message + '</font>'
        if formatted_msg != self.error_message.formattedText:
            self.error_message.formattedText = formatted_msg
            self.error_message.numRows = count_lines if count_lines > 0 else 3
            self.error_message.isVisible = True if message else False

    def set_warn_message(self, message, count_lines=0):
        formatted_msg = '<font color="#ae6300">' + message + '</font>'
        if formatted_msg != self.warn_message.formattedText:
            self.warn_message.formattedText = formatted_msg
            self.warn_message.numRows = count_lines if count_lines > 0 else 3
            self.warn_message.isVisible = True if message else False

    def gear_standard_changed(self, sender):
        choice = sender.eval()
        if choice == GearStandards.sunderland:
            # self.pressure_angle.value = math.radians(20)
            # self.helix_angle.value = math.radians(22.5)
            self.pressure_angle.isEnabled = False
            self.helix_angle.isEnabled = False
        else:
            self.pressure_angle.isEnabled = True
            self.helix_angle.isEnabled = True

    def on_execute(self, args) -> 'execute':
        self.generate_gear(True, False)

        # ---------------------------------------------------
        # NSC start   C2: Improved value persistence

        # Preservers values to dictionary
        self.pers[self.gear_standard.name] = self.gear_standard.selectedItem.name
        self.pers[self.handedness.name] = self.handedness.selectedItem.name
        for i in [self.helix_angle,
                  self.pressure_angle,
                  self.module,
                  self.tooth_count,
                  self.backlash,
                  self.gear_thickness,
                  self.base_feature,
                  self.herringbone]:  # NSC C2
            self.pers[i.name] = i.value  # NSC C2

        # NSC end
        # ---------------------------------------------------

    def on_preview(self, args) -> 'preview':
        ret = self.generate_gear(self.full_preview.value, True)
        if self.full_preview.value and ret:
            args.isValidResult = True
            self.preserve_inputs()

    @property
    def resource_dir(self):
        try:
            resource_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'resources')
            return resource_dir if os.path.isdir(resource_dir) else ''
        except:
            return ''

    def make_gear(self):
        gear_standard = self.gear_standard.eval()

        if gear_standard == GearStandards.normal_system:
            gear = HelicalGear.create_in_normal_system(
                self.tooth_count.eval(),
                self.module.eval(),
                self.pressure_angle.eval(),
                self.helix_angle.eval(),
                self.handedness.eval(),
                self.backlash.eval())
        elif gear_standard == GearStandards.radial_system:
            gear = HelicalGear.create_in_radial_system(
                self.tooth_count.eval(),
                self.module.eval(),
                self.pressure_angle.eval(),
                self.helix_angle.eval(),
                self.handedness.eval(),
                self.backlash.eval())
        elif gear_standard == GearStandards.sunderland:
            gear = HelicalGear.create_sunderland(
                self.tooth_count.eval(),
                self.module.eval(),
                self.handedness.eval(),
                self.backlash.eval())
        else:
            assert False, 'Gear standard not yet supported.'

        gear_stat_text = str(gear)

        if self.last_gear_stat_text != gear_stat_text:
            self.last_gear_stat_text = gear_stat_text
            self.gear_stats.numRows = len(gear_stat_text.split('\n')) + 1
            self.gear_stats.formattedText = 'Stats:<pre>' + gear_stat_text + '</pre>'
        return gear

    def generate_gear(self, generate_all_teeth, is_preview):
        gear = self.make_gear()
        if not gear.is_valid:
            return False
        if not generate_all_teeth:
            return False

        loops = self.gear_thickness.eval() / gear.pitch_helix.vertical_loop_seperation
        if loops > 25 and is_preview:
            return False
        if gear.tooth_count > 150 and is_preview:
            return False

        component = self.design.CreateNewComponent()
        component.name = 'Healical Gear ({0}{1}@{2:.2f} m={3})'.format(
            gear.tooth_count,
            'R' if gear.handedness == Handedness.right else 'L',
            math.degrees(gear.helix_angle),
            round(gear.normal_module * 10, 4))

        # ---------------------------------------------------
        # NSC start
        if self.base_feature.value and self.design.design.designType:
            self.basefeat = component.features.baseFeatures.add()  # NSC
            self.basefeat.startEdit()  # NSC

        pitch_helix = gear.pitch_helix

        thickness = (self.gear_thickness.value / 2) if self.herringbone.value else self.gear_thickness.value

        # Creates sketch
        sketch = component.sketches.add(component.xYConstructionPlane)

        # Names sketch
        sketch.name = 'Gear ({0:.3f} module; {1:.3f} pitch dia)'.format(gear.module, gear.pitch_diameter)

        involute = Involute(gear)

        sketch.isComputeDeferred = True
        # Draws all the teeth
        for i in range(gear.tooth_count):
            involute.draw(sketch, 0, (i / gear.tooth_count) * 2 * math.pi)

        sketch.isComputeDeferred = False

        # Draws base circle
        sketch.sketchCurves.sketchCircles.addByCenterRadius(fission.Point3D(0, 0), gear.root_diameter / 2)

        # Draws the sweep path (Vertical line with length of thickness)
        line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0),
                                                               adsk.core.Point3D.create(0, 0, thickness))

        # Turns that line into a path
        path1 = component.features.createPath(line1)

        if (self.herringbone.value):
            line2 = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0),
                                                                   adsk.core.Point3D.create(0, 0, -thickness))
            path2 = component.features.createPath(line2)

        # Creates sweep

        profs = adsk.core.ObjectCollection.create()

        for prof in sketch.profiles:
            profs.add(prof)

        sweepInput = component.features.sweepFeatures.createInput(profs, path1,
                                                                  adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        sweepInput.twistAngle = adsk.core.ValueInput.createByString(str(pitch_helix.t_for(thickness)) + " rad")
        sweepFeature = component.features.sweepFeatures.add(sweepInput)

        if (self.herringbone.value):
            sweepInput2 = component.features.sweepFeatures.createInput(profs, path2,
                                                                       adsk.fusion.FeatureOperations.JoinFeatureOperation)
            sweepInput2.twistAngle = adsk.core.ValueInput.createByString(str(-pitch_helix.t_for(thickness)) + " rad")
            sweepFeature2 = component.features.sweepFeatures.add(sweepInput2)

        print(self.base_feature.value)
        print(self.design.design.designType)

        if self.base_feature.value:
            self.basefeat.finishEdit()
            timelineGroups = self.design.design.timeline.timelineGroups
            endIndex = self.basefeat.timelineObject.index
            timelineGroup = timelineGroups.add(endIndex - 1, endIndex)
            timelineGroup.name = component.name
        else:
            if self.design.design.designType:
                timelineGroups = self.design.design.timeline.timelineGroups
                startIndex = component.features[0].timelineObject.index - 2  # <--   Not good code but works
                endIndex = sweepFeature.timelineObject.index + (1 if self.herringbone.value else 0)
                timelineGroup = timelineGroups.add(startIndex, endIndex)
                timelineGroup.name = component.name

        sketch.isVisible = False

        return (gear, component, sketch)


def run(context):
    global __addin
    __addin = HelicalGearAddin()
    __addin.run()


def stop(context):
    global __addin
    if __addin:
        __addin.stop(context)

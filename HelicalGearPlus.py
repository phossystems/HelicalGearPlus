#TODO
# next steps
# v  hole in gear
# v  key way in gear
#    update validate funtion
#
#    can we speed up the gear creation by copying teeth using circular pattern
#       this allows fillet etc on teeth
#
#    if profile shift is used the positioning is not correct should we calc the centercentercorrectioncoeff ?
#
#    check in internal gear position works too
#    check rack positioning
#
# enhancements:
#    end relief (chamfer at the side edges not on the top)
#    chamfer
#    allow appearance in timeline 



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
#
# other sources used :
#   https://www.tec-science.com/mechanical-power-transmission/involute-gear/geometry-of-involute-gears/
#   https://www.tec-science.com/mechanical-power-transmission/involute-gear/calculation-of-involute-gears/
#   https://www.tec-science.com/mechanical-power-transmission/involute-gear/profile-shift/
#   https://www.tec-science.com/mechanical-power-transmission/involute-gear/meshing-line-action-contact-pitch-circle-law/
#   https://www.tec-science.com/mechanical-power-transmission/involute-gear/undercut/
#   https://www.tec-science.com/mechanical-power-transmission/involute-gear/rack-meshing/
#
# verification of profile shift
#   https://khkgears.net/new/gear_knowledge/gear_technical_reference/involute_gear_profile.html tabel 3.5
#
# verification of center center calculation
#   https://khkgears.net/new/gear_knowledge/gear_technical_reference/calculation_gear_dimensions.html tabel 4.3

import adsk.core, adsk.fusion, traceback
import math

# Global set of event _handlers to keep them referenced for the duration of the command
_handlers = []

# Caches last gear for 
lastGear = None
lastInput = ""

COMMANDID = "helicalGearPlus"
COMMANDNAME = "Helical Gear+"
COMMANDTOOLTIP = "Generates Helical Gears"
TOOLBARPANELS = ["SolidCreatePanel"]

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
    'VIDedendum': 1.25,
    'VIShift': 0.0,
    'BVCtrCtr': False,
    'VIShift2': 0.0,
    'VICtrCtr': 0.0,
    'ISTeeth2': 24,
    'BVNoUnderCut': False,
    'BVBore':True,
    'VIBoreDiameter':0.6,
    'BVKey':True,
    'VIKeyWidth': 0.3,
    'VIKeyHeight':0.3}

# TO DONE : have bore and key parameters persistent between edits

class Involute:
    def __init__(self, gear):
        self.gear = gear

    def draw(self, sketch, zShift=0, rotation=0, involutePointCount=10):
        # Calculate points along the involute curve.
        originPoint = adsk.core.Point3D.create(0, 0, zShift)
        involutePoints = []
        keyPoints = []

        if self.gear.baseDiameter >= self.gear.rootDiameter:
            involuteFromRad = self.gear.baseDiameter / 2.0
        else:
            involuteFromRad = self.gear.rootDiameter / 2
        radiusStep = (self.gear.outsideDiameter / 2 - involuteFromRad) / (involutePointCount - 1)
        involuteIntersectionRadius = involuteFromRad
        for i in range(0, involutePointCount):
            newPoint = self.InvolutePoint(self.gear.baseDiameter / 2.0, involuteIntersectionRadius, zShift)
            involutePoints.append(newPoint)
            involuteIntersectionRadius = involuteIntersectionRadius + radiusStep

        # Determine the angle between the X axis and a line between the origin of the curve
        # and the intersection point between the involute and the pitch diameter circle.
        pitchInvolutePoint = self.InvolutePoint(self.gear.baseDiameter / 2.0, self.gear.pitchDiameter / 2.0,
                                                zShift)
        pitchPointAngle = math.atan2(pitchInvolutePoint.y, pitchInvolutePoint.x)

        # Rotate the involute so the intersection point lies on the x axis.
        # add half of the profile shift 
        # TODO the next 3 sections can be combined in less loops and calculation
        halfShiftAngle = self.gear.modifier.profileShift * math.tan(self.gear.pressureAngle)

        rotateAngle = -((self.gear.toothArcAngle / 4) + pitchPointAngle - (self.gear.backlashAngle / 4) + halfShiftAngle/2)
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
            involute2Points.append(adsk.core.Point3D.create(involutePoints[i].x, -involutePoints[i].y, zShift))

        # Rotate involute
        # TODO check if this is just a rotation ? if so get rid of it since rotating the gear is simpler
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

        midIndex = int(pointSet1.count / 2)
        keyPoints.append(pointSet1.item(0))
        keyPoints.append(pointSet2.item(0))
        keyPoints.append(pointSet1.item(midIndex))
        keyPoints.append(pointSet2.item(midIndex))

        # Create splines.
        spline1 = sketch.sketchCurves.sketchFittedSplines.add(pointSet1)
        spline2 = sketch.sketchCurves.sketchFittedSplines.add(pointSet2)
        oc = adsk.core.ObjectCollection.create()
        oc.add(spline2)
        (_, _, crossPoints) = spline1.intersections(oc)
        assert len(crossPoints) == 0 or len(crossPoints) == 1, 'Failed to compute a valid involute profile!'
        if len(crossPoints) == 1:
            # involute splines cross, clip the tooth
            # clip = spline1.endSketchPoint.geometry.copy()
            # spline1 = spline1.trim(spline2.endSketchPoint.geometry).item(0)
            # spline2 = spline2.trim(clip).item(0)
            keyPoints.append(crossPoints[0])
        else:
            # Draw the tip of the tooth - connect the splines
            if self.gear.toothCount >= 100:
                sketch.sketchCurves.sketchLines.addByTwoPoints(spline1.endSketchPoint, spline2.endSketchPoint)
                keyPoints.append(spline1.endSketchPoint.geometry)
                keyPoints.append(spline2.endSketchPoint.geometry)
            else:
                tipCurve1Angle = math.atan2(involutePoints[-1].y, involutePoints[-1].x)
                tipCurve2Angle = math.atan2(involute2Points[-1].y, involute2Points[-1].x)
                if tipCurve2Angle < tipCurve1Angle:
                    tipCurve2Angle += math.pi * 2
                tipRad = originPoint.distanceTo(involutePoints[-1])
                tipArc = sketch.sketchCurves.sketchArcs.addByCenterStartSweep(
                    originPoint,
                    adsk.core.Point3D.create(math.cos(tipCurve1Angle) * tipRad,
                                             math.sin(tipCurve1Angle) * tipRad,
                                             zShift),
                    tipCurve2Angle - tipCurve1Angle)
                keyPoints.append(tipArc.startSketchPoint.geometry)
                keyPoints.append(adsk.core.Point3D.create(tipRad, 0, zShift))
                keyPoints.append(tipArc.endSketchPoint.geometry)

        # Draw root circle
        # rootCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(originPoint, self.gear.rootDiameter/2)
        rootArc = sketch.sketchCurves.sketchArcs.addByCenterStartSweep(
            originPoint,
            adsk.core.Point3D.create(math.cos(curve1Angle) * (self.gear.rootDiameter / 2 - 0.01),
                                     math.sin(curve1Angle) * (self.gear.rootDiameter / 2 - 0.01),
                                     zShift),
            curve2Angle - curve1Angle)

        # if the offset tooth profile crosses the offset circle then trim it, else connect the offset tooth to the circle
        oc = adsk.core.ObjectCollection.create()
        oc.add(spline1)
        if True:
            if rootArc.intersections(oc)[1].count > 0:
                spline1 = spline1.trim(originPoint).item(0)
                spline2 = spline2.trim(originPoint).item(0)
                rootArc.trim(rootArc.startSketchPoint.geometry)
                rootArc.trim(rootArc.endSketchPoint.geometry)
            else:
                sketch.sketchCurves.sketchLines.addByTwoPoints(originPoint, spline1.startSketchPoint).trim(
                    originPoint)
                sketch.sketchCurves.sketchLines.addByTwoPoints(originPoint, spline2.startSketchPoint).trim(
                    originPoint)
        else:
            if rootArc.intersections(oc)[1].count > 0:
                spline1 = spline1.trim(originPoint).item(0)
                spline2 = spline2.trim(originPoint).item(0)
            rootArc.deleteMe()
            sketch.sketchCurves.sketchLines.addByTwoPoints(originPoint, spline1.startSketchPoint)
            sketch.sketchCurves.sketchLines.addByTwoPoints(originPoint, spline2.startSketchPoint)

    # Calculate points along an involute curve.
    def InvolutePoint(self, baseCircleRadius, distFromCenterToInvolutePoint, zShift):
        l = math.sqrt(
            distFromCenterToInvolutePoint * distFromCenterToInvolutePoint - baseCircleRadius * baseCircleRadius)
        alpha = l / baseCircleRadius
        theta = alpha - math.acos(baseCircleRadius / distFromCenterToInvolutePoint)
        x = distFromCenterToInvolutePoint * math.cos(theta)
        y = distFromCenterToInvolutePoint * math.sin(theta)
        return adsk.core.Point3D.create(x, y, zShift)
        
    def Involute(angle):
        return math.tan(angle)-angle

    def InverseInvolute(I):
    # https://gearsolutions.com/features/calculating-the-inverse-of-an-involute/
    # https://www.researchgate.net/publication/336140084_An_Analytic_Expression_for_the_Inverse_Involute
    #         
        angle = 1.44792*pow(I,1/3)-0.0472447 *pow(I,2/3) -0.29949*I
        for i in range(3):
            angle = angle + (I - (math.tan(angle) - angle))/pow(math.tan(angle),2)
        return angle

class GearModifier:
    def __init__(self,profileshift = 0, otherToothCount = 0, otherProfileShift = 0, centerCenterDistanct = 0 ):
        self.profileShift = profileshift
        self.otherToothCount = otherToothCount
        self.otherProfileShift = otherProfileShift
        self.centerCenterDistance = centerCenterDistanct

    def centerDistanceCorrectionFactor(Gear):
    # TO DONE add calculation can't be a property
        halfTotalTeeth = (Gear.otherToothCount  + otherToothCount)/2
        centerModificationCoefficient = centerCenterDistance / Gear.module - halfTotalTeeth
        workingPressureAngle = math.acos( math.cos(Gear.press)/(centerModificationCoefficient/halfTotalTeeth +1))
        sumshift = halfTotalTeeth*(Involute.Involute(workingPressureAngle) - Involute.Involute(Gear.press))/math.tan(Gear.press)
                    
        if (Gear.profileshift != 0):
            shift2 = sumshift - Gear.profileshift
        else:
            if (otherProfileShift != 0):
                Gear.profileshift = sumshift - otherProfileShift
            else:
                Gear.profileshift = sumshift / 2
                otherProfileShift = shift1

        return centerModificationCoefficient

class GearMount:
    def __init__(self, boreDiameter = 0, keyHeight = 0, keyWidth = 0):    
        self.bore = boreDiameter
        self.keyHeight = keyHeight
        self.keyWidth = keyWidth

    
        
class HelicalGear(object):
    def __init__(self):
        pass

    @property
    def module(self):
        return self.__module

    @module.setter
    def module(self, val):
        self.__module = val
        
    @property
    def isUndercutRequried(self):
        return self.virtualTeeth < self.critcalVirtualToothCount        

    @property
    def backlashAngle(self):
        """The backlash is split between both sides of this and (an assumed) mating gear - each side of a tooth will be narrowed by 1/4 this value."""
        return 2 * self.backlash / self.pitchDiameter if self.pitchDiameter > 0 else 0

    @property
    def toothArcAngle(self):
        """Arc angle of a single tooth."""
        return 2 * math.pi / self.toothCount if self.toothCount > 0 else 0

    @property
    def toothThickness(self):
        """"Thickness of the tooth at reference pitch circle"""
        return self.module *(math.pi / 2 + 2 * self.modifier.profileShift * math.tan(self.pressureAngle))

    @property
    def tipPressureAngle(self):
        """Pressure angle at the tip of the tooth."""
        return math.acos(self.baseDiameter / self.outsideDiameter)

    @property
    def involuteA(self):
        """Involute at nominal pressure angle."""
        return math.tan(self.pressureAngle) - self.pressureAngle

    @property
    def involuteAa(self):
        """Involute at tip pressure angle."""
        return math.tan(self.tipPressureAngle) - self.tipPressureAngle

    @property
    def involuteW(self): # from table 4.4  http://qtcgears.com/tools/catalogs/PDF_Q420/Tech.pdf
        """Involute of the working pressure angle."""
        return 2 * math.tan(self.pressureAngle) * (self.modifier.profileShift + self.modifier.otherProfileShift) / (self.toothCount + self.modifier.otherToothCount) + self.involuteA

    @property
    def workingPressureAngle(self):
        """Working pressure angle."""
        return Involute.InverseInvolute(self.involuteW)
    
    @property
    def pitchDiameter(self):
        """Pitch diameter, contact point diameter at given pressure angle."""
        return module * toothcount

    @property 
    def baseDiameter(self):
        """Base diameter """
        return pitchDiameter * math.cos(gear.pressureAngle)

    @property
    def workingPitchDiameter(self):
        """"Working Pitch diameter, point of contact on the working pressure angle."""        
        return baseDiameter / math.cos(workingPressureAngle)

    @property
    def centerDistanceIncrementFactor(self):
        """Center increment factor due to profile shift."""
        return (self.toothCount + self.modifier.otherToothCount)/2 * (math.cos(self.pressureAngle) / math.cos(self.workingPressureAngle) - 1 )

    @property
    def topLandAngle(self):
    # v TO DONE fix this        verified with documentation
        """Top land is the (sometimes flat) surface of the top of a gear tooth.
        DOES NOT APPEAR TO PRODUCE THE CORRECT VALUE."""
        return ((math.pi / (2 * self.toothCount)) + ((2 * self.modifier.profileShift * math.tan(self.pressureAngle)) / self.toothCount) + (self.involuteA- self.involuteAa))

    @property
    def topLandThickness(self):
    # v TO DONE fix this         verified with documentation
        """Top land is the (sometimes flat) surface of the top of a gear tooth.
        DOES NOT APPEAR TO PRODUCE THE CORRECT VALUE."""
        return self.topLandAngle * self.outsideDiameter

    @property
    def critcalVirtualToothCount(self):
        q = math.pow(math.sin(self.normalPressureAngle), 2)
        return 2*(1 - self.modifier.profileShift) / q if q != 0 else float('inf')

    @property
    def isInvalid(self):
        if (self.width <= 0):
            return "Width too low"
        if (math.radians(-90) > self.helixAngle):
            return "Helix angle too low"
        if (math.radians(90) < self.helixAngle):
            return "Helix angle too high"
        if (self.module <= 0):
            return "Module to low"
        if (self.addendum <= 0):
            return "Addendum too low"
        if (self.wholeDepth <= 0):
            return "Dedendum too low"
        if (self.pressureAngle < 0):
            return "Pressure angle too low"
        if (self.pressureAngle > math.radians(80)):
            return "Pressure angle too high"
        if (self.normalPressureAngle < 0):
            return "Pressure angle too low"
        if (self.normalPressureAngle > math.radians(80)):
            return "Pressure angle too high"
        if (self.toothCount <= 0):
            return "Too few teeth"
        if (abs(self.backlashAngle) / 4 >= self.toothArcAngle / 8):
            return "Backlash too high"
        if (self.internalOutsideDiameter):
            if (self.internalOutsideDiameter <= self.outsideDiameter):
                return "Outside diameter too low"
        if (self.circularPitch <= 0):
            return "Invalid: circularPitch"
        if (self.baseDiameter <= 0):
            return "Invalid Gear"
        if (self.pitchDiameter <= 0):
            return "Invalid Gear"
        if (self.rootDiameter <= 0.03):
            return "Invalid Gear"
        if (self.outsideDiameter <= 0):
            return "Invalid Gear"
        return False

    @property
    def verticalLoopSeperation(self):
        return math.tan(math.radians(90) + self.helixAngle) * self.pitchDiameter * math.pi

    # returns the number of turns for a given distance
    def tFor(self, displacement):
        return displacement / (math.tan(math.radians(90) + self.helixAngle) * (self.pitchDiameter / 2))

    def __str__(self):
        # TODO make 2 versions short version and extended based on checkbox input
        str = ''
        str += '\n'
        str += 'root diameter..............:  {0:.3f} mm\n'.format(self.rootDiameter * 10)
        str += 'base diameter..............:  {0:.3f} mm\n'.format(self.baseDiameter * 10)
        str += 'pitch diameter.............:  {0:.3f} mm\n'.format(self.pitchDiameter * 10)
        str += 'outside diameter...........:  {0:.3f} mm\n'.format(self.outsideDiameter * 10)
        str += 'profile shift coefficient : {0:.3f} \n'.format(self.modifier.profileShift)
        str += '\n'
        str += 'module.....................:  {0:.3f} mm\n'.format(self.module * 10)
        str += 'normal module..............:  {0:.3f} mm\n'.format(self.normalModule * 10)
        str += 'pressure angle.............:  {0:.4f} deg\n'.format(math.degrees(self.pressureAngle))
        str += 'normal press. angle......:  {0:.4f} deg\n'.format(math.degrees(self.normalPressureAngle))
        str += 'Working press. angle.....:  {0:.4f} deg\n'.format(math.degrees(self.workingPressureAngle))
        str += '\n'
        if (self.helixAngle != 0):
            str += 'length per revolution......:  {0:.3f} mm\n'.format(abs(self.verticalLoopSeperation) * 10)
            str += '\n'
        str += 'Needs undercut.............: {!s:^5} \n'.format(self.isUndercutRequried)
        str += 'backlash angle.............: {0:.3f} deg\n'.format(math.degrees(self.backlashAngle))
        str += 'addendum...................: {0:.3f} mm\n'.format(self.addendum * 10)
        str += 'whole depth................: {0:.3f} mm\n'.format(self.wholeDepth * 10)
        str += 'Tip Pressure angle.........: {0:.3f} deg\n'.format(math.degrees(self.tipPressureAngle))
        str += 'Tooth arc angle............: {0:.3f} deg\n'.format(math.degrees(self.toothArcAngle))
        str += 'Top land angle.............: {0:.3f} deg\n'.format(math.degrees(self.topLandAngle))
        str += 'Tooth thickness............: {0:.3f} mm\n'.format(self.toothThickness * 10)
        str += 'Top land thickness.........: {0:.3f} mm\n'.format(self.topLandThickness * 10)
        str += 'Center increment factor....: {0:.5f}\n'.format(self.centerDistanceIncrementFactor)
        str += '\n'
        str += 'Involute A..................: {0:.6f}\n'.format(self.involuteA)
        str += 'Involute Aa.................: {0:.6f}\n'.format(self.involuteAa)
        str += 'Involute W..................: {0:.6f}\n'.format(self.involuteW)

        return str

    @staticmethod
    def createInNormalSystem(toothCount, normalModule, normalPressureAngle, helixAngle, backlash=0, addendum=1,
                             dedendum=1.25, width=1, herringbone=False, internalOutsideDiameter=None,
                             gearmodifier = None, gearmount = None ):
#                             dedendum=1.25, profileshift=0, width=1, herringbone=False, internalOutsideDiameter=None,toothcount2=0,profileshift2=0,
#                             ctrctrcorrection=0, bore=0, keyheight=0, keywidth=0):
# TODO  should we replace the profileshift2 by the center-center correction factor
        toothCount = toothCount if toothCount > 0 else 1
        # normalModule = normalModule if normalModule > 0 else 1e-10
        # normalPressureAngle = normalPressureAngle if 0 <= normalPressureAngle < math.radians(90) else 0
        # helixAngle = helixAngle if math.radians(-90) < helixAngle < math.radians(90) else 0

        gear = HelicalGear()
        gear.backlash = backlash
        gear.helixAngle = helixAngle
        gear.toothCount = toothCount
        gear.width = width
        gear.herringbone = herringbone
        gear.internalOutsideDiameter = internalOutsideDiameter
        gear.modifier = gearmodifier if not gearmodifier is None else GearModifier
        #gear.toothCount2 = toothcount2
        #gear.profileShift2 = profileshift2
        #gear.centerCenterCorrection = ctrctrcorrection
        gear.mount = gearmount if not gearmount is None else GearMount
        #gear.bore = bore
        #gear.keyHeight = keyheight
        #gear.keyWidth = keywidth

        gear.normalModule = normalModule
        gear.normalPressureAngle = normalPressureAngle

        gear.normalCircularPitch = gear.normalModule * math.pi
        cosHelixAngle = math.cos(helixAngle)
        gear.virtualTeeth = gear.toothCount / math.pow(cosHelixAngle, 3)

        # Radial / Transverse figures
        gear.module = gear.normalModule / cosHelixAngle                                     #
        gear.
        gear.pressureAngle = math.atan2(math.tan(gear.normalPressureAngle), cosHelixAngle)  #
        ## gear.pitchDiameter = gear.module * gear.toothCount                                  # v
        ## gear.baseDiameter = gear.pitchDiameter * math.cos(gear.pressureAngle)               # v
        # TODO add profile shift
        #
        ## gear.workingPitchDiameter = gear.baseDiameter / math.cos(gear.workingPressureAngle)
        
        # gear.addendum = addendum
        # gear.dedendum = dedendum
        # gear.addendumDiameter = gear.module * (gear.toothCount + 2 * gear.modifier.profileShift + 2 * gear.addendum)
        # gear.dedemdumDiameter = gear.module * (gear.toothCount + 2 * gear.modifier.profileShift - 2 * gear.dedendum) 

        if (gear.modifier.otherProfileShift == 0):
            gear.addendum = (addendum  + gear.modifier.profileShift) * gear.module 
            gear.wholeDepth = (addendum + dedendum ) * gear.module 
        else:
            gear.addendum = ( addendum  + gear.centerDistanceIncrementFactor - gear.modifier.otherProfileShift ) * gear.module
            gear.wholeDepth = (addendum + dedendum + gear.centerDistanceIncrementFactor - gear.modifier.profileShift - gear.modifier.otherProfileShift) * gear.module 
        
        # gear.wholeDepth = (addendum + dedendum + gear.centerDistanceIncrementFactor - gear.modifier.profileShift - profileshift2) * gear.module 
        gear.tipDiameter = gear.pitchDiameter + 2 * gear.addendum
        # is the next line correct ???
        gear.outsideDiameter = gear.workingPitchDiameter + 2 * gear.addendum 
        gear.outsideDiameter = gear.tipDiameter
        gear.rootDiameter = gear.outsideDiameter - 2 * gear.wholeDepth # this should be workingPitchDiameter when profile shifted 1/11/2020
        gear.circularPitch = gear.module * math.pi

        return gear

    @staticmethod
    def createInRadialSystem(toothCount, radialModule, radialPressureAngle, helixAngle, backlash=0, addendum=1,
                             dedendum=1.25, width=1, herringbone=False, internalOutsideDiameter=None,
                             gearmodifier = None, gearmount = None ):
#                             toothcount2=0,profileshift2=0):
        toothCount = toothCount if toothCount > 0 else 1
        radialModule = radialModule if radialModule > 0 else 1e-10
        radialPressureAngle = radialPressureAngle if 0 <= radialPressureAngle < math.radians(90) else 0
        helixAngle = helixAngle if math.radians(-90) < helixAngle < math.radians(90) else 0

        gear = HelicalGear()
        gear.backlash = backlash
        gear.helixAngle = helixAngle
        gear.toothCount = toothCount
        gear.width = width
        gear.herringbone = herringbone
        gear.internalOutsideDiameter = internalOutsideDiameter

        gear.normalModule = radialModule * math.cos(gear.helixAngle)
        gear.normalPressureAngle = math.atan(math.tan(radialPressureAngle) * math.cos(gear.helixAngle))
        gear.normalCircularPitch = gear.normalModule * math.pi

        cosHelixAngle = math.cos(helixAngle)
        gear.virtualTeeth = gear.toothCount / math.pow(cosHelixAngle, 3)

        # Radial / Transverse figures
        gear.module = radialModule
        gear.pressureAngle = radialPressureAngle
        ## gear.pitchDiameter = gear.module * gear.toothCount
        ## gear.baseDiameter = gear.pitchDiameter * math.cos(gear.pressureAngle)

        gear.modifier = gearmodifier if not gearmodifier is None else GearModifier
        #gear.toothCount2 = toothcount2
        #gear.profileShift2 = profileshift2
        #gear.centerCenterCorrection = ctrctrcorrection
        gear.mount = gearmount if not gearmount is None else GearMount
        #gear.bore = bore
        #gear.keyHeight = keyheight
        #gear.keyWidth = keywidth

        

        # fixed addendum to 1 instead of taking the value of the UI
        # fixed dedendum to 1.25 instead of taking the value of the UI
        #   the clearance is set to 0.25 while 0.167 is sufficient
        #
        #  https://www.tec-science.com/mechanical-power-transmission/involute-gear/geometry-of-involute-gears/
        #  addendumdiameter = pitchdiameter + 2 * module
        #  dedendumdiameter = pitchdiameter - 2 * module - 2 * clearance
        #  wholedepth = addendum + dedendum + clearance = 2 * module + clearance 
        #
        #  with profile shift coeff x
        #
        #  addendumdiameter = module * ( toothcount + 2x + 2)
        #  dedendumdiameter = module * ( toothcount + 2x - 2) - 2 * clearance
        #  wholedepth  does not change by a profile shift
        #  wholedepth = (addendumdiameter - dedendumdiameter) / 2 = 2 * module + clearnce

        # TODO add profile shift
        gear.addendum = ( addendum  + profileshift ) * gear.module
        gear.wholeDepth = (addendum + dedendum) * gear.module
        gear.outsideDiameter = gear.pitchDiameter + 2 * gear.addendum
        gear.rootDiameter = gear.outsideDiameter - 2 * gear.wholeDepth
        gear.circularPitch = gear.module * math.pi

        return gear

    def modelGear(self, parentComponent, sameAsLast=False):
        # Stores a copy of the last gear generated to speed up regeneation of the same gear
        global lastGear
        try: 
            # The temporaryBRep manager is a tool for creating 3d geometry without the use of features
            # The word temporary referrs to the geometry being created being virtual, but It can easily be converted to actual geometry
            tbm = adsk.fusion.TemporaryBRepManager.get()
            # Create new component
            occurrence = parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            component = occurrence.component
            if (self.modifier.profileShift != 0 ):
                component.name = 'Healical Gear ({0}{1}@{2:.2f} m={3} x={4:.2f})'.format(
                    self.toothCount,
                    'L' if self.helixAngle < 0 else 'R',
                    abs(math.degrees(self.helixAngle)),
                    round(self.normalModule * 10, 4),
                    self.modifier.profileShift)
            else:
                component.name = 'Healical Gear ({0}{1}@{2:.2f} m={3})'.format(
                    self.toothCount,
                    'L' if self.helixAngle < 0 else 'R',
                    abs(math.degrees(self.helixAngle)),
                    round(self.normalModule * 10, 4))
            
            # Creates BaseFeature if DesignType is parametric 
            if (parentComponent.parentDesign.designType):
                baseFeature = component.features.baseFeatures.add()
                baseFeature.startEdit()
            else:
                baseFeature = None

            if (not (sameAsLast and lastGear)):

                # Creates sketch and draws tooth profile
                involute = Involute(self)

                # Creates profile on z=0 if herringbone and on bottom if not
                if (not self.herringbone):
                    plane = adsk.core.Plane.create(adsk.core.Point3D.create(0, 0, -self.width / 2),
                                                adsk.core.Vector3D.create(0, 0, 1))
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
    #                for i in range(self.toothCount):
    #                    involute.draw(sketch, 0, (i / self.toothCount) * 2 * math.pi)
    #                # Base Circle
    #               sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0),
    #                                                                   self.rootDiameter / 2)
                else:
                    sketch = component.sketches.add(component.xYConstructionPlane)
                    sketch.isComputeDeferred = True
                    
                # Draws All Teeth
                # TO DONE: Optimize by copying instead of regenerating
                #          this is only for the sketch. keep it to remove teeth if needed
                for i in range(self.toothCount):
                    involute.draw(sketch, 0, (i / self.toothCount) * 2 * math.pi)
                # Base Circle
                sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0),
                                                                    self.rootDiameter / 2)
                # Center Hole
                # TO DONE: use input parameter
                if (self.mount.bore > 0) : 
                    sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0),
                                                                    self.mount.bore / 2 )
                # key way
                # TO DONE: use input parameter
                    if (self.mount.keyHeight > 0 and 
                        self.mount.keyWidth > 0):
                        centerPoint = adsk.core.Point3D.create(self.mount.bore / 2, 0, 0)
                        cornerPoint = adsk.core.Point3D.create(self.mount.bore / 2 + self.mount.keyHeight , self.mount.keyWidth / 2, 0)
                        sketch.sketchCurves.sketchLines.addCenterPointRectangle(centerPoint, cornerPoint)


                # Creates path line for sweep feature
                if (not self.herringbone):
                    line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0),
                                                                        adsk.core.Point3D.create(0, 0, self.width))
                else:
                    line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0),
                                                                        adsk.core.Point3D.create(0, 0, self.width / 2))
                    line2 = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0),
                                                                        adsk.core.Point3D.create(0, 0, -self.width / 2))

                # Reactivates sketch computation and puts all profiles into an ObjectCollection (OC)
                sketch.isComputeDeferred = False
                profs = adsk.core.ObjectCollection.create()
                for prof in sketch.profiles:
                    profs.add(prof)

                # Creates sweep features
                if (not self.herringbone):
                    path1 = component.features.createPath(line1)
                    sweepInput = component.features.sweepFeatures.createInput(profs, path1,
                                                                            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
                    sweepInput.twistAngle = adsk.core.ValueInput.createByReal(-self.tFor(self.width))
                    if (baseFeature):
                        sweepInput.targetBaseFeature = baseFeature
                    gearBody = sweepFeature = component.features.sweepFeatures.add(sweepInput).bodies.item(0)
                else:
                    path1 = component.features.createPath(line1)
                    sweepInput = component.features.sweepFeatures.createInput(profs, path1,
                                                                            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
                    sweepInput.twistAngle = adsk.core.ValueInput.createByReal(-self.tFor(self.width / 2))
                    if (baseFeature):
                        sweepInput.targetBaseFeature = baseFeature
                    sweepFeature = component.features.sweepFeatures.add(sweepInput)

                    path2 = component.features.createPath(line2)
                    sweepInput = component.features.sweepFeatures.createInput(profs, path2,
                                                                            adsk.fusion.FeatureOperations.JoinFeatureOperation)
                    sweepInput.twistAngle = adsk.core.ValueInput.createByReal(self.tFor(self.width / 2))
                    if (baseFeature):
                        sweepInput.targetBaseFeature = baseFeature
                    gearBody = sweepFeature = component.features.sweepFeatures.add(sweepInput).bodies.item(0)

                cpgearBody = tbm.copy(gearBody)

                # "Inverts" internal Gears
                if (self.internalOutsideDiameter):
                    cyl = cylinder = tbm.createCylinderOrCone(adsk.core.Point3D.create(0, 0, -self.width / 2),
                                                            self.internalOutsideDiameter / 2,
                                                            adsk.core.Point3D.create(0, 0, self.width / 2),
                                                            self.internalOutsideDiameter / 2)
                    tbm.booleanOperation(cyl, tbm.copy(gearBody), 0)
                    # Deletes external gear
                    gearBody.deleteMe()

                    if (baseFeature):
                        gearBody = component.bRepBodies.add(cyl, baseFeature)
                    else:
                        gearBody = component.bRepBodies.add(cyl)
                else:
                # generate hole in the center
                # Center Hole
                # TO DONE: use input parameter
                    if (self.mount.bore > 0) : 
                        cyl = cylinder = tbm.createCylinderOrCone(adsk.core.Point3D.create(0, 0, -self.width / 2),
                                                                self.mount.bore/2,
                                                                adsk.core.Point3D.create(0, 0, self.width / 2),
                                                                self.mount.bore/2)
                        difference = adsk.fusion.BooleanTypes.DifferenceBooleanType
                        isSucces = tbm.booleanOperation( cpgearBody,cyl, difference)
                   
                # generate  key way.
                        if (self.mount.keyHeight > 0 and self.mount.keyWidth > 0):
                            centerPoint = adsk.core.Point3D.create(self.mount.bore/2, 0, 0)
                            cornerPoint = adsk.core.Point3D.create(self.mount.bore/2 + self.mount.keyHeight, self.mount.keyWidth / 2, 0)
                            sketch.sketchCurves.sketchLines.addCenterPointRectangle(centerPoint, cornerPoint)


                            edgePoint = adsk.core.Point3D.create(self.mount.bore/2 , 0, 0)
                            lengthDir = adsk.core.Vector3D.create(1.0, 0.0, 0.0)
                            widthDir = adsk.core.Vector3D.create(0.0, 1.0, 0.0)
                            orientedBoundingBox3D = adsk.core.OrientedBoundingBox3D.create(edgePoint, 
                                                                                        lengthDir,
                                                                                        widthDir,
                                                                                        self.mount.keyHeight * 2 ,  
                                                                                        self.mount.keyWidth , 
                                                                                        self.width
                                                                                        )
                    # Create box                                                              
                            key = keyway = tbm.createBox(orientedBoundingBox3D)
                    
                            isSuccess = tbm.booleanOperation( cpgearBody,keyway, difference)
                    # Deletes external gear
                    gearBody.deleteMe()

                    if (baseFeature):
                        gearBody = component.bRepBodies.add(cpgearBody, baseFeature)
                    else:
                        gearBody = component.bRepBodies.add(cpgearBody)
                   
                # Delete tooth sketch for performance
                # sketch.deleteMe()

                

                # Stores a copy of the newly generated gear    
                lastGear = tbm.copy(gearBody)
            else:
                if (baseFeature):
                    component.bRepBodies.add(lastGear, baseFeature)
                else:
                    component.bRepBodies.add(lastGear)

            # Draws pitch diameter
            pitchDiameterSketch = component.sketches.add(component.xYConstructionPlane)
            pitchDiameterSketch.name = "PD: {0:.3f}mm".format(self.pitchDiameter * 10)
            pitchDiameterCircle = pitchDiameterSketch.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(0, 0, 0), self.pitchDiameter / 2)
            pitchDiameterCircle.isConstruction = True
            pitchDiameterCircle.isFixed = True

            # Finishes BaseFeature if it exists
            if (baseFeature):
                baseFeature.finishEdit()

            # add attributes to component to remember when used in placement
            gear_attributes = occurrence.attributes
            gear_attributes.add('HelicalGear', 'type', "Gear")
            gear_attributes.add('HelicalGear', 'module', "{0}".format(self.module))
            gear_attributes.add('HelicalGear', 'teeth', "{0}".format(self.toothCount))
            gear_attributes.add('HelicalGear', 'pressureAngle', "{0}".format(self.pressureAngle))
            gear_attributes.add('HelicalGear', 'helixAngle', "{0}".format(self.helixAngle))
            gear_attributes.add('HelicalGear', 'rootDiameter', "{0}".format(self.rootDiameter))
            gear_attributes.add('HelicalGear', 'baseDiameter', "{0}".format(self.baseDiameter))
            gear_attributes.add('HelicalGear', 'pitchDiameter', "{0}".format(self.pitchDiameter))
            gear_attributes.add('HelicalGear', 'outsideDiameter', "{0}".format(self.outsideDiameter))
            gear_attributes.add('HelicalGear', 'backlash', "{0}".format(self.backlash))
            gear_attributes.add('HelicalGear', 'herringbone', "{0}".format(self.herringbone))
            # gear mounting
            gear_attributes.add('HelicalGear', 'bore', "{0}".format(self.mount.bore))
            gear_attributes.add('HelicalGear', 'keyWidth', "{0}".format(self.mount.keyWidth))
            gear_attributes.add('HelicalGear', 'keyHeight', "{0}".format(self.mount.keyHeight))
            # TODO add additional attributes
            # gear modifiers
            gear_attributes.add('HelicalGear', 'profileShiftCoefficient', "{0}".format(self.modifier.profileShift))
            # gear_attributes.add('HelicalGear', 'centercorrectionfactor', "{0}".format(self.herringbone))

            return occurrence

        except:
            print(traceback.format_exc())


class RackGear:

    def __init__(self):
        pass

    @staticmethod
    def createInNormalSystem(normalModule, normalPressureAngle, helixAngle, herringbone, length, width, height,
                             backlash=0, addendum=1, dedendum=1.25):
        gear = RackGear()

        gear.normalModule = normalModule
        gear.normalPressureAngle = normalPressureAngle
        gear.helixAngle = helixAngle
        gear.herringbone = herringbone
        gear.length = length
        gear.width = width
        gear.height = height
        gear.backlash = backlash

        gear.addendum = addendum * gear.normalModule
        gear.dedendum = dedendum * gear.normalModule

        cosHelixAngle = math.cos(helixAngle)
        gear.module = gear.normalModule / cosHelixAngle
        gear.pressureAngle = math.atan2(math.tan(gear.normalPressureAngle), cosHelixAngle)

        return gear

    @staticmethod
    def createInRadialSystem(radialModule, radialPressureAngle, helixAngle, herringbone, length, width, height,
                             backlash=0, addendum=1, dedendum=1.25):
        gear = RackGear()

        gear.module = radialModule
        gear.pressureAngle = radialPressureAngle
        gear.helixAngle = helixAngle
        gear.herringbone = herringbone
        gear.length = length
        gear.width = width
        gear.height = height
        gear.backlash = backlash

        gear.addendum = addendum * gear.module
        gear.dedendum = dedendum * gear.module

        cosHelixAngle = math.cos(helixAngle)
        gear.normalModule = gear.module * cosHelixAngle
        gear.normalPressureAngle = math.atan(math.tan(radialPressureAngle) * math.cos(gear.helixAngle))

        return gear

    def __str__(self):
        str = ''
        str += '\n'
        str += 'module.......................:  {0:.3f} mm\n'.format(self.module * 10)
        str += 'normal module...........:  {0:.3f} mm\n'.format(self.normalModule * 10)
        str += 'pressure angle............:  {0:.3f} deg\n'.format(math.degrees(self.pressureAngle))
        str += 'normal pressure angle:  {0:.3f} deg\n'.format(math.degrees(self.normalPressureAngle))
        str += '\n'
        return str

    @property
    def isInvalid(self):
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
        if (not (0 < self.pressureAngle < math.radians(90))):
            return "Invalid pressure angle"
        if (not (math.radians(-90) < self.helixAngle < math.radians(90))):
            return "Invalid helix angle"
        # Not actually the limit but close enough
        if ((-3 * self.normalModule) > self.backlash):
            return "Backlash too low"
        if (self.backlash > (3 * self.normalModule)):
            return "Backlash too high"
# bore has to be less than the root diameter
# bore + key has to be less than root diameter
# key width ???
# profile shift limits ???
# backlash ???
# center center ???? or is this covered by profile shift

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

    def modelGear(self, parentComponent, sameAsLast=False):
        # Storres a copy of the last gear generated to speed up regeneation of the same gear
        global lastGear

        # Create new component
        occurrence = parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        component = occurrence.component
        component.name = 'Healical Rack ({0}mm {1}@{2:.2f} m={3})'.format(
            self.length * 10,
            'L' if self.helixAngle < 0 else 'R',
            abs(math.degrees(self.helixAngle)),
            round(self.normalModule * 10, 4))
        if (parentComponent.parentDesign.designType):
            baseFeature = component.features.baseFeatures.add()
            baseFeature.startEdit()
        else:
            baseFeature = None

        if (not (sameAsLast and lastGear)):

            teeth = math.ceil(
                (self.length + 2 * math.tan(abs(self.helixAngle)) * self.width) / (self.normalModule * math.pi))
            # The temporaryBRep manager is a tool for creating 3d geometry without the use of features
            # The word temporary referrs to the geometry being created being virtual, but It can easily be converted to actual geometry
            tbm = adsk.fusion.TemporaryBRepManager.get()
            # Array to keep track of TempBRepBodies
            tempBRepBodies = []
            # Creates BRep wire object(s), representing edges in 3D space from an array of 3Dcurves
            if (self.herringbone):
                wireBody1, _ = tbm.createWireFromCurves(self.rackLines(
                    -self.length / 2 - (math.tan(abs(self.helixAngle)) + math.tan(self.helixAngle)) * self.width / 2,
                    -self.width / 2,
                    0,
                    self.normalModule, teeth, self.height, self.normalPressureAngle, self.helixAngle,
                    self.backlash, self.addendum, self.dedendum
                ))
                wireBody2, _ = tbm.createWireFromCurves(self.rackLines(
                    -self.length / 2 - math.tan(abs(self.helixAngle)) * self.width / 2,
                    0,
                    0,
                    self.normalModule, teeth, self.height, self.normalPressureAngle, self.helixAngle,
                    self.backlash, self.addendum,
                    self.dedendum
                ))
                wireBody3, _ = tbm.createWireFromCurves(self.rackLines(
                    -self.length / 2 - (math.tan(abs(self.helixAngle)) + math.tan(self.helixAngle)) * self.width / 2,
                    self.width / 2,
                    0,
                    self.normalModule, teeth, self.height, self.normalPressureAngle, self.helixAngle,
                    self.backlash, self.addendum, self.dedendum
                ))
            else:
                wireBody1, _ = tbm.createWireFromCurves(self.rackLines(
                    -self.length / 2 - (math.tan(abs(self.helixAngle)) + math.tan(self.helixAngle)) * self.width,
                    -self.width / 2,
                    0,
                    self.normalModule, teeth, self.height, self.normalPressureAngle, self.helixAngle,
                    self.backlash, self.addendum, self.dedendum
                ))
                wireBody2, _ = tbm.createWireFromCurves(self.rackLines(
                    -self.length / 2 - math.tan(abs(self.helixAngle)) * self.width,
                    self.width / 2,
                    0,
                    self.normalModule, teeth, self.height, self.normalPressureAngle, self.helixAngle,
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
                if (baseFeature):
                    tools.add(component.bRepBodies.add(b, baseFeature))
                else:
                    tools.add(component.bRepBodies.add(b))
            # Boundary fills enclosed voulume
            boundaryFillInput = component.features.boundaryFillFeatures.createInput(tools,
                                                                                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            if baseFeature:
                boundaryFillInput.targetBaseFeature = baseFeature
            boundaryFillInput.bRepCells.item(0).isSelected = True
            body = component.features.boundaryFillFeatures.add(boundaryFillInput).bodies.item(0)
            # Creates a box to cut off angled ends
            obb = adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(0, 0, 0),
                                                         adsk.core.Vector3D.create(1, 0, 0),
                                                         adsk.core.Vector3D.create(0, 1, 0),
                                                         self.length, self.width * 2, (self.height + self.addendum) * 2)
            box = tbm.createBox(obb)
            tbm.booleanOperation(box, tbm.copy(body), 1)
            if (baseFeature):
                gearBody = component.bRepBodies.add(box, baseFeature)
            else:
                gearBody = component.bRepBodies.add(box)
            body.deleteMe()
            # Deletes tooling bodies
            for b in tools:
                b.deleteMe()

            # Storres a copy of the newly generated gear            
            lastGear = tbm.copy(gearBody)
        else:
            if (baseFeature):
                component.bRepBodies.add(lastGear, baseFeature)
            else:
                component.bRepBodies.add(lastGear)

        # Adds "pitch diameter" line
        pitchDiameterSketch = component.sketches.add(component.xYConstructionPlane)
        pitchDiameterSketch.name = "Pitch Diameter Line"
        pitchDiameterLine = pitchDiameterSketch.sketchCurves.sketchLines.addByTwoPoints(
            adsk.core.Point3D.create(-self.length / 2, 0, 0),
            adsk.core.Point3D.create(self.length / 2, 0, 0)
        )
        pitchDiameterLine.isFixed = True
        pitchDiameterLine.isConstruction = True
        
        if (baseFeature):
            baseFeature.finishEdit()

        # add attributes to component to remember when used in placement
        gear_attributes = occurrence.attributes
        gear_attributes.add('HelicalGear', 'type', "Rack}")
        gear_attributes.add('HelicalGear', 'module', "{0}".format(self.module))
        gear_attributes.add('HelicalGear', 'teeth', "{0}".format(self.toothCount))
        gear_attributes.add('HelicalGear', 'pressureAngle', "{0}".format(self.pressureAngle))
        gear_attributes.add('HelicalGear', 'helixAngle', "{0}".format(self.helixAngle))
        gear_attributes.add('HelicalGear', 'profileShiftCoefficient', "{0}".format(self.modifier.profileShift))
        gear_attributes.add('HelicalGear', 'backlash', "{0}".format(self.backlash))
        gear_attributes.add('HelicalGear', 'herringbone', "{0}".format(self.herringbone))

        return occurrence


# Fires when the CommandDefinition gets executed.
# Responsible for adding commandInputs to the command &
# registering the other command _handlers.
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
            tabBore = inputs.addTabCommandInput("TabBore", "Bore")
            tabAdvanced = inputs.addTabCommandInput("TabAdvanced", "Advanced")
            tabPosition = inputs.addTabCommandInput("TabPosition", "Position")
            tabProperties = inputs.addTabCommandInput("TabProperties", "Info")
            
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

            # Bore commmand inputs
            bvBore = tabBore.children.addBoolValueInput("BVBore", "Bore", True, "", pers['BVBore'])
            bvBore.toolClipFilename = 'resources/captions/Bore Keyway.png'
            bvBore.tooltip = "Center Bore"
            bvBore.tooltipDescription = "Generates the center bore of the gear."

            viBoreDiameter = tabBore.children.addValueInput("VIBoreDiameter", "Bore Diameter", "mm",
                                                         adsk.core.ValueInput.createByReal(pers['VIBoreDiameter']))
            viBoreDiameter.toolClipFilename = 'resources/captions/BoreKey.png'
            viBoreDiameter.tooltip = "Bore Diameter"
            viBoreDiameter.tooltipDescription = "Diameter of the center bore"

            bvKey = tabBore.children.addBoolValueInput("BVKey", "Keyway", True, "", pers['BVKey'])
            bvKey.toolClipFilename = 'resources/captions/Bore Keyway.png'
            bvKey.tooltip = "Key way on the center bore"
            bvKey.tooltipDescription = "Generates the key way on the center bore of the gear."

            viKeyWidth = tabBore.children.addValueInput("VIKeyWidth", "Key Width", "mm",
                                                         adsk.core.ValueInput.createByReal(pers['VIKeyWidth']))
            viKeyWidth.toolClipFilename = 'resources/captions/BoreKey.png'                                             
            viKeyWidth.tooltip = "Keywidth"
            viKeyWidth.tooltipDescription = "Width of the key"

            viKeyHeight = tabBore.children.addValueInput("VIKeyHeight", "Key Height", "mm",
                                                         adsk.core.ValueInput.createByReal(pers['VIKeyHeight']))
            viKeyHeight.toolClipFilename = 'resources/captions/BoreKey.png'                                             
            viKeyHeight.tooltip = "KeyHeight"
            viKeyHeight.tooltipDescription = "Height of the key"

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

            viShift = tabAdvanced.children.addValueInput("VIShift", "Profile Shift", "",
                                                            adsk.core.ValueInput.createByReal(pers['VIShift']))
            viShift.tooltip = "Profile Shift"
            viShift.tooltipDescription = "Represents the profile shift factor."
            

                                                                              
            bvNoUnderCut = tabAdvanced.children.addBoolValueInput("BVNoUnderCut", "NoUnderCut", False, "", pers['BVNoUnderCut'])
            bvNoUnderCut.toolClipFilename = 'resources/captions/Undercut.png'
            bvNoUnderCut.tooltip = "No Under Cut"
            bvNoUnderCut.tooltipDescription = "Calculates the profile shift coefficient to avoid under cut."
            
            # TO DONE png needs to be generated 
            bvCtrCtr = tabAdvanced.children.addBoolValueInput("BVCtrCtr", "Center - Center", True, "", pers['BVCtrCtr'])
            bvCtrCtr.toolClipFilename = 'resources/captions/CtrCtr.png'
            bvCtrCtr.tooltip = "Calculate Center Center"
            bvCtrCtr.tooltipDescription = "Calculate the profile shifts for given Center Center distance."

            viCtrCtr = tabAdvanced.children.addValueInput("VICtrCtr", "Center Center", "mm",
                                                            adsk.core.ValueInput.createByReal(pers['VICtrCtr']))
            viCtrCtr.tooltip = "Center Center"
            viCtrCtr.tooltipDescription = "Center Center distance."
            viCtrCtr.isVisible = pers['BVCtrCtr'] 

            siGear2 = tabAdvanced.children.addSelectionInput("SIGear2", "Gear", "Select mating gear ")
            siGear2.addSelectionFilter("Occurrences")
            siGear2.setSelectionLimits(0, 1)
            siGear2.tooltip = "Mating Gear Selection"
            siGear2.tooltipDescription = "Select a gear that will be used to mesh with the generated gear."
            siGear2.isVisible = pers['BVCtrCtr']

            isTeeth2 = tabAdvanced.children.addIntegerSpinnerCommandInput("ISTeeth2", "Teeth2", 1, 99999, 1,pers['ISTeeth2'])

            isTeeth2.tooltip = "Number of Teeth 2nd gear"
            isTeeth2.tooltipDescription = "The number of teeth the second gear has."
            isTeeth2.isVisible = pers['BVCtrCtr'] 

            viShift2 = tabAdvanced.children.addValueInput("VIShift2", "Profile Shift2", "",
                                                            adsk.core.ValueInput.createByReal(pers['VIShift2']))
            viShift2.tooltip = "Profile Shift for second gear"
            viShift2.tooltipDescription = "Represents the profile shift factor for second gear."
            viShift2.isVisible = pers['BVCtrCtr'] 

            tbWarning2 = tabAdvanced.children.addTextBoxCommandInput("TBWarning2", "", '', 2, True)

            # Position
            #
            # TODO  center center selection
            #   calculate center center if value supplied should be larger than the (z1+z2)*m/2
            #   if profile shift required for no undercut that profile shift coefficient is a lower limit for that gear
            #   shift coefficient should be distributed by the user for the first gear. 
            #   so if no gear selected then user is free to define profile shift distribution over the 2 gears.
            #   if master gear is selected then the profile shift coefficient for that gear is determined the center center distance
            #   determines the second gear
            #   either the user keys in the parameters of the 2 gears and then calculates the master gear
            #   or the master gear is selected and the following gear is calculated
            #
            siGear = tabPosition.children.addSelectionInput("SIGear", "Gear", "Select Meshing Gear ")
            siGear.addSelectionFilter("Occurrences")
            siGear.setSelectionLimits(0, 1)
            siGear.tooltip = "Meshing Gear Selection"
            siGear.tooltipDescription = "Select a gear that will be used to mesh with the generated gear."

            siPlane = tabPosition.children.addSelectionInput("SIPlane", "Plane", "Select Gear Plane")
            siPlane.addSelectionFilter("ConstructionPlanes")
            siPlane.addSelectionFilter("Profiles")
            siPlane.addSelectionFilter("Faces")
            siPlane.setSelectionLimits(0, 1)
            siPlane.tooltip = "Gear Plane"
            siPlane.tooltipDescription = "Select the plane the gear will be placed on.\n\nValid selections are:\n    Sketch Profiles\n    Construction Planes\n    BRep Faces"

            siDirection = tabPosition.children.addSelectionInput("SIDirection", "Line", "Select Rack Direction")
            siDirection.addSelectionFilter("ConstructionLines")
            siDirection.addSelectionFilter("SketchLines")
            siDirection.addSelectionFilter("LinearEdges")
            siDirection.setSelectionLimits(0, 1)
            siDirection.isVisible = False
            siDirection.tooltip = "Rack Path"
            siDirection.tooltipDescription = "Select the line the rack is placed on.\nWill be projected onto the plane.\n\nValid selections are:\n    Sketch Lines\n    Construction Lines\n    BRep Edges"

            siOrigin = tabPosition.children.addSelectionInput("SIOrigin", "Center", "Select Gear Center")
            siOrigin.addSelectionFilter("ConstructionPoints")
            siOrigin.addSelectionFilter("SketchPoints")
            siOrigin.addSelectionFilter("Vertices")
            siOrigin.addSelectionFilter("CircularEdges")
            siOrigin.setSelectionLimits(0, 1)
            siOrigin.tooltip = "Gear Center Point"
            siOrigin.tooltipDescription = "Select the center point of the gear.\nWill be projected onto the plane.\n\nValid selections:\n    Sketch Points\n    Construction Points\n    BRep Vertices\n    Circular BRep Edges\n"

            bvFlipped = tabPosition.children.addBoolValueInput("BVFlipped", "Flip", True)
            bvFlipped.isVisible = False
            bvFlipped.tooltip = "Flips rack direction"

            ddDirection = tabPosition.children.addDropDownCommandInput("DDDirection", "Direction", 0)
            ddDirection.listItems.add("Front", True, "resources/front")
            ddDirection.listItems.add("Center", False, "resources/center")
            ddDirection.listItems.add("Back", False, "resources/back")
            ddDirection.tooltip = "Direction"
            ddDirection.tooltipDescription = "Choose what side of the plane the gear is placed on."

            avRotation = tabPosition.children.addAngleValueCommandInput("AVRotation", "Rotation",
                                                                        adsk.core.ValueInput.createByReal(0))
            avRotation.isVisible = False
            avRotation.tooltip = "Rotation"
            avRotation.tooltipDescription = "Rotates the gear around its axis."

            dvOffsetX = tabPosition.children.addDistanceValueCommandInput("DVOffsetX", "Offset (X)",
                                                                          adsk.core.ValueInput.createByReal(0))
            dvOffsetX.setManipulator(
                adsk.core.Point3D.create(0, 0, 0),
                adsk.core.Vector3D.create(1, 0, 0)
            )
            dvOffsetX.isVisible = False
            dvOffsetX.tooltip = "Offset along path."

            dvOffsetY = tabPosition.children.addDistanceValueCommandInput("DVOffsetY", "Offset (Y)",
                                                                          adsk.core.ValueInput.createByReal(0))
            dvOffsetY.setManipulator(
                adsk.core.Point3D.create(0, 0, 0),
                adsk.core.Vector3D.create(0, 1, 0)
            )
            dvOffsetY.isVisible = False

            dvOffsetZ = tabPosition.children.addDistanceValueCommandInput("DVOffsetZ", "Offset (Z)",
                                                                          adsk.core.ValueInput.createByReal(0))
            dvOffsetZ.setManipulator(
                adsk.core.Point3D.create(0, 0, 0),
                adsk.core.Vector3D.create(0, 0, 1)
            )
            dvOffsetZ.isVisible = False
            dvOffsetZ.tooltip = "Offset from plane"

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
            preserveInputs(args.command.commandInputs, pers)

            gear = generateGear(args.command.commandInputs).modelGear(
                adsk.core.Application.get().activeProduct.rootComponent)

            moveGear(gear, args.command.commandInputs)

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
                preserveInputs(args.command.commandInputs, pers)

                global lastInput

                reuseGear = lastInput in ["APITabBar", "SIPlane", "SIOrigin", "SIDirection", "DDDirection",
                                          "AVRotation", "BVFlipped", "DVOffsetX", "DVOffsetY", "DVOffsetZ", "SIGear","SIGear2"]

                gear = generateGear(args.command.commandInputs).modelGear(
                    adsk.core.Application.get().activeProduct.rootComponent, reuseGear)

                moveGear(gear, args.command.commandInputs)

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
            isInvalid = generateGear(args.inputs).isInvalid
            args.areInputsValid = not isInvalid
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
                args.input.parentCommand.commandInputs.itemById("VIShift").isVisible = gearType != "Rack Gear"
                args.input.parentCommand.commandInputs.itemById("BVNoUnderCut").isVisible = gearType != "Rack Gear"
                #
                # this does not work :( 
                #args.input.parentCommand.commandInputs.itemById("TabBore").isEnabled = gearType == "External Gear"
                args.input.parentCommand.commandInputs.itemById("BVBore").isEnabled = gearType == "External Gear"
                args.input.parentCommand.commandInputs.itemById("VIBoreDiameter").isEnabled = gearType == "External Gear"
                args.input.parentCommand.commandInputs.itemById("BVKey").isEnabled = gearType == "External Gear"
                args.input.parentCommand.commandInputs.itemById("VIKeyWidth").isEnabled = gearType == "External Gear"
                args.input.parentCommand.commandInputs.itemById("VIKeyHeight").isEnabled = gearType == "External Gear"
                if (gearType != "External Gear"):
                    args.input.parentCommand.commandInputs.itemById("BVBore").value = False
                    args.input.parentCommand.commandInputs.itemById("BVKey").value = False
                
                
            # Bore tab
            if ((args.input.id == "BVBore") or (args.input.id == "BVKey")):
                # TO DONE reset borediameter value if not selected
                hasBore = args.inputs.itemById("BVBore").value
                hasKey = args.inputs.itemById("BVKey").value
                args.inputs.itemById("VIBoreDiameter").isVisible = hasBore
                if (not hasBore):
                    args.input.parentCommand.commandInputs.itemById("BVKey").value = False
                    hasKey = False
                args.inputs.itemById("BVKey").isVisible = hasBore
                # TO DONE reset keyHeight value if not selected
                args.inputs.itemById("VIKeyWidth").isVisible = hasBore and hasKey
                args.inputs.itemById("VIKeyHeight").isVisible = hasBore and hasKey
            # Update profile shift when NoUnderCut selected
            if (args.input.id == "BVNoUnderCut"):
                if ( args.inputs.itemById("BVNoUnderCut").value):
                    gear = generateGear(args.input.parentCommand.commandInputs)
                    gear.modifier.profileShift = 0
                    Zcrit = math.floor(gear.critcalVirtualToothCount)
                    # check if teeth count is less or equal then 16 (limit of undercut)
                    if ( gear.toothCount < Zcrit ):
                        args.inputs.itemById("VIShift").value =  1 - (gear.toothCount / Zcrit)
            # Deselect NoUnderCut when profileShift is filled out
            if (args.input.id in ["VIShift", "DDType" , "ISTeeth" , "VIModule" , "VIHelixAngle", "VIPressureAngle"] ):
                args.input.parentCommand.commandInputs.itemById("BVNoUnderCut").value = False
            # Center Center calculation
            if (args.input.id == "BVCtrCtr"):
                ctrCtr = args.inputs.itemById("BVCtrCtr").value
                args.inputs.itemById("ISTeeth2").isVisible = ctrCtr
                args.inputs.itemById("VIShift2").isVisible = ctrCtr
                args.inputs.itemById("VICtrCtr").isVisible = ctrCtr
                args.inputs.itemById("SIGear2").isVisible = ctrCtr
            # Updates Information
            if (args.inputs.itemById("TabProperties") and args.inputs.itemById("TabProperties").isActive):
                gear = generateGear(args.inputs)
                tbProperties = args.inputs.itemById("TBProperties")
                info = str(gear)
                tbProperties.numRows = len(info.split('\n'))
                tbProperties.text = info
            # Updates Warning Message
            if (not args.input.id[:2] == "TB"):
                isInvalid = generateGear(args.input.parentCommand.commandInputs).isInvalid
                if (isInvalid):
                    args.input.parentCommand.commandInputs.itemById(
                        "TBWarning1").formattedText = '<h3><font color="darkred">Error: {0}</font></h3>'.format(
                        isInvalid)
                    args.input.parentCommand.commandInputs.itemById(
                        "TBWarning2").formattedText = '<h3><font color="darkred">Error: {0}</font></h3>'.format(
                        isInvalid)
                else:
                    args.input.parentCommand.commandInputs.itemById("TBWarning1").formattedText = ''
                    args.input.parentCommand.commandInputs.itemById("TBWarning2").formattedText = ''
            # Hides Positioning Manipulators when inactive
            if (args.input.id == "APITabBar"):
                if (args.inputs.itemById("TabPosition") and args.inputs.itemById("TabPosition").isActive):
                    args.input.parentCommand.commandInputs.itemById("SIOrigin").isVisible = True
                    args.input.parentCommand.commandInputs.itemById("SIPlane").isVisible = True
                    args.input.parentCommand.commandInputs.itemById("DVOffsetZ").isVisible = True
                    if (args.input.parentCommand.commandInputs.itemById("DDType").selectedItem.name == "Rack Gear"):
                        args.input.parentCommand.commandInputs.itemById("SIDirection").isVisible = True
                        args.input.parentCommand.commandInputs.itemById("DVOffsetX").isVisible = True
                        args.input.parentCommand.commandInputs.itemById("DVOffsetY").isVisible = True
                        args.input.parentCommand.commandInputs.itemById("BVFlipped").isVisible = True
                    else:
                        args.input.parentCommand.commandInputs.itemById("AVRotation").isVisible = True
                else:

                    args.input.parentCommand.commandInputs.itemById("SIOrigin").isVisible = False
                    args.input.parentCommand.commandInputs.itemById("SIDirection").isVisible = False
                    args.input.parentCommand.commandInputs.itemById("SIPlane").isVisible = False
                    args.input.parentCommand.commandInputs.itemById("DVOffsetX").isVisible = False
                    args.input.parentCommand.commandInputs.itemById("DVOffsetY").isVisible = False
                    args.input.parentCommand.commandInputs.itemById("DVOffsetZ").isVisible = False
                    args.input.parentCommand.commandInputs.itemById("AVRotation").isVisible = False
                    args.input.parentCommand.commandInputs.itemById("BVFlipped").isVisible = False
            if (args.input.id == "SIGear2")                    :
                args.input.parentCommand.commandInputs.itemById("SIGear").clearSelection()
                args.input.parentCommand.commandInputs.itemById("SIGear").addSelection(args.input.selection(0).entity)
                # TO DONE update position
        
            # Update manipulators
            if (args.input.id in ["SIOrigin", "SIDirection", "SIPlane", "AVRotation", "DVOffsetX", "DVOffsetY",
                                  "DVOffsetZ", "BVFlipped", "DDDirection", "DDType", "SIGear", "SIGear2"]):
                if (args.input.parentCommand.commandInputs.itemById("DDType").selectedItem.name != "Rack Gear"):
                    mat = regularMoveMatrix(args.input.parentCommand.commandInputs)

                    # Creates a direction vector aligned to relative Z+
                    d = adsk.core.Vector3D.create(0, 0, 1)
                    d.transformBy(mat)

                    p = mat.translation

                    # Scales vector by Offset to remove offset from manipulator position
                    d0 = d.copy()
                    d0.normalize()
                    d0.scaleBy(args.input.parentCommand.commandInputs.itemById("DVOffsetZ").value)
                    p0 = p.copy()
                    p0.subtract(d0)

                    pln = adsk.core.Plane.create(
                        adsk.core.Point3D.create(0, 0, 0),
                        d
                    )

                    args.input.parentCommand.commandInputs.itemById("DVOffsetZ").setManipulator(p0.asPoint(), d)
                    args.input.parentCommand.commandInputs.itemById("AVRotation").setManipulator(p.asPoint(),
                                                                                                 pln.uDirection,
                                                                                                 pln.vDirection)

                else:
                    mat = rackMoveMatrix(args.input.parentCommand.commandInputs)

                    # Creates a directin vector aligned to relative xyz
                    x = adsk.core.Vector3D.create(1, 0, 0)
                    x.transformBy(mat)
                    y = adsk.core.Vector3D.create(0, 0, 1)
                    y.transformBy(mat)
                    z = adsk.core.Vector3D.create(0, -1, 0)
                    z.transformBy(mat)

                    p = mat.translation

                    # Flippes x when rack is flipped
                    xf = x.copy()
                    if (args.input.parentCommand.commandInputs.itemById("BVFlipped").value):
                        xf.scaleBy(-1)

                    # Creates scaled direction vectors for position compensation
                    x0 = xf.copy()
                    x0.normalize()
                    x0.scaleBy(args.input.parentCommand.commandInputs.itemById("DVOffsetX").value)

                    y0 = y.copy()
                    y0.normalize()
                    y0.scaleBy(args.input.parentCommand.commandInputs.itemById("DVOffsetY").value)

                    z0 = z.copy()
                    z0.normalize()
                    z0.scaleBy(args.input.parentCommand.commandInputs.itemById("DVOffsetZ").value)

                    # Compensates position
                    px = p.copy()
                    px.subtract(x0)

                    py = p.copy()
                    py.subtract(y0)

                    pz = p.copy()
                    pz.subtract(z0)

                    args.input.parentCommand.commandInputs.itemById("DVOffsetX").setManipulator(px.asPoint(), xf)
                    args.input.parentCommand.commandInputs.itemById("DVOffsetY").setManipulator(py.asPoint(), y)
                    args.input.parentCommand.commandInputs.itemById("DVOffsetZ").setManipulator(pz.asPoint(), z)
            
                

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


def preserveInputs(commandInputs, pers):
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
    pers['VIShift'] = commandInputs.itemById("VIShift").value
    pers['BVNoUnderCut'] = commandInputs.itemById("BVNoUnderCut").value
    pers['VICtrCtr'] = commandInputs.itemById("VICtrCtr").value
    pers['VIShift2'] = commandInputs.itemById("VIShift2").value
    pers['ISTeeth2'] = commandInputs.itemById("ISTeeth2").value
    pers['VIBoreDiameter'] = commandInputs.itemById("VIBoreDiameter").value
    pers['VIKeyWidth'] = commandInputs.itemById("VIKeyWidth").value
    pers['VIKeyHeight'] = commandInputs.itemById("VIKeyHeight").value




def generateGear(commandInputs):
    gearType = commandInputs.itemById("DDType").selectedItem.name
    standard = commandInputs.itemById("DDStandard").selectedItem.name

    if (gearType == "Rack Gear"):
        if (standard == "Normal"):
            gear = RackGear.createInNormalSystem(
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
            gear = RackGear.createInRadialSystem(
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
            teeth1 = commandInputs.itemById("ISTeeth").value
            teeth2 = commandInputs.itemById("ISTeeth2").value
            ctrctr = commandInputs.itemById("VICtrCtr").value
            shift1 = commandInputs.itemById("VIShift").value
            shift2 = commandInputs.itemById("VIShift2").value
            module = commandInputs.itemById("VIModule").value   
            press  = commandInputs.itemById("VIPressureAngle").value
            gearModifier = GearModifier( shift1,teeth2, shift2,ctrctr)
            bore = commandInputs.itemById("VIBoreDiameter").value if commandInputs.itemById("BVBore").value else 0,
            height = commandInputs.itemById("VIKeyHeight").value if commandInputs.itemById("BVKey").value else 0,
            width =commandInputs.itemById("VIKeyWidth").value if commandInputs.itemById("BVKey").value else 0
            if (commandInputs.itemById("BVCtrCtr").value):              # we have a center center calculation
                # check if all input is available
                # TODO do the same for the helical gear and herringbone
                if (ctrctr > 0 and teeth2 > 0):
                    halfTotalTeeth = (teeth1 + teeth2)/2
                    centerModificationCoefficient = ctrctr/module - halfTotalTeeth
                    workingPressureAngle = math.acos( math.cos(press)/(centerModificationCoefficient/halfTotalTeeth +1))
                    sumshift = halfTotalTeeth*(Involute.Involute(workingPressureAngle) - Involute.Involute(press))/math.tan(press)
                    
                    if (shift1 != 0):
                        shift2 = sumshift - shift1
                    else:
                        if (shift2 != 0):
                            shift1 = sumshift - shift2
                        else:
                            shift1 = sumshift / 2
                            shift2 = shift1
            else: 
                centerModificationCoefficient = 0

            if (standard == "Normal"):
                gear = HelicalGear.createInNormalSystem(
                    teeth1,                                             #toothCount
                    module,                                             #normalModule
                    press,                                              #normalPressureAngle
                    commandInputs.itemById("VIHelixAngle").value,       #helixAngle
                    commandInputs.itemById("VIBacklash").value,         #backlash
                    commandInputs.itemById("VIAddendum").value,         #addendum
                    commandInputs.itemById("VIDedendum").value,         #dedendum
                    commandInputs.itemById("VIWidth").value,            #width
                    commandInputs.itemById("BVHerringbone").value,      #herringbone
                    0,                                                  #internalOutsideDiameter
                    #teeth2,                                             #toothcount2
                    #shift2,                                              #profileshift2
                    #centerModificationCoefficient,
                    GearModifier(shift1,teeth2,shift2,ctrctr),
                    GearMount(
                        commandInputs.itemById("VIBoreDiameter").value if commandInputs.itemById("BVBore").value else 0,
                        commandInputs.itemById("VIKeyHeight").value if commandInputs.itemById("BVKey").value else 0,
                        commandInputs.itemById("VIKeyWidth").value if commandInputs.itemById("BVKey").value else 0)
                )
            else:
                gear = HelicalGear.createInRadialSystem(
                    commandInputs.itemById("ISTeeth").value,
                    commandInputs.itemById("VIModule").value,
                    commandInputs.itemById("VIPressureAngle").value,
                    commandInputs.itemById("VIHelixAngle").value,
                    commandInputs.itemById("VIBacklash").value,
                    commandInputs.itemById("VIAddendum").value,
                    commandInputs.itemById("VIDedendum").value,
                    commandInputs.itemById("VIWidth").value,
                    commandInputs.itemById("BVHerringbone").value,      #herringbone
                    0,                                                  #internalOutsideDiameter
                    #commandInputs.itemById("ISTeeth2").value,           #toothcount2
                    #commandInputs.itemById("VIShift2").value,           #profileshift2
                    #centerModificationCoefficient,
                    GearModifier(shift1,teeth2,shift2,ctrctr),
                    GearMount(
                        commandInputs.itembyId("VIBoreDiameter").value if commandInputs.itemById("BVBore").value else 0,
                        commandInputs.itembyId("VIKeyHeight").value if commandInputs.itemById("BVKey").value else 0,
                        commandInputs.itembyId("VIKeyWidth").value if commandInputs.itemById("BVKey").value else 0)
                )
        else:
            if (standard == "Normal"):
                gear = HelicalGear.createInNormalSystem(
                    commandInputs.itemById("ISTeeth").value,
                    commandInputs.itemById("VIModule").value,
                    commandInputs.itemById("VIPressureAngle").value,
                    commandInputs.itemById("VIHelixAngle").value,
                    -commandInputs.itemById("VIBacklash").value,
                    commandInputs.itemById("VIDedendum").value,
                    commandInputs.itemById("VIAddendum").value,
                    commandInputs.itemById("VIWidth").value,
                    commandInputs.itemById("BVHerringbone").value,
                    commandInputs.itemById("VIDiameter").value,      
                    GearModifier(shift1,teeth2,shift2,ctrctr)
                )
            else:
                gear = HelicalGear.createInRadialSystem(
                    commandInputs.itemById("ISTeeth").value,
                    commandInputs.itemById("VIModule").value,
                    commandInputs.itemById("VIPressureAngle").value,
                    commandInputs.itemById("VIHelixAngle").value,
                    commandInputs.itemById("VIBacklash").value,
                    commandInputs.itemById("VIDedendum").value,
                    commandInputs.itemById("VIAddendum").value,
                    commandInputs.itemById("VIWidth").value,
                    commandInputs.itemById("BVHerringbone").value,
                    commandInputs.itemById("VIDiameter").value,      
                    GearModifier(shift1,teeth2,shift2,ctrctr)
                )
    return gear


def moveGear(gear, commandInputs):
    if (commandInputs.itemById("DDType").selectedItem.name != "Rack Gear"):
        modeltransform = regularMoveMatrix(commandInputs)
    else:
        modeltransform = rackMoveMatrix(commandInputs)
    # do we have to mesh with another gear ?
    if (commandInputs.itemById("SIGear").selectionCount):
        selection = commandInputs.itemById("SIGear").selection(0).entity
        
        attGearType = selection.attributes.itemByName('HelicalGear','type')
        if attGearType != None:
            modeltransform = meshGears(gear,commandInputs,modeltransform)
		
    gear.transform = modeltransform
    # Applies the movement in parametric design mode
    if (adsk.core.Application.get().activeDocument.design.designType):
        adsk.core.Application.get().activeDocument.design.snapshots.add()

def printRotationArray(msg,arr):
    print (msg)
    for i in range(4):
        for j in range(4):
            print ('{:1.4f} '.format(arr[i*4 + j]),end="")
        print (' ')

def meshGears(gear,commandInputs,transform):
    # we have selected another gear get the origin and translation matrix
    selection = commandInputs.itemById("SIGear").selection(0).entity
    selectionTransform = selection.transform
    asMat = selectionTransform.asArray()
    mstrPitchDiameter = float(selection.attributes.itemByName('HelicalGear','pitchDiameter').value)
    mstrToothCount = float(selection.attributes.itemByName('HelicalGear','teeth').value)
    mstrShift = float(selection.attributes.itemByName('HelicalGear','profileShiftCoefficient').value)
    
    gearModul = float(gear.attributes.itemByName('HelicalGear','module').value)
    gearPressureAngle = float(gear.attributes.itemByName('HelicalGear','pressureAngle').value)
    gearShift = float(gear.attributes.itemByName('HelicalGear','profileShiftCoefficient').value)
    gearPitchDiameter = float( gear.attributes.itemByName('HelicalGear','pitchDiameter').value)
    gearToothCount = int(gear.attributes.itemByName('HelicalGear','teeth').value)
    # from the gear retrieve the origin point and origin plane should all be in its transform.
    # the calculated transform already move the center of the new gear to the selected gear center
    # TODO  - recalc gear offset based on profile shifting
    #               a = m ( z1 + z2 ) * cos(A0)/cos(Ab)
    #               inv(Ab) = 2 (x1 +x2)/(z1 + z2)*tan(A0) + inv(A0)
    #               inv(x) = tan(x) - x
    # 
    #   given center - center distance Ax ==> distance increment factor = Ax/m - (z1 + z2)/2
    #         working pressure angle = acos((z1+z2)*cos(a)/(2y+z1+z2))
    #
    # DONE  - rotational input is rotation around selected gear center
    #           - take into account gear ratio : angle * gearratio (= master / new) --> rotational angle
    #               deltaX : offset * cos(angle)
    #               deltaY : offset * sin(angle)
    # DONE  - rotation of the gear around its center has to mesh with the selected gear
    #           - if toothcount is even rotate around the gear center by 360/toothcount ???? is this correct  3rd gear with even toothcount ????
    #           - take into account the rotation of the master gear and the gear ratio

    diff = Involute.InverseInvolute(Involute.Involute(gearPressureAngle)) - gearPressureAngle
    involuteWorking = (2*(mstrShift+gearShift)/(mstrToothCount+gearToothCount)*math.tan(gearPressureAngle) + Involute.Involute(gearPressureAngle))
    workingAngle = Involute.InverseInvolute(2*(mstrShift+gearShift)/(mstrToothCount+gearToothCount)*math.tan(gearPressureAngle) + Involute.Involute(gearPressureAngle))
    print('working pressure  angle  {0:1.6f} ( {1:1.4f}°)'.format(workingAngle, math.degrees(workingAngle)))

    centerDistanceCorrectionFactor = (mstrToothCount + gearToothCount)/2*(math.cos(gearPressureAngle)/math.cos(workingAngle) -1)

    gearOffset = (gearPitchDiameter + mstrPitchDiameter)/2
    
    gearOffset = ((mstrToothCount + gearToothCount)/2 + centerDistanceCorrectionFactor )*gearModul

    baserotation = adsk.core.Matrix3D.create()
    origin = adsk.core.Point3D.create(0, 0, 0)
    zaxis =  adsk.core.Vector3D.create(0, 0, 1)
    angle = 0
    mstrAngle = 0
    if (gearToothCount % 2) == 0 :
        angle = math.pi / gearToothCount
    # compensate for master tooth count angle
    if (mstrToothCount % 2) == 0 :
        mstrAngle = (-math.pi / mstrToothCount)
    mstrRelAngle = math.acos(selectionTransform.getCell(0,0))        
    
    baserotation.setToRotation(angle + ( mstrRelAngle) *  mstrPitchDiameter / gearPitchDiameter ,zaxis,origin) #rotate to mesh gear with even teeth
    basearray = baserotation.asArray()
    print('meshing angle Gear   {0:1.6f} ( {1:1.4f}°)'.format(angle, math.degrees(angle)))
    print('meshing angle Master {0:1.6f} ( {1:1.4f}°)'.format(mstrAngle, math.degrees(mstrAngle)))
    print('meshing angle Total  {0:1.6f} ( {1:1.4f}°)'.format(angle + mstrAngle *  mstrPitchDiameter / gearPitchDiameter , math.degrees(angle + mstrAngle *  mstrPitchDiameter / gearPitchDiameter )))
    printRotationArray("baserotation",basearray)

    # move to mastergear position and orientation
    (mstrOrigin, mstrXAxis, mstrYAxis, mstrZAxis) = selectionTransform.getAsCoordinateSystem()
    mat = adsk.core.Matrix3D.create()
    mat.translation = selectionTransform.translation
    #baserotation.transformBy(selectionTransform)
    baserotation.transformBy(mat)
    basearray = baserotation.asArray()
    printRotationArray("selection transform applied ",basearray)

    angle = commandInputs.itemById("AVRotation").value
    #angle = math.acos(selectionTransform.getCell(0,0))
    print('selected rotation angle {0:1.6f} ( {1:1.4f}°)'.format(angle,math.degrees(angle)))
    basearray = baserotation.asArray()

    rotation = angle *  mstrPitchDiameter / gearPitchDiameter 
    print('relative rotation angle {0:1.6f} ( {1:1.4f}°)'.format(rotation,math.degrees(rotation)))
    mat = adsk.core.Matrix3D.create()
    mat.setToRotation(rotation, zaxis,mstrOrigin) # rotate the gear around the zaxis to compensate the relative rotation around the master gear
    baserotation.transformBy(mat)
    basearray = baserotation.asArray()
    printRotationArray("teeth align",basearray)

    # position new gear meshed with master gear along the X-axis
    mat = adsk.core.Matrix3D.create()
    mat.translation = adsk.core.Vector3D.create(gearOffset,0,0)
    baserotation.transformBy(mat)
    basearray = baserotation.asArray()
    printRotationArray("move horizontal to mesh",basearray)

    # rotate the new gear around the mastergear 
    printRotationArray('selection transform ', selectionTransform.asArray())
    (mstrOrigin, mstrXAxis, mstrYAxis, mstrZAxis) = selectionTransform.getAsCoordinateSystem()
    mat = adsk.core.Matrix3D.create()
    mat.setToRotation(angle,mstrZAxis,mstrOrigin)
    
    # deltaX = gearOffset * math.cos(angle)
    # deltaY = gearOffset * math.sin(angle)
    # transform.setCell(0, 3, transform.getCell(0, 3) + deltaX) # + selection.transform.getCell(0,3)) #x-component
    # transform.setCell(1, 3, transform.getCell(1, 3) + deltaY) # + selection.transform.getCell(1,3)) #y-component
    
    baserotation.transformBy(mat)
    basearray = baserotation.asArray()
    printRotationArray("final transform",basearray)
    return baserotation


def regularMoveMatrix(commandInputs):
    # DDDirection is 0,1,2 front,center,back --> sideOffset 0.5;0.0;-0.5*VIWidth
    sideOffset = (0.5 - (commandInputs.itemById("DDDirection").selectedItem.index * 0.5)) * commandInputs.itemById(
        "VIWidth").value
    # is a gear selected ?
    if (commandInputs.itemById("SIGear").selectionCount):
        selection = commandInputs.itemById("SIGear").selection(0).entity
        attGearType = selection.attributes.itemByName('HelicalGear','type')
        if attGearType != None:
            # we have selected another gear get the origin and translation matrix
            print ("get selected gear transform")
            return  selection.transform

    if (commandInputs.itemById("SIOrigin").selectionCount):
        point = commandInputs.itemById("SIOrigin").selection(0).entity
        pointPrim = getPrimitiveFromSelection(point)

        # Both Plane and Origin selected, regular move
        if (commandInputs.itemById("SIPlane").selectionCount):
            plane = commandInputs.itemById("SIPlane").selection(0).entity
            planePrim = getPrimitiveFromSelection(plane)

        # Just sketch point selected, use sketch plane as plane
        elif (point.objectType == "adsk::fusion::SketchPoint"):
            planePrim = adsk.core.Plane.createUsingDirections(
                point.parentSketch.origin,
                point.parentSketch.xDirection,
                point.parentSketch.yDirection
            )

        # No useable plane selected --> default X-Y plane
        else:
            planePrim = adsk.core.Plane.createUsingDirections(
                pointPrim,
                adsk.core.Vector3D.create(1, 0, 0),
                adsk.core.Vector3D.create(0, 1, 0)
            )

        print ("tranform to Z-offset and selected rotation")
        return moveMatrixPdro(
            projectPointOnPlane(pointPrim, planePrim),
            planePrim.normal,
            commandInputs.itemById("AVRotation").value,
            commandInputs.itemById("DVOffsetZ").value + sideOffset
        )
    else:
        # No valid selection combination, no move just side & rotation
        return moveMatrixPdro(
            adsk.core.Point3D.create(0, 0, 0),
            adsk.core.Vector3D.create(0, 0, 1),
            commandInputs.itemById("AVRotation").value,
            commandInputs.itemById("DVOffsetZ").value + sideOffset
        )


def rackMoveMatrix(commandInputs):
    sideOffset = (0.5 - (commandInputs.itemById("DDDirection").selectedItem.index * 0.5)) * commandInputs.itemById(
        "VIWidth").value

    if (commandInputs.itemById("SIDirection").selectionCount):
        # Line selected
        line = commandInputs.itemById("SIDirection").selection(0).entity
        linePrim = getPrimitiveFromSelection(line)

        if (commandInputs.itemById("SIPlane").selectionCount):
            # Plane selected
            plane = commandInputs.itemById("SIPlane").selection(0).entity
            planePrim = getPrimitiveFromSelection(plane)
        elif (line.objectType == "adsk::fusion::SketchLine"):
            # No Plane selected, using sketch plane
            planePrim = adsk.core.Plane.createUsingDirections(
                line.parentSketch.origin,
                line.parentSketch.xDirection,
                line.parentSketch.yDirection
            )
        else:
            # Do no move
            planePrim = adsk.core.Plane.create(
                adsk.core.Point3D.create(0, 0, 0),
                adsk.core.Vector3D.create(0, 0, 1)
            )

        if (commandInputs.itemById("SIOrigin").selectionCount):
            # Point selected
            point = commandInputs.itemById("SIOrigin").selection(0).entity
            pointPrim = getPrimitiveFromSelection(point)

        elif (line.objectType == "adsk::fusion::SketchLine"):
            a = line.worldGeometry.startPoint.copy()
            b = line.worldGeometry.endPoint

            v = a.vectorTo(b)
            v.scaleBy(0.5)

            a.translateBy(v)

            pointPrim = a
        else:
            # Do no move
            pointPrim = adsk.core.Point3D.create(0, 0, 0)


    else:
        # No valid selection, no move, just offsets
        linePrim = adsk.core.InfiniteLine3D.create(
            adsk.core.Point3D.create(0, 0, 0),
            adsk.core.Vector3D.create(1, 0, 0)
        )
        planePrim = adsk.core.Plane.create(
            adsk.core.Point3D.create(0, 0, 0),
            adsk.core.Vector3D.create(0, 0, 1)
        )
        pointPrim = adsk.core.Point3D.create(0, 0, 0)

    return moveMatrixPxzfxyz(
        projectPointOnLine(pointPrim, projectLineOnPlane(linePrim, planePrim)),
        projectVectorOnPlane(linePrim.direction, planePrim),
        planePrim.normal,
        commandInputs.itemById("BVFlipped").value,
        commandInputs.itemById("DVOffsetX").value,
        commandInputs.itemById("DVOffsetY").value,
        commandInputs.itemById("DVOffsetZ").value + sideOffset
    )


def moveMatrixPdro(position, direction, rotation, offset):
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


def moveMatrixPxzfxyz(position, x, z, flip, offsetX, offsetY, offsetZ):
    x.normalize()
    z.normalize()

    mat = adsk.core.Matrix3D.create()

    # Flip Z so results line up with regular gears
    z.scaleBy(-1)

    if (flip):
        x.scaleBy(-1)
        offsetX *= -1

    p = adsk.core.Plane.createUsingDirections(adsk.core.Point3D.create(0, 0, 0), z, x)

    # Z & Y flipped due to racks beining generated out of plane
    mat.setToAlignCoordinateSystems(
        adsk.core.Point3D.create(-offsetX, offsetZ, -offsetY),
        adsk.core.Vector3D.create(1, 0, 0),
        adsk.core.Vector3D.create(0, 0, -1),
        adsk.core.Vector3D.create(0, 1, 0),
        position,
        x,
        p.normal,
        z
    )

    return mat


def getPrimitiveFromSelection(selection):
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

    # BRepEdge
    if selection.objectType == "adsk::fusion::BRepEdge":
        # Linear edge
        if (selection.geometry.objectType == "adsk::core::Line3D"):
            _, tangent = selection.evaluator.getTangent(0)
            return adsk.core.InfiniteLine3D.create(
                selection.pointOnEdge,
                tangent
            )
        # Circular edge
        if (selection.geometry.objectType in ["adsk::core::Circle3D", "adsk::core::Arc3D"]):
            return selection.geometry.center

    # Sketch Line
    if selection.objectType == "adsk::fusion::SketchLine":
        return selection.worldGeometry.asInfiniteLine()

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


def projectPointOnPlane(point, plane):
    originToPoint = plane.origin.vectorTo(point)

    normal = plane.normal.copy()
    normal.normalize()
    distPtToPln = normal.dotProduct(originToPoint)

    normal.scaleBy(-distPtToPln)

    ptOnPln = point.copy()
    ptOnPln.translateBy(normal)

    return ptOnPln


def projectVectorOnPlane(vector, plane):
    normal = plane.normal.copy()
    normal.normalize()
    normal.scaleBy(normal.dotProduct(vector))

    vOnPln = vector.copy()
    vOnPln.subtract(normal)

    return vOnPln


def projectLineOnPlane(line, plane):
    return adsk.core.InfiniteLine3D.create(
        projectPointOnPlane(line.origin, plane),
        projectVectorOnPlane(line.direction, plane)
    )


def projectPointOnLine(point, line):
    tangent = line.direction.copy()
    tangent.normalize()

    d = line.origin.vectorTo(point).dotProduct(tangent)
    tangent.scaleBy(d)

    ptOnLn = line.origin.copy()
    ptOnLn.translateBy(tangent)

    return ptOnLn


def run(context):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        commandDefinitions = ui.commandDefinitions
        # check the command exists or not
        cmdDef = commandDefinitions.itemById(COMMANDID)
        if not cmdDef:
            cmdDef = commandDefinitions.addButtonDefinition(COMMANDID, COMMANDNAME,
                                                            COMMANDTOOLTIP, 'resources')
            cmdDef.tooltip = "Generates external, inrernal & rack gears of any helix angle.\nThis includes regular sput gears as well as worm gears."
            cmdDef.toolClipFilename = 'resources/captions/Gears.png'
        # Adds the commandDefinition to the toolbar
        for panel in TOOLBARPANELS:
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
        for panel in TOOLBARPANELS:
            p = ui.allToolbarPanels.itemById(panel).controls.itemById(COMMANDID)
            if p:
                p.deleteMe()

        # Deletes the commandDefinition
        ui.commandDefinitions.itemById(COMMANDID).deleteMe()
    except:
        print(traceback.format_exc())

#encoding:UTF-8
import numpy as np
import bsplines

# Knot Sequence Update Strategies need two method:
# 1) generateKnotList: which generates a new knotlist given the current spline and reprojection errors
#    Returns a boolean flag if the updated knot sequence needs another step of optimization
# 2) getUpdatedSpline: given a knot list, spline order and spline generates a new spline initialized with the
#    values of the given spline and the given knot sequence.
class ReprojectionErrorKnotSequenceUpdateStrategy(object):

    __previousKnotSequence = None

    __previousErrorTerms = None

    __framerate = None

    __maxKnotsPerSecond = None

    __disabledTimeSegments = []

    def __init__(self, framerate):
        self.__framerate = framerate
        self.__maxKnotsPerSecond = 1/(2*self.__framerate)

    def generateKnotList(self, reprojection_errors, poseSpline):
        [times, errors] = self.__getErrorAndTimestamp(reprojection_errors)

        # take a copy of the old knots:
        knots = poseSpline.knots()

        [errorTermsPerSegment, errorPerSegment] = self.__getErrorPerSegment(times, errors, knots)

        disabledTimeSegments = self.__removeSegmentsWithoutImprovement(times, errors, self.__disabledTimeSegments)

        [filteredKnots, disabledTimeSegments] = self.__removeSegmentsWithoutObservations(knots, errorPerSegment, disabledTimeSegments)

        [errorTermsPerSegmentFiltered, errorPerSegmentFiltered] = self.__getErrorPerSegment(times, errors, filteredKnots)

        updatedKnots = self.__generateKnotSequence(errorPerSegmentFiltered, errorTermsPerSegmentFiltered, knots, disabledTimeSegments)

        if (self.__previousKnotSequence is None):
            requiresUpdate = True
        else:
            # require at least a 1% increase in knots for a next iteration being worth the effort
            requiresUpdate = (len(updatedKnots) > 1.01*len(self.__previousKnotSequence))

        # keep a copy of the knot sequence
        self.__previousKnotSequence = np.copy(updatedKnots)
        self.__previousErrorTerms = errorPerSegmentFiltered
        self.__disabledTimeSegments = disabledTimeSegments

        return [updatedKnots, requiresUpdate]

    def getUpdatedSpline(self, poseSpline, knots, splineOrder):
        """Get a spline with the new knot sequence build upon the poses of the old spline"""

        # linearly sample the old spline
        times = np.linspace(poseSpline.t_min(), poseSpline.t_max(), len(knots))

        splinePoses = np.zeros((6, len(knots)))
        for i, time in enumerate(times):
            splinePoses[:,i] = poseSpline.eval(time)

        # guarantee that beginning and end times of the spline remain unchanged
        oldKnots = poseSpline.knots()

        i = 0
        while oldKnots[i] < knots[0]:
            i += 1
        knots = np.insert(knots, 0, oldKnots[0:i])

        i = -1
        while oldKnots[i] > knots[-1]:
            i -= 1
        knots = np.append(knots, oldKnots[i:])

        newPoseSpline = bsplines.BSplinePose(splineOrder, poseSpline.rotation())
        newPoseSpline.initPoseSplineSparseKnots(times, splinePoses, np.array(knots), 1e-6)

        return newPoseSpline


    def __getErrorAndTimestamp(self,reprojection_errors):
        """Extract the timestamps and reprojection error values"""
        errors = []
        times = []
        for reprojection_error in reprojection_errors:
            times.append(reprojection_error.observationTime())
            errors.append(reprojection_error.evaluateError())

        # it is not guaranteed that the errors are sorted in tiem
        newIdx = sorted(range(len(times)),key=times.__getitem__)
        times = [ times[i] for i in newIdx]
        errors = [ errors[i] for i in newIdx]

        return [times, errors]

    def __getErrorPerSegment(self,times, errors, knots):
        """Get the total error per segment and number of error terms per segment"""
        errorPerSegment = np.zeros(len(knots))
        errorTermsPerSegment = np.zeros(len(knots))
        segment = (-1,-1)
        # analyse each section of the knot sequence:
        for i, t in enumerate(times):
            segment = self.__time2KnotSection(t, knots, segment)
            errorPerSegment[segment[0]] += errors[i]
            errorTermsPerSegment[segment[0]] += 1

        return [errorTermsPerSegment, errorPerSegment]

    def __removeSegmentsWithoutObservations(self, knots, errorPerSegment, disabledTimeSegments = []):
        filteredKnots = []

        # remove segments with consecutive 0-valued errors
        p_error = 0
        for i, error in enumerate(errorPerSegment):
            if p_error == 0 and error == 0 and i > 6 and i < len(errorPerSegment) - 6:  # this should depend on the splineOrder!
                # add the segment between the previous and the next knot the the "blacklist"
                disabledTimeSegments.append((knots[i-1],knots[i+1]))
            else:
                filteredKnots.append(knots[i])
            p_error = error

        return [filteredKnots, disabledTimeSegments]

    def __generateKnotSequence(self, errorPerSegmentFiltered, errorTermsPerSegmentFiltered, knots, disabledTimeSegments):
        newKnots = []
        numberOfKnots = len(knots)
        for i, error in enumerate(errorPerSegmentFiltered):
            expectedNormalError = errorTermsPerSegmentFiltered[i]
            if error > expectedNormalError and i < numberOfKnots-1:
                newKnot = (knots[i] + knots[i+1])/2.0
                deltaT = newKnot - knots[i]
                # max number of knots per second hit: do not split
                if deltaT <= self.__maxKnotsPerSecond:
                    newKnots.append(knots[i])
                # segment is disabled: do not split
                elif (disabledTimeSegments is not None and self.__isSegmentDisabled(disabledTimeSegments, newKnot)):
                    newKnots.append(knots[i])
                else:
                    # split:
                    newKnots.append(knots[i])
                    newKnots.append(newKnot)
            else:
                newKnots.append(knots[i])

        return newKnots

    def __time2KnotSection(self, t, knots, segment):
        i = segment[0] # we assume that the times are ordered thus we do not need to check before the previous segment
        if i == -1:
            i = 0
        while(i < len(knots)-1):
            if knots[i] < t and knots[i+1] > t:
                return (i, i+1)
            i+=1
        return (-1,-1)

    # true: disabled
    # false: not disabled
    def __isSegmentDisabled(self, disabledTimeSegments, t):
        for seg in disabledTimeSegments:
            if t < seg[1] and t >= seg[0]:
                return True
        return False

    def __removeSegmentsWithoutImprovement(self, times, errors, disabledTimeSegments):

        ## first compare the reprojection error in the "previous" segments
        # if we do not observe a significant drop. stop adding errors!
        if self.__previousKnotSequence is not None and self.__previousErrorTerms is not None:
            timeSegments = []
            errorPerOldSegment = np.zeros(len(self.__previousKnotSequence))
            segment = (-1,-1)
            # analyse each section of the knot sequence:
            # this kind of inefficient as it adds the same time segment multiple times...
            for i, t in enumerate(times):
                segment = self.__time2KnotSection(t, self.__previousKnotSequence, segment)
                errorPerOldSegment[segment[0]] += errors[i]
                timeSegments.append((self.__previousKnotSequence[segment[0]], self.__previousKnotSequence[segment[1]]))

            # disabledTimeSegments = []

            for i, pe in enumerate(self.__previousErrorTerms):
                if pe * 0.8 < errorPerOldSegment[i]:
                    disabledTimeSegments.append(timeSegments[i])
                    pass

        return disabledTimeSegments

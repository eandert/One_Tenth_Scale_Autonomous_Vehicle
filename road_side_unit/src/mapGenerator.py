import math


class MapSpecs():
    def __init__(self):
        # Intersection Params
        self.intersectionStraightLength = 1.0
        self.intersectionWidth = .5
        self.centerX = 500
        self.centerY = 500
        self.meters_to_print_scale = 100.0

        # Parameters for the trajectory point generation
        self.distanceInterval = .1

        # Simulation/Real Params
        self.isSimulation = 1
        self.lightTime = 0
        self.lightTimePeriod = 8 * 5  # 5 seconds * 8 hz

        # Generating a figure 8 for now TODO: incorporate SUMO/ATLAS
        self.xCoordinates, self.yCoordinates, self.vCoordinates = self.generateFigureEight(
            self.intersectionStraightLength, self.intersectionWidth, self.distanceInterval)

    def generateFigureEight(self, intersectionStraightLength, intersectionWidth, distanceInterval):

        # Generating our coordinates to follow
        xCoordinates = []
        yCoordinates = []
        vCoordinates = []

        # Set up the target intervals
        targetRadius = intersectionStraightLength + (intersectionWidth / 2)
        targetArcLength = distanceInterval / targetRadius

        # Generate the coordintes in a line first
        xCurrent = - intersectionStraightLength - (intersectionWidth / 2)
        yCurrent = 0

        xCoordinates.append(xCurrent)
        yCoordinates.append(yCurrent)
        if abs(xCurrent) <= (intersectionWidth / 2):
            # Add this to light group 1
            vCoordinates.append(1)
        else:
            # No light group, no control
            vCoordinates.append(0)

        while True:
            xCurrent = xCurrent + distanceInterval
            if xCurrent < (intersectionStraightLength + (intersectionWidth / 2)):
                xCoordinates.append(xCurrent)
                yCoordinates.append(yCurrent)
                if abs(xCurrent) <= (intersectionWidth / 2):
                    # Add this to light group 1
                    vCoordinates.append(1)
                else:
                    # No light group, no control
                    vCoordinates.append(0)
            else:
                break

        remaining = (intersectionStraightLength + (intersectionWidth / 2)) - xCurrent
        # print ( " Remaining: ", remaining )
        # print ( " Distance Interval: " , distanceInterval )
        startL = distanceInterval - remaining
        # print ( " StartL: ", startL )
        thetaCurrent = math.radians(startL / targetRadius) + math.radians(270)
        # print ( " Current Theta: ", thetaCurrent )
        # print ( " Target Radius: ", targetRadius )
        xCurrent = targetRadius * math.cos(thetaCurrent) + intersectionStraightLength + (intersectionWidth / 2)
        YCurrent = targetRadius * math.sin(thetaCurrent) + intersectionStraightLength + (intersectionWidth / 2)

        xCoordinates.append(xCurrent)
        yCoordinates.append(yCurrent)
        vCoordinates.append(0)

        while True:
            thetaCurrent = thetaCurrent + targetArcLength
            if thetaCurrent > math.radians(540):
                break
            xCurrent = targetRadius * math.cos(thetaCurrent) + intersectionStraightLength + (intersectionWidth / 2)
            yCurrent = targetRadius * math.sin(thetaCurrent) + intersectionStraightLength + (intersectionWidth / 2)
            xCoordinates.append(xCurrent)
            yCoordinates.append(yCurrent)
            # No light group, no control
            vCoordinates.append(0)

        # Generate the coordintes in a line second
        yCurrent = intersectionStraightLength + (intersectionWidth / 2)
        xCurrent = 0

        xCoordinates.append(xCurrent)
        yCoordinates.append(yCurrent)
        if abs(yCurrent) <= (intersectionWidth / 2):
            # Add this to light group 1
            vCoordinates.append(2)
        else:
            # No light group, no control
            vCoordinates.append(0)

        while True:
            yCurrent = yCurrent - distanceInterval
            if yCurrent >= (-intersectionStraightLength - (intersectionWidth / 2)):
                xCoordinates.append(xCurrent)
                yCoordinates.append(yCurrent)
                if abs(yCurrent) <= (intersectionWidth / 2):
                    # Add this to light group 1
                    vCoordinates.append(2)
                else:
                    # No light group, no control
                    vCoordinates.append(0)
            else:
                break

        # print ( " X CUrrent: ", xCurrent )
        remaining = - (intersectionStraightLength - (intersectionWidth / 2)) + yCurrent
        # print ( " Remaining: ", remaining )
        # print ( " Distance Interval: " , distanceInterval )
        startL = distanceInterval - remaining
        # print ( " StartL: ", startL )
        thetaCurrent = - math.radians(startL / targetRadius)
        # print ( " Current Theta: ", thetaCurrent )
        # print ( " Target Radius: ", targetRadius )
        xCurrent = targetRadius * math.cos(thetaCurrent) - intersectionStraightLength - (intersectionWidth / 2)
        yCurrent = targetRadius * math.sin(thetaCurrent) - intersectionStraightLength - (intersectionWidth / 2)

        xCoordinates.append(xCurrent)
        yCoordinates.append(yCurrent)
        vCoordinates.append(0)

        while True:
            thetaCurrent = thetaCurrent - targetArcLength
            if thetaCurrent < math.radians(-270):
                break
            xCurrent = targetRadius * math.cos(thetaCurrent) - intersectionStraightLength - (intersectionWidth / 2)
            yCurrent = targetRadius * math.sin(thetaCurrent) - intersectionStraightLength - (intersectionWidth / 2)
            xCoordinates.append(xCurrent)
            yCoordinates.append(yCurrent)
            # No light group, no control
            vCoordinates.append(0)

        return xCoordinates, yCoordinates, vCoordinates
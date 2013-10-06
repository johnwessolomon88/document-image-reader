import cv2
import numpy as np
import math
import os
import copy

from PIL import Image

from scipy import stats
from scipy import ndimage

import GeometryFunctions as GEO

def GetVerticeListFromContour(contour):
    vList = []
    for v in contour:
        vList.append([v[0][0], v[0][1]])
    return vList

def rotateImage(image, angle):
    return ndimage.rotate(image, angle, cval = 255)

def ConvertOpenCVToPIL(ocvImg):
    ocvImg = cv2.cvtColor(ocvImg, cv2.COLOR_BGR2RGB)
    pil_IM = Image.fromarray(ocvImg)
    return pil_IM

def IfContourIsRectangleReturnCoordinates(cnt, faultTol = .05):
    epsilon = cv2.arcLength(cnt, True) * .02
    poly = cv2.approxPolyDP(cnt, epsilon, True)
    if len(poly) == 4:
        vList = GetVerticeListFromContour(poly)
        vList.sort(key = lambda v: (v[0], v[1]))
        X = 0
        Y = 1
        # Compare vertices
        v0v2A, v0v2L = GEO.GetLineAngleAndLen(vList[0][X], vList[0][Y], vList[2][X], vList[2][Y])
        v1v3A, v1v3L = GEO.GetLineAngleAndLen(vList[1][X], vList[1][Y], vList[3][X], vList[3][Y])

        pctDiffA = abs(1 - (float(v0v2A) / float(v1v3A)))
        pctDiffL = abs(1 - (float(v0v2L) / float(v1v3L)))

        if pctDiffA > faultTol or pctDiffL > faultTol:
            return None

        v0v1A, v0v1L = GEO.GetLineAngleAndLen(vList[0][X], vList[0][Y], vList[1][X], vList[1][Y])
        v2v3A, v2v3L = GEO.GetLineAngleAndLen(vList[2][X], vList[2][Y], vList[3][X], vList[3][Y])

        pctDiffA = abs(1 - (float(v0v1A) / float(v2v3A)))
        pctDiffL = abs(1 - (float(v0v1L) / float(v2v3L)))

        if pctDiffA > faultTol or pctDiffL > faultTol:
            return None

        return poly
    else:
        return None

def GetHoughLinesP(im, minHoughLineLen = 5, maxLineGap = 1000):
    try:
        lines = cv2.HoughLinesP(im, 1, math.pi / 180, 100, maxLineGap = 5, minLineLength = minHoughLineLen)[0]
    except:
        return None
    linesList = []
    lens = []
    angles = []
    if lines != None:
        for l in lines:
            x1, y1, x2, y2 = l
            linesList.append([[x1, y1], [x2, y2]])
    return linesList

def DivideHoughLineListIntoRegions(lines, numOfRgns):
    X = 0
    Y = 1

    lineRgns = []
    angles = []

    for i in range(numOfRgns):
        lineRgns.append([])
        angles.append([])

    multiplier = 1
    regionLen = math.ceil(float(len(lines)) / float(4))
    for i in range(len(lines)):
        if i > regionLen * multiplier:
            multiplier = multiplier + 1
        lineRgns[multiplier - 1].append(lines[i])
        angle, length = GEO.GetLineAngleAndLen(lines[i][0][X], lines[i][0][Y], lines[i][1][X], lines[i][1][Y])
        angles[multiplier - 1].append(angle)

    return lineRgns, angles


# Gets correlation between left Y coordinate of a HoughLines and 
def SkewLinRegress(houghLines):
    linesY = sorted(houghLines, key = lambda l: (l[0][1], l[1][1]))
    angles = []
    lengths = []
    leftYs = []
    for l in linesY:
        X1 = l[0][0]
        Y1 = l[0][1]
        X2 = l[1][0]
        Y2 = l[1][1]
        leftYs.append(Y1)
        angle, length = GEO.GetLineAngleAndLen(X1, Y1, X2, Y2)
        angles.append(angle)
        lengths.append(length)

    slope, intercept, r_value, p_value, std_err = stats.linregress(angles, leftYs)
    '''
    # Show linear points and trend line on screen
    # -------------------------------------------
    (m, b) = polyfit(angles, leftYs, 1)
    yp = polyval([m,b], angles)
    plot(angles, yp)
    scatter(angles, leftYs)
    grid(True)
    xlabel('angles')
    ylabel('leftYs')
    show()
    '''
	
    return LinearRegressionResult(slope, intercept, r_value, p_value, std_err)
    
def GetContentRegionsHoriz(imEdge, lineAngleNeg45ToPos45, offsetPix = 2):
    if lineAngleNeg45ToPos45 > - 45 and lineAngleNeg45ToPos45 < 45:
        X = 0
        Y = 1

        imHeight = imEdge.shape[0]
        imWidth = imEdge.shape[1]
        lines = []

        # Create RGB Image for cropping
        imEdgeRGB = cv2.cvtColor(imEdge, cv2.COLOR_GRAY2BGR)
        cv2.imwrite('Images\TextSkew\imEdgePRELoop.png', imEdgeRGB)

        contentRgnStart = None
        contentRgnLns = []

        for i in xrange(1, imHeight):
            adjAng = 180 - 90 - abs(lineAngleNeg45ToPos45)
            adjLen = imWidth - 1
            hypAng = 90
            hypLen = math.sin(math.radians(hypAng)) * (adjLen / math.sin(math.radians(adjAng)))
            oppLen = math.sqrt((hypLen * hypLen) - (adjLen * adjLen))

            if i < oppLen and lineAngleNeg45ToPos45 < 0:
                factor = float(i) / float(oppLen)
                adjLen = factor * adjLen
                hypLen = factor * hypLen
                oppLen = factor * oppLen
            elif i + oppLen > imHeight and lineAngleNeg45ToPos45 > 0:
                oppLenNew = imHeight - i
                factor = float(oppLenNew) / float(oppLen)
                adjLen = factor * adjLen
                hypLen = factor * hypLen
                oppLen = factor * oppLen

            ptA = [0, i]

            # negative angle implies upward skew
            if lineAngleNeg45ToPos45 < 0:
                ptB = [int(math.ceil(adjLen)), int(math.ceil(i - oppLen))]
            # positive angle implies downward skew
            else: 
                ptB = [int(math.ceil(adjLen)), int(math.ceil(i + oppLen))]
            line = [ptA, ptB] 
            lineCoords = GEO.GetCoordinateListFromLine([ptA, ptB])

            collision = False
            for c in lineCoords:
                if c[Y] < imHeight and c[X] < imWidth:
                    pixelIntesnity = imEdge[c[Y], c[X]]  
                    if pixelIntesnity > 0:
                        collision = True
                        break
        
            if collision == True and contentRgnStart == None:
                lnAdd = copy.deepcopy(line)
                if line[0][Y] - offsetPix > 0 and line[1][Y] - offsetPix > 0:
                    lnAdd[0][Y] = line[0][Y] - offsetPix
                    lnAdd[1][Y] = line[1][Y] - offsetPix
                contentRgnStart = lnAdd
            elif contentRgnStart != None and collision == False:
                lnAdd = copy.deepcopy(line)
                if line[0][Y] + offsetPix < imHeight and line[1][Y] + offsetPix < imHeight:
                    lnAdd[0][Y] = line[0][Y] + offsetPix
                    lnAdd[1][Y] = line[1][Y] + offsetPix
                contentRgnLns.append([contentRgnStart, lnAdd])
                contentRgnStart = None
        
        return contentRgnLns

    else:
        return None

# GOAL: Take 2 content region bounding lines from 0 to imWidth and filter
# out any are in the region that exceeds the median gap between content. 
def ContentScanVerticalForHorizBoundingLines(imEdge, 
                                             contentRgnHoriz, 
                                             pctMedianRemoveGapFilter = 250, 
                                             pctWidthIgnoreGapFilter = 0.25,
                                             offsetPix = 2):
    collisionRgnBegX = None
    collision_Beg_End = []
    ln1, ln2 = contentRgnHoriz 
    ln1Coords = GEO.GetCoordinateListFromLine(ln1)
    ln1Coords.sort(key = lambda ln: (ln[0], ln[1]))
    ln2Coords = GEO.GetCoordinateListFromLine(ln2)
    ln2Coords.sort(key = lambda ln: (ln[0], ln[1]))
    imWidth = imEdge.shape[1]
    minGapWidth = int(round(float(imWidth) * (pctWidthIgnoreGapFilter / float(100))))

    for x in range (len(ln1Coords)):
        yLo = ln1Coords[x][1]
        yHi = ln2Coords[x][1]
        if collisionRgnBegX == None:
            for y in range(yLo, yHi):
                pixelIntesnity = imEdge[y, x]
                if pixelIntesnity > 0:
                    collisionRgnBegX = x
                    break
        else: # Collision region has begun
            collisionRgnEndX = None
            for y in range(yLo, yHi):
                pixelIntesnity = imEdge[y, x]
                if pixelIntesnity > 0:
                    collisionRgnEndX = x
                    break
            if collisionRgnEndX == None:
                collision_Beg_End.append([collisionRgnBegX, x])
                collisionRgnBegX = None

    gaps = []
    gapSizes = []
    for i in range(len(collision_Beg_End)):
        gap = None
        if i == 0:
            gap = [0, collision_Beg_End[i][0]]
        else:
            gap = [collision_Beg_End[i - 1][1], collision_Beg_End[i][0]]

        if gap[1] - gap[0] > minGapWidth:
            gaps.append(gap)
            gapSizes.append(gap[1] - gap[0])
           
    gapFinal = [collision_Beg_End[len(collision_Beg_End) - 1][1], imWidth]
    if gapFinal[1] - gapFinal[0] > minGapWidth:
        gaps.append(gapFinal)
        gapSizes.append(gapFinal[1] - gapFinal[0])

    medianGap = np.median(gapSizes)

    # Index varaibles for clarity
    topLn = 0
    btmLn = 1
    E = 0
    W = 1

    contentRegionsNew = [[ln1, ln2]]

    for i in range(len(gaps)):
        significantGapWidth = medianGap * (float(pctMedianRemoveGapFilter) / float(100))
        gapSize = gapSizes[i]
        if gapSize > significantGapWidth:
            if gaps[i][0] == 0:
                xCoord = gaps[i][1]
                yCoordTop = ln1Coords[xCoord][1]
                yCoordBtm = ln2Coords[xCoord][1]
                contentRegionsNew[0][topLn][E] = [xCoord, yCoordTop]
                contentRegionsNew[0][btmLn][E] = [xCoord, yCoordBtm]
            elif gaps[i][1] == imWidth:
                xCoord = gaps[i][0]
                yCoordTop = ln1Coords[xCoord][1]
                yCoordBtm = ln2Coords[xCoord][1]
                contentRegionsNew[len(contentRegionsNew) - 1][topLn][W] = [xCoord, yCoordTop]
                contentRegionsNew[len(contentRegionsNew) - 1][btmLn][W] = [xCoord, yCoordBtm]
            else:
                slopeTop, interTop = GEO.GetSlopeAndIntercept(ln1)
                slopeBtm, interBtm = GEO.GetSlopeAndIntercept(ln2)

                coordEnd = [gaps[i][0], int(round((slopeTop * gaps[i][0]) + interTop))]
                newRgnEastTopLn = [contentRegionsNew[len(contentRegionsNew) - 1][topLn][E], coordEnd]

                coordEnd = [gaps[i][0], int(round((slopeBtm * gaps[i][0]) + interBtm))]
                newRgnEastBtmLn = [contentRegionsNew[len(contentRegionsNew) - 1][btmLn][E], coordEnd]

                coordBeg = [gaps[i][1], int(round((slopeBtm * gaps[i][1]) + interTop))]
                newRgnWestTopLn = [coordBeg, contentRegionsNew[len(contentRegionsNew) - 1][topLn][W]]

                coordBeg = [gaps[i][1], int(round((slopeBtm * gaps[i][1]) + interBtm))]
                newRgnWestBtmLn = [coordBeg, contentRegionsNew[len(contentRegionsNew) - 1][btmLn][W]]

                contentRegionsNew[len(contentRegionsNew) - 1] = [newRgnEastTopLn, newRgnEastBtmLn]
                contentRegionsNew.append([newRgnWestTopLn, newRgnWestBtmLn])

    return contentRegionsNew

def GetImgFromContentRegion(cRgn, imRGB):
        X = 0
        Y = 1

        ln1, ln2 = cRgn
        lineAngle, length = GEO.GetLineAngleAndLen(ln1[0][X], ln1[0][Y], ln1[1][X], ln1[1][Y])

        newIm_W = ln1[1][X] - ln1[0][X]
        newIm_H = None
        yOffset = None
        newIm = None
        mask = None

        # Angles < 1 point upwards
        if lineAngle <= 1:
            newIm_H = ln2[0][Y] - ln1[0][Y]
            newIm = imRGB[ln1[1][Y]:ln2[0][Y], ln1[0][X]:ln1[1][X]]
            yOffset = ln1[1][Y]
            ptC_ln1 = [ln1[0][X], ln1[1][Y] - yOffset]
            ptC_ln2 = [ln2[1][X], ln2[0][Y] - yOffset]
        # Angles > 1 point downwards
        else:
            newIm_H = ln2[1][Y] - ln1[0][Y]
            newIm = imRGB[ln1[0][Y]:ln2[1][Y], ln1[0][X]:ln1[1][X]]
            yOffset = ln1[0][Y]
            ptC_ln1 = [ln1[1][X], ln1[0][Y] - yOffset]
            ptC_ln2 = [ln2[0][X], ln2[1][Y] - yOffset]

        ln1Offset = [[ln1[0][X], ln1[0][Y] - yOffset], [ln1[1][X], ln1[1][Y] - yOffset]]
        ln2Offset = [[ln2[0][X], ln2[0][Y] - yOffset], [ln2[1][X], ln2[1][Y] - yOffset]]

        triUp = np.array([ln1Offset[0], ln1Offset[1], ptC_ln1], np.int32)
        triDn = np.array([ln2Offset[0], ln2Offset[1], ptC_ln2], np.int32)

        cv2.fillConvexPoly(newIm, triUp, (255, 255, 255))
        cv2.fillConvexPoly(newIm, triDn, (255, 255, 255))

        return newIm    
                  

def DrawLineOnImage(imRGB, line, color = (0, 255, 0), thickness = 2):
    l = line
    cv2.line(imRGB, (l[0][0], l[0][1]), (l[1][0], l[1][1]), color, thickness)

# OBJECTS
class LinearRegressionResult:
    def __init__(self, slope, intercept, r_value, p_value, std_err):
        self.slope = slope
        self.intercept = intercept
        self.r_value = r_value
        self.p_value = p_value
        self.std_err = std_err



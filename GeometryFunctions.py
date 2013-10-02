import numpy as np
import math

# BEGIN POLYGON FUNCTIONS
def Get2DPolygonCentroid(vertices):
    X = 0
    Y = 1
    centroid = [0.0, 0.0]
    signedArea = 0.0

    for i in range(len(vertices) - 1):
        v0 = vertices[i]
        v1 = vertices[i + 1]
        area = (v0[X] * v1[Y]) - (v1[X] * v0[Y])
        signedArea = signedArea + area
        centroid[X] = centroid[X] + ((v0[X] + v1[X]) * area)
        centroid[Y] = centroid[Y] + ((v0[Y] + v1[Y]) * area)

    v0 = vertices[len(vertices) - 1]
    v1 = vertices[0]
    area = (v0[X] * v1[Y]) - (v1[X] * v0[Y])
    signedArea = signedArea + area
    centroid[X] = centroid[X] + ((v0[X] + v1[X]) * area)
    centroid[Y] = centroid[Y] + ((v0[Y] + v1[Y]) * area)

    signedArea = signedArea * 0.5
    centroid[X] = centroid[X] / (6.0 * signedArea)
    centroid[Y] = centroid[Y] / (6.0 * signedArea)

    return centroid

def SortVerticesClockwise(vertices):
    centroid = Get2DPolygonCentroid(vertices)


def DoesLineTouchPolygonBorder(line, polyVertsClockwise):
    X = 0
    Y = 1
    line.sort(key = lambda v: v[X])
    for i in range(len(polyVertsClockwise)):
        if i < len(polyVertsClockwise) - 1:
            lineCur = [polyVertsClockwise[i], polyVertsClockwise[i + 1]]
        else:
            lineCur = [polyVertsClockwise[i], polyVertsClockwise[0]]
        lineCur.sort(key = lambda v: v[X])
        intersectPnt = GetIntersectionPoint2Lines(line, lineCur)
        if intersectPnt is not None:
            return True
    return False

# BEGIN LINE FUNCTIONS
def GetSlopeAndIntercept(line):
    X = 0
    Y = 1
    slope = None
    intercept = None
    if (float(line[1][X] - line[0][X]) != 0):
        slope = float(line[1][Y] - line[0][Y]) / float(line[1][X] - line[0][X])       
        # L1[0][Y] = L1_Slope * L1[0][X] + L1_Intercept 
        # so...
        intercept = line[0][Y] - (slope * line[0][X])
    return slope, intercept

def GetIntersectionPoint2Lines(L1, L2):
    X = 0
    Y = 1
    L1Slope, L1Inter = GetSlopeAndIntercept(L1)
    L2Slope, L2Inter = GetSlopeAndIntercept(L2)

    if L1Slope == L2Slope:
        return None
    elif L1Slope == None or L2Slope == None:
        return None
    else:
        intersection = float(L2Inter - L1Inter) / float(L1Slope - L2Slope)
        if ((intersection >= L1[0][X] and intersection <= L1[1][X]) and
            (intersection >= L2[0][X] and intersection <= L2[1][X])):
            return intersection
        else:
            return None

# X2, Y2 must be a coordinate to the right of  X1, Y1
# Angles are calculated as if X1, Y1 is the opposite angle
# of a right triangle and X2, Y2 is the adjacent angle or a 
# right triangle. Assumes Hypotenuse angle is 90 degrees.
def GetLineAngleAndLen(x1, y1, x2, y2):
    deltaY = y2 - y1
    deltaX = x2 - x1
    lineLen = math.sqrt(abs(math.pow(y2 - y1, 2)) + abs(math.pow(x2 - x1, 2)))
    angle = math.atan2(deltaY, deltaX) * 180 / math.pi
    return angle, lineLen

def DivideLineSegment(line, divisor):
    lines = []
    X = 0
    Y = 1
    angle, length = GetLineAngleAndLen(line[0][X], line[0][Y], line[1][X], line[1][Y])
    ptA = line[0]
    ptB = line[1]

    opp = ptB[Y] - ptA[Y] 
    adj = ptB[X] - ptA[X]
    hyp = int(math.floor(math.sqrt((opp * opp) + (adj * adj))))

    flt_OppDiv = opp / float(divisor)
    avg_Opp = None
    lst_Opp = []

    flt_AdjDiv = adj / float(divisor)
    avg_Adj = None
    lst_Adj = []

    curPt = [ptA[X], ptA[Y]]

    for i in range(divisor):
        curOpp = None
        curAdj = None
        if i == 0:
            curOpp = int(math.floor(flt_OppDiv))
            curAdj = int(math.floor(flt_AdjDiv))
        else:
            if avg_Opp > flt_OppDiv:
                curOpp = int(math.floor(flt_OppDiv + .000001))
            else:
                curOpp = int(math.ceil(flt_OppDiv - .000001))

            if avg_Adj > flt_AdjDiv:
                curAdj = int(math.floor(flt_AdjDiv + .000001))
            else:
                curAdj = int(math.ceil(flt_AdjDiv - .000001))
        

        lst_Opp.append(curOpp)
        lst_Adj.append(curAdj)

        newPt = []
        if angle <= 0:
            newPt = [curPt[X] + curAdj - 1, curPt[Y] - curOpp + 1]
        else:
            newPt = [curPt[X] + curAdj - 1, curPt[Y] + curOpp - 1]

        lineSeg = [curPt, newPt]
        lines.append(lineSeg)

        curPt = [newPt[X] + 1, newPt[Y] + 1]

        avg_Opp = np.mean(lst_Opp)
        avg_Adj = np.mean(lst_Adj)
            
    return lines


def GetCoordinateListFromLine(line):
    coordList = []

    x0 = line[0][0]
    x1 = line[1][0]
    y0 = line[0][1]
    y1 = line[1][1]

    dX = x1 - x0
    dY = y1 - y0

    slope, inter = GetSlopeAndIntercept(line)

    if abs(float(dY / dX)) > 1:
        # Line is more vertical than horizontal
        for y in xrange (y0, y1 + 1):
            x = float(y - inter) / float(slope)
            x = int(round(x))
            coordList.append([x, y])
    else:
        # Line is more horizontal than vertical
        for x in xrange(x0, x1 + 1):
            y = (slope * x) + inter
            y = int(round(y))
            coordList.append([x, y])
        
    return coordList
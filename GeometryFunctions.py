import numpy as np
import math

# Globals 
X = 0
Y = 1

# BEGIN POLYGON FUNCTIONS
def Get2DPolygonCentroid(vertices):
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
    slope = None
    intercept = None
    if (float(line[1][X] - line[0][X]) != 0):
        slope = float(line[1][Y] - line[0][Y]) / float(line[1][X] - line[0][X])       
        # L1[0][Y] = L1_Slope * L1[0][X] + L1_Intercept 
        # so...
        intercept = line[0][Y] - (slope * line[0][X])
    return slope, intercept

def GetIntersectionPoint2Lines(L1, L2):
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
    lineLen = np.sqrt(abs(math.pow(y2 - y1, 2)) + abs(math.pow(x2 - x1, 2)))
    angle = math.atan2(deltaY, deltaX) * 180 / math.pi
    return angle, lineLen

def DivideLineSegment(line, divisor):
    lines = []
    angle, length = GetLineAngleAndLen(line[0][X], line[0][Y], line[1][X], line[1][Y])
    ptA = line[0]
    ptB = line[1]

    opp = ptB[Y] - ptA[Y] 
    adj = ptB[X] - ptA[X]
    hyp = int(np.floor(math.sqrt((opp * opp) + (adj * adj))))

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
            curOpp = int(np.floor(flt_OppDiv))
            curAdj = int(np.floor(flt_AdjDiv))
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


def GetCoordinateListFromLine(x0, y0, x1, y1):
    slope, inter = GetSlopeAndIntercept([[x0, y0], [x1, y1]])

    if slope == None:
       coord_count = y1 - y0 
       coords = np.zeros(shape = (coord_count, 2), dtype = np.int32)
       for i in xrange(len(coords)):
           coords[i] = [x0, y0 + i]
       return coords

    if abs(slope) <= 1:
        coord_count = x1 - x0
    elif y0 > y1:
        coord_count = y0 - y1
    else:
        coord_count = y1 - y0

    coords = np.zeros(shape = (coord_count, 2), dtype = np.int32)

    steep = abs(y1 - y0) > abs(x1 - x0)
    if steep == True:
        x0, y0 = y0, x0
        x1, y1 = y1, x1
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0
    delta_x = x1 - x0
    delty_y = abs(y1 - y0)
    error = delta_x / 2
    y = y0
    y_step = None
    if y0 < y1:
        y_step = 1
    else:
        y_step = -1

    for i in xrange(0, coord_count):
        if steep:
            coords[i] = [y, i + x0]
        else:
            coords[i] = [i + x0, y]
        error = error - delty_y
        if error < 0:
            y = y + y_step
            error = error + delta_x

    return coords

def Get_Perpendicular_Slope_And_Intercept(x0, y0, x1, y1, intersection_coordinate):
    slope_orig, inter_orig = GetSlopeAndIntercept([[x0, y0], [x1, y1]])

    if slope_orig == 0:
        slope_new = None
        inter_new = None
    else:
        slope_new = -1 * np.reciprocal(slope_orig)
        inter_new = intersection_coordinate[Y] - (slope_new * intersection_coordinate[X])

    return slope_new, inter_new

def Get_Ln_Endpt_From_Opp_Ang_And_Top_Lft_Crd_To_West_Edge\
    (opp_ang, x_max, y_max, x0, y0):
    # If Y increases as we go south on the coordinate plane, a negative angle implies a northward
    # skew from east to west. 
    # If Y decreases as we go south on the coordinate plane, the opposite is true. 
    if x0 > x_max or y0 > y_max: 
        return None

    adj_ang = 180 - (90 + abs(opp_ang))
    if opp_ang < 0:
        opp_len = y0
        adj_len = (opp_len * np.sin(np.radians(adj_ang))) / np.sin(np.radians(abs(opp_ang))) 
        y1 = 0
    else:
        adj_len = x_max - x0
        opp_len = abs((adj_len * np.sin(np.radians(opp_ang))) / np.sin(np.radians(adj_ang)))
        y1 = int(np.round(y0 + opp_len))

    x1 = int(np.round(x0 + adj_len))
    if x1 > x_max:
        slope, inter = GetSlopeAndIntercept([[x0, y0], [x1, y1]])
        x1 = x_max
        y1 = int(np.round((slope * x1) + inter))

    return [x1, y1]

def Get_Perp_Line_At_X_Coord_Of_Line(line_src, x0, line_height):
    slope, intercept = GetSlopeAndIntercept(line_src)
    y0 = (slope * x0) + intercept
    slope_perp, inter_perp = \
        Get_Perpendicular_Slope_And_Intercept\
        (line_src[0][X], line_src[0][Y], line_src[1][X], line_src[1][Y], [x0, y0])
    y1 = y0 + line_height
    if slope_perp == None:
        x1 = line_src[1][X]
    else:
        x1 = (y1 - inter_perp) / slope_perp
        
    return int(np.round(x0)), int(np.round(y0)), int(np.round(x1)), int(np.round(y1))

def LineOffset(line, offset_amt, min_val, max_val, axis = 1):
    if (line[0][axis] + offset_amt > min_val and line[0][axis] + offset_amt < max_val and 
        line[1][axis] + offset_amt > min_val and line[1][axis] + offset_amt < max_val):
        line[0][axis] = line[0][axis] + offset_amt
        line[1][axis] = line[1][axis] + offset_amt

    return line

class Triangle:
    def __init__(self, 
                 adj_len, 
                 adj_ang, 
                 hyp_ang):

        self.adj_len = adj_len
        self.adj_ang = adj_ang
        self.hyp_ang = hyp_ang
        self.hyp_len = math.sin(math.radians(hyp_ang)) * (adj_len / math.sin(math.radians(adj_ang)))
        self.opp_len = math.sqrt((self.hyp_len * self.hyp_len) - (adj_len * adj_len))
        self.opp_ang = 180 - (self.adj_ang + self.hyp_ang)

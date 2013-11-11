import numpy as np
import scipy.stats
import matplotlib.pyplot
import math

X = 0
Y = 1

X1 = 0
Y1 = 1
X2 = 2
Y2 = 3


def SkewLinRegress(h_lines, sort_axis = Y1):
    lines = []
    lines = h_lines[h_lines[:, sort_axis].argsort()]

    angles = []
    lengths = []
    leftYs = []
    for l in lines:
        leftYs.append(l[Y1])
        angle, length = GetLineAngleAndLen(l[X1], l[Y1], l[X2], l[Y2])
        angles.append(angle)
        lengths.append(length)

    slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(leftYs, angles)
    # Show linear points and trend line on screen
    # -------------------------------------------
    '''
    (m, b) = np.polyfit(leftYs, angles, 1)
    yp = np.polyval([m,b], leftYs)
    matplotlib.pyplot.plot(leftYs, yp)
    matplotlib.pyplot.scatter(leftYs, angles)
    matplotlib.pyplot.grid(True)
    matplotlib.pyplot.xlabel('leftYs')
    matplotlib.pyplot.ylabel('angles')
    matplotlib.pyplot.show()
    '''
    return LinearRegressionResult(slope, intercept, r_value, p_value, std_err)

def Get_Median_Angle_List(lines, number_of_angle_groups = 5):
    # Sort the list of lines by the left most Y coordinate
    lines.sort(key = lambda l: (l[0][1], l[1][1]))

    # Get angles in a list, sorted by the left most Y coordinate
    angles = []
    for l in lines:
        ang, length = GetLineAngleAndLen(l[0][X], l[0][Y], l[1][X], l[1][Y])
        angles.append(ang)

    ang_list_cur = [] # temporary list: holds groups of angles we want to get
                      # the mediean for.

    med_ang_list = [] # return list: holds the median angles of desired angle
                      # groups. 


    # set the i value to move on to the next set at
    i_iter = int(round(float(len(angles)) / float(number_of_angle_groups)))
    move_to_next_angle_at_i_equals = i_iter

    for i in xrange(len(angles)):
        # add the angle to the temporary list
        ang_list_cur.append(angles[i])
        if i > move_to_next_angle_at_i_equals:
            # If we are at a point where we want to move to the next grouping,
            # set the i value where we want to move to the next grouping,
            # append the return list with the median of everything in the 
            # temporary list, and clear the temporary list
            move_to_next_angle_at_i_equals =  move_to_next_angle_at_i_equals + i_iter
            med_ang_list.append(np.median(ang_list_cur))      
            ang_list_cur = []
        if i == len(angles) - 1:
            # end of loop behavior 
            med_ang_list.append(np.median(ang_list_cur))
            ang_list_cur = []

    return med_ang_list


# Copied over some geometry functions so as not to produce a dependency
# ------------------------------------------------------------------
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

# OBJECTS
class LinearRegressionResult:
    def __init__(self, slope, intercept, r_value, p_value, std_err):
        self.slope = slope
        self.intercept = intercept
        self.r_value = r_value
        self.p_value = p_value
        self.std_err = std_err
import cv2
import numpy as np
import copy

import GeometryFunctions as GEO
import StatsFunctions as STATS

X = 0
Y = 1

class Document_Page:
    def __init__(self, document_im):
        self.document_im = document_im

        self.im_height = document_im.shape[0]
        self.im_width = document_im.shape[1]

        self.document_im_processed = self.GetProcessedDocIm()
        self.hough_lines_P = self.Get_Hough_Lines_P(min_h_line_len = float(self.im_width) / float(10), 
                                                    max_line_gap = float(self.im_width) / float(50))
        self.hough_line_lin_rgrs = STATS.SkewLinRegress(self.hough_lines_P)
 

    def GetProcessedDocIm(self):
        im_gry = cv2.cvtColor(copy.deepcopy(self.document_im), cv2.COLOR_BGR2GRAY)
        im_gry = cv2.medianBlur(im_gry, 5)
        im_thr = cv2.adaptiveThreshold(copy.deepcopy(im_gry), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        edges = cv2.Canny(copy.deepcopy(im_thr), 200, 250)
        cnts, h = cv2.findContours(copy.deepcopy(edges), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        RGB_edge = cv2.cvtColor(copy.deepcopy(edges), cv2.COLOR_GRAY2BGR)
        for c in cnts:
            epsilon = cv2.arcLength(c, True) * .02
            approx = cv2.approxPolyDP(c, epsilon, True)
            cv2.fillPoly(RGB_edge, pts = [approx], color = (255, 255, 255))

        GRY_edge = cv2.cvtColor(RGB_edge, cv2.COLOR_BGR2GRAY)
        return GRY_edge


    def Get_Hough_Lines_P(self, min_h_line_len, max_line_gap):
        return cv2.HoughLinesP(self.document_im_processed, 1, np.pi / 180, 100, maxLineGap = 5, minLineLength = min_h_line_len)[0]


    def GetContentRegionsHoriz(self, offset_pix = 0, step_rows = 3):
        im_height = self.document_im_processed.shape[0]
        im_width = self.document_im_processed.shape[1]

        content_rgn_beg = None
        content_rgns = []
        content_rgn_heights = []
        content_rgn_widths = []
        ang_cur = self.hough_line_lin_rgrs.intercept

        if ang_cur > 0:
            # If the angle is > 0 the the document is skewed downward from  
            # left to right. We need to sweep the top of the document since
            # downward pointed lines beginning at the left of the document
            # could miss things. 
            for x in reversed(xrange(1, im_width, step_rows)):
                ptA = [x, 0]
                if content_rgn_beg == None:
                    ptB = GEO.Get_Ln_Endpt_From_Opp_Ang_And_Top_Lft_Crd_To_West_Edge\
                            (ang_cur, im_width, im_height - 1, ptA[X], ptA[Y])
                else:
                    ptB = GEO.Get_Ln_Endpt_From_Opp_Ang_And_Top_Lft_Crd_To_West_Edge\
                            (ang_cur, content_rgn_beg[1][X], im_height, ptA[X], ptA[Y])

                line = [ptA, ptB]
                collision_max = int(round(float(ptB[X] - ptA[X]) / float(100)))
                collision_thresh_reached, collision_coords = self.DoesLineCollideWithContentInDocIM(line, collision_max)

                if collision_thresh_reached == True and content_rgn_beg == None:
                    content_rgn_beg = self.Get_Line_From_Precision_Collisn_Chk\
                                        (ang_cur, ptA, step_rows)

                elif collision_thresh_reached == False and content_rgn_beg != None:
                    content_rgn_end = self.Get_Line_From_Precision_Collisn_Chk\
                                        (ang_cur, ptA, step_rows, 0, True, 3)
                    content_rgn_beg, content_rgn_end = \
                        self.Reconcile_Rgn_Beg_X_End_X(content_rgn_beg, content_rgn_end)

                    cRgn = Content_Rgn_Horiz([content_rgn_beg, content_rgn_end], self.document_im_processed)
                    content_rgns.append(cRgn)

                    content_rgn_beg = None
                    content_rgn_end = None

        # Go down the side from north to south
        for y in xrange(1, im_height - 1, step_rows):
            # Plug numbers into the linear regression equation to get the angle
            # to draw the line at. 
            # If the content region has already begun we want to set ang_cur 
            # equal to the angle where the content region began 
            if content_rgn_beg == None:
                ang_cur = (y * self.hough_line_lin_rgrs.slope) + self.hough_line_lin_rgrs.intercept

            ptA = [0, y]

            if content_rgn_beg == None:
                ptB = GEO.Get_Ln_Endpt_From_Opp_Ang_And_Top_Lft_Crd_To_West_Edge\
                        (ang_cur, im_width, im_height - 1, ptA[X], ptA[Y])
            else:
                ptB = GEO.Get_Ln_Endpt_From_Opp_Ang_And_Top_Lft_Crd_To_West_Edge\
                        (ang_cur, content_rgn_beg[1][X], im_height, ptA[X], ptA[Y])

            line = [ptA, ptB] 

            collision_max = int(round(float(ptB[X] - ptA[X]) / float(100)))
            collision_thresh_reached, collision_pts = self.DoesLineCollideWithContentInDocIM(line, collision_max)
        
            if collision_thresh_reached == True and content_rgn_beg == None:
                content_rgn_beg = self.Get_Line_From_Precision_Collisn_Chk\
                                        (ang_cur, ptA, step_rows)
            elif content_rgn_beg != None and collision_thresh_reached == False:
                content_rgn_end = self.Get_Line_From_Precision_Collisn_Chk\
                                    (ang_cur, ptA, step_rows, 0, True, 3)
                content_rgn_beg, content_rgn_end = \
                    self.Reconcile_Rgn_Beg_X_End_X(content_rgn_beg, content_rgn_end)

                cRgn = Content_Rgn_Horiz([content_rgn_beg, content_rgn_end], self.document_im_processed)
                content_rgns.append(cRgn)

                content_rgn_beg = None
                content_rgn_end = None
    
        # If the document is skewed at an upward angle, we need to sweep bottom of
        # the document from east to west. 
        ang_cur = ((im_height - 1) * self.hough_line_lin_rgrs.slope) + self.hough_line_lin_rgrs.intercept
        if ang_cur < 1:
            for x in xrange(1, im_width, step_rows):
                ptA = [x, im_height - 1]

                if content_rgn_beg == None:
                    ptB =   GEO.Get_Ln_Endpt_From_Opp_Ang_And_Top_Lft_Crd_To_West_Edge\
                            (ang_cur, im_width, im_height - 1, ptA[X], ptA[Y])
                else:
                    ptB = GEO.Get_Ln_Endpt_From_Opp_Ang_And_Top_Lft_Crd_To_West_Edge\
                    (ang_cur, content_rgn_beg[1][X], im_height, ptA[X], ptA[Y])

                line = [ptA, ptB]
                collision_max = int(round(float(ptB[X] - ptA[X]) / float(100)))
                collision_thresh_reached, collision_coords = self.DoesLineCollideWithContentInDocIM(line, collision_max)

                if collision_thresh_reached == True and content_rgn_beg == None:
                    content_rgn_beg = GEO.LineOffset(copy.deepcopy(line), -1 * (offset_pix), 0, im_height - 1)
                elif collision_thresh_reached == False and content_rgn_beg != None:
                    content_rgn_end = self.Get_Line_From_Precision_Collisn_Chk\
                                        (ang_cur, ptA, step_rows, 0, True, 3)
                    content_rgn_beg, content_rgn_end = \
                        self.Reconcile_Rgn_Beg_X_End_X(content_rgn_beg, content_rgn_end)

                    cRgn = Content_Rgn_Horiz([content_rgn_beg, content_rgn_end], self.document_im_processed)
                    content_rgns.append(cRgn)

                    content_rgn_beg = None
                    content_rgn_end = None

        content_rgns_ret = []

        return content_rgns

    def DoesLineCollideWithContentInDocIM(self, line, collision_max):
        line_coords = GEO.GetCoordinateListFromLine(line[0][X], line[0][Y], line[1][X], line[1][Y])
        collision_coords = []
        thresh_reached = False

        for c in line_coords:
            if c[Y] < self.document_im_processed.shape[0] and c[X] < self.document_im_processed.shape[1]:
                pixel_intensity = self.document_im_processed[c[Y], c[X]]  
                if pixel_intensity > 0:
                    collision_coords.append(c)
                    if len(collision_coords) > collision_max:
                        thresh_reached = True

        return thresh_reached, collision_coords

    def Reconcile_Rgn_Beg_X_End_X(self, content_rgn_beg, content_rgn_end):     
        if content_rgn_beg[1][Y] > self.im_height - 1:
            # The south most y coordinate of the content rgn exceeds im_height - 1
            slope, inter = GEO.GetSlopeAndIntercept(content_rgn_beg)
            content_rgn_beg[1][Y] = self.im_height - 1
            content_rgn_beg[1][X] = int(np.round(((content_rgn_beg[1][Y] - inter) / slope)))
        if content_rgn_end[1][Y] > self.im_height - 1:
            # The south most y coordinate of the content rgn exceeds im_height - 1
            slope, inter = GEO.GetSlopeAndIntercept(content_rgn_end)
            content_rgn_end[1][Y] = self.im_height - 1
            content_rgn_end[1][X] = int(np.round((content_rgn_end[1][Y] - inter) / slope))

        if content_rgn_end[1][X] < content_rgn_beg[1][X]:
            # The right most X coordinate of the btm line is less
            # than the right most X coordinate of the top line.
            # Occurs in the btm left corner of a south skewed doc. 
            content_rgn_beg[1][X] = content_rgn_end[1][X]
            content_rgn_beg[1][Y] = content_rgn_end[1][Y] - (content_rgn_end[0][Y] - content_rgn_beg[0][Y])
        elif content_rgn_end[0][X] < content_rgn_beg[0][X]:
            # The left most X coordinate of the btm line is less
            # than the left most X coordinate of the top line.
            # Occurs in the btm left corner of a south skewed doc.
            content_rgn_end[0][X] = content_rgn_beg[0][X]
            content_rgn_end[0][Y] = content_rgn_beg[0][Y] + (content_rgn_end[1][Y] - content_rgn_beg[1][Y])
        elif content_rgn_beg[1][X] < content_rgn_end[1][X]:
            # The right most X coordinate of the top line is less
            # than the right most X coordinate of the btm line.
            # Occurs in the top left corner of a north skewed doc.
            content_rgn_end[1][X] = content_rgn_beg[1][X]
            content_rgn_end[1][Y] = content_rgn_beg[1][Y] + (content_rgn_end[0][Y] - content_rgn_beg[0][Y])
        elif content_rgn_beg[0][X] < content_rgn_end[0][X]:
            # The left most X coordinate of the top line is less
            # than the left most X coordinate of the btm line.
            # Occurs in the btm left corner of a north skewed doc.
            content_rgn_beg[0][X] = content_rgn_end[0][X]
            content_rgn_beg[0][Y] = content_rgn_beg[0][Y] - (content_rgn_end[1][Y] - content_rgn_beg[1][Y])

        return content_rgn_beg, content_rgn_end

    def Get_Line_From_Precision_Collisn_Chk\
        (self, angle, ref_pt, n, collision_max = 0, rgn_beg_found = False, xtra_chk_num = 0):
        # -----------------------------------------------------------------------------------
        if ref_pt[Y] == 0 or ref_pt[Y] == self.im_height - 1:
            lines = self.Get_N_Prev_Post_Lines(angle, ref_pt, xtra_chk_num, n, X)
            if ref_pt[Y] == 0:
                lines.sort(key = lambda l: l[0][X], reverse = True)
            else:
                lines.sort(key = lambda l: l[0][X], reverse = False)
        else:
            lines = self.Get_N_Prev_Post_Lines(angle, ref_pt, xtra_chk_num, n, Y)

        collision_tresh_hit = False
        collision_counts = dict([])
        for line_idx in xrange(len(lines)):
            line = lines[line_idx]
            collision_thresh_hit, collision_pts = self.DoesLineCollideWithContentInDocIM(line, collision_max)
            collision_counts[line_idx] = len(collision_pts)
            if collision_thresh_hit == True and rgn_beg_found == False:
                return line
            elif collision_tresh_hit == False and rgn_beg_found == True:
                return line

        if collision_tresh_hit == False and rgn_beg_found == True:
            min_collision_line_idx = min(collision_counts, key = collision_counts.get)
            return lines[min_collision_line_idx]

    def Get_N_Prev_Post_Lines(self, angle, ref_pt, prev, post, var_crd = Y, collision_max = 0):
        ref_coord_beg = ref_pt[var_crd] - prev
        ref_coord_end = ref_pt[var_crd] + post
        lines = []
        for i in xrange(ref_coord_beg, ref_coord_end):
            if var_crd == Y:
                ptA = [ref_pt[X], i]
                ptB = GEO.Get_Ln_Endpt_From_Opp_Ang_And_Top_Lft_Crd_To_West_Edge\
                        (angle, self.im_width, self.im_height - 1, ptA[X], ptA[Y])
            else:
                ptA = [i, ref_pt[Y]]
                ptB = GEO.Get_Ln_Endpt_From_Opp_Ang_And_Top_Lft_Crd_To_West_Edge\
                        (angle, self.im_width, self.im_height - 1, ptA[X], ptA[Y])
            lines.append([ptA, ptB])

        return lines
        
    def Chk_For_Collisn_Ret_Line_ThreshBool_CollisnPts\
            (self, angle, ref_pt, collision_max = 0):

        ptA = ref_pt
        ptB = GEO.Get_Ln_Endpt_From_Opp_Ang_And_Top_Lft_Crd_To_West_Edge\
                (angle, self.im_width, self.im_height - 1, ptA[X], ptA[Y])

        if ptB[Y] > self.im_height:
            slope, inter = GEO.GetSlopeAndIntercept([ptA, ptB])
            ptB[Y] = self.im_height - 1
            ptB[X] = int(np.round(float(ptB[Y] - inter) / slope))

        # collision_thresh_reached - boolean 
        # collision_pts are points where the line collides with document 
        # img content. 
        collision_thresh_reached, collision_pts = \
            self.DoesLineCollideWithContentInDocIM([ptA, ptB], collision_max)

        # return the data gathered by the function. 
        return ptA, ptB, collision_thresh_reached, collision_pts



class Content_Rgn_Horiz():
    def __init__(self, c_rgn_lns, base_im):
        self.c_rgn_lns = c_rgn_lns
        self.line_ang, self.line_len = GEO.GetLineAngleAndLen(c_rgn_lns[0][0][X], c_rgn_lns[0][0][Y],
                                                              c_rgn_lns[0][1][X], c_rgn_lns[0][1][Y])
        self.height = c_rgn_lns[1][0][Y] - c_rgn_lns[0][0][Y]
        self.width = c_rgn_lns[0][1][X] - c_rgn_lns[0][0][X]
        self.im_height = base_im.shape[0]
        self.im_width = base_im.shape[1]

        # Declaring self.gaps and self.column_sep_gaps so Visual Studio
        # intellisense picks them up. 
        self.gaps = []
        self.column_sep_gaps = []

        self.Init_Gaps(base_im)
        self.Init_Column_Sep_Gaps()
        
    def DrawRgnOnImage(self, out_im_RGB, color_beg = (0, 255, 0), color_end = (255, 0, 0), thickness = 1):
        l0 = self.c_rgn_lns[0]
        cv2.line(out_im_RGB, (l0[0][X], l0[0][Y]), (l0[1][X], l0[1][Y]), color_beg, thickness)
        l1 = self.c_rgn_lns[1]
        cv2.line(out_im_RGB, (l1[0][X], l1[0][Y]), (l1[1][X], l1[1][Y]), color_end, thickness)

    def Init_Gaps(self, base_im):
        gaps = []
        content_beg_x = None
        content_sections = []
        ln1, ln2 = self.c_rgn_lns

        if ln1[0][Y] == 148:
            flag = True
        # The angle and length of ln1 and ln2 should be equivalent. 
        angle, length = GEO.GetLineAngleAndLen(ln1[0][X], ln1[0][Y], ln1[1][X], ln1[1][Y])

        # Grab the coordinates of both lines and sort them by 
        # their left most X coordiante
        ln1_coords = GEO.GetCoordinateListFromLine(ln1[0][X], ln1[0][Y], ln1[1][X], ln1[1][Y])
        ln1_coords = ln1_coords[ln1_coords[:, 0].argsort()]
        ln2_coords = GEO.GetCoordinateListFromLine(ln2[0][X], ln2[0][Y], ln2[1][X], ln2[1][Y])
        ln2_coords = ln2_coords[ln2_coords[:, 0].argsort()]

        # min_X represents the lowest X coordinate on ln1 and ln2
        # max_X represents the greatest X coordinate on ln1 and ln2
        min_X = ln1_coords[0][X]
        max_X = ln1_coords[len(ln1_coords) - 1][X]

        im_width = base_im[1]

        for i in xrange(len(ln1_coords) - 1):
            # Iterate through each coordinate in ln1 and ln2. 
            # ln1 and ln2 should always have the same number 
            # of coordinates. 
            x_coord = i + min_X
            yLo = ln1_coords[i][1]
            yHi = ln2_coords[i][1]

            slope_perp, inter_perp = \
                GEO.Get_Perpendicular_Slope_And_Intercept(ln1[0][X], ln1[0][Y], 
                                                          ln1[1][X], ln1[1][Y], 
                                                          [x_coord, yLo])

            if slope_perp == None:
                scan_coords = []
                for y_iter in range(yLo, yHi):
                    scan_coords.append([x_coord, y_iter])
            else:
                end_X = (yHi - inter_perp) / slope_perp
                if end_X > base_im.shape[1] - 1:
                    end_X = base_im.shape[1] - 1
                scan_coords = GEO.GetCoordinateListFromLine(x_coord, yLo, int(np.floor(end_X)), int(np.floor(yHi + 1)))

            if content_beg_x == None: 
                # Collision region has not begun. Check to see if it is 
                # beginning
                for coord in scan_coords:
                    if (coord[X] >= 0 and coord[Y] >= 0 and 
                        coord[X] < self.im_width and coord[Y] < self.im_height):
                        pixel_intensity = base_im[coord[Y], coord[X]]
                        if pixel_intensity > 0:
                            content_beg_x = x_coord
                            break
            else: 
                # Collision region has begun. Check to see if it is ending
                content_end_x = None
                for coord in scan_coords:
                    pixel_intensity = base_im[coord[Y], coord[X]]
                    if pixel_intensity > 0:
                        content_end_x = x_coord
                        break
                if content_end_x == None:
                    content_sections.append([content_beg_x, i + min_X])
                    content_beg_x = None

        for i in xrange(0, len(content_sections)):
            if i == 0:
                gap = [0, content_sections[i][0]]
            else:
                gap = [content_sections[i - 1][1], content_sections[i][0]]
            gaps.append(gap)
        
        if (len(content_sections)) > 0:
            final_gap = [content_sections[len(content_sections) - 1][1], max_X]
            gaps.append(final_gap)

        self.gaps = gaps

    def Init_Column_Sep_Gaps(self, rgn_height_factor = 1):
        self.column_sep_gaps = []
        col_sep_gap_min_distance = self.height * rgn_height_factor

        if self.c_rgn_lns[0][0][Y] == 148:
            flag = True
        for i in xrange(len(self.gaps)):
            g_cur = self.gaps[i]

            # Get the length of the line formed by the angle and horizontal distance
            # between the two points. 
            line_len = ((g_cur[1] - g_cur[0]) /
                         np.sin(np.radians((180 - 90 - abs(self.line_ang)))))

            if (line_len > col_sep_gap_min_distance):  
                if len(self.column_sep_gaps) < 1:
                    self.column_sep_gaps.append(g_cur)
                else:
                    idx_prv = len(self.column_sep_gaps) - 1
                    g_prv = self.column_sep_gaps[idx_prv]
                    if g_cur[0] - g_prv[1] < col_sep_gap_min_distance:
                        self.column_sep_gaps[idx_prv][1] = g_cur[1]
                    else:
                        self.column_sep_gaps.append(g_cur)

    def Get_Content_Rgns_From_Col_Sep_Gaps(self):
        content_rgns = []
        l0_horiz = self.c_rgn_lns[0]
        l1_horiz = self.c_rgn_lns[1]
        if len(self.column_sep_gaps) == 0:
            rgn_beg_x = l0_horiz[0][X]
            rgn_end_x = l0_horiz[1][X]
            if rgn_end_x - rgn_beg_x > 1:    
                x0, y0, x1, y1 = GEO.Get_Perp_Line_At_X_Coord_Of_Line(l0_horiz, rgn_beg_x, self.height)
                ln0_vert = [[x0, y0], [x1, y1]]
                x0, y0, x1, y1 = GEO.Get_Perp_Line_At_X_Coord_Of_Line(l0_horiz, rgn_end_x, self.height)
                ln1_vert = [[x0, y0], [x1, y1]]
                content_rgns.append(Content_Rgn(l0_horiz, l1_horiz, ln0_vert, ln1_vert))
        else:
            for i in xrange(len(self.column_sep_gaps)):
                if i == 0 and len(self.column_sep_gaps) == 1 and self.column_sep_gaps[i][0] == 0:
                    rgn_beg_x = self.column_sep_gaps[i][1]
                    rgn_end_x = l0_horiz[1][X]
                elif i == 0:
                    rgn_beg_x = l0_horiz[0][X]
                    rgn_end_x = self.column_sep_gaps[i][0]
                else:
                    rgn_beg_x = self.column_sep_gaps[i - 1][1]
                    rgn_end_x = self.column_sep_gaps[i][0]
             
                if rgn_end_x - rgn_beg_x > 1:    
                    x0, y0, x1, y1 = GEO.Get_Perp_Line_At_X_Coord_Of_Line(l0_horiz, rgn_beg_x, self.height)
                    ln0_vert = [[x0, y0], [x1, y1]]
                    x0, y0, x1, y1 = GEO.Get_Perp_Line_At_X_Coord_Of_Line(l0_horiz, rgn_end_x, self.height)
                    ln1_vert = [[x0, y0], [x1, y1]]
                    content_rgns.append(Content_Rgn(l0_horiz, l1_horiz, ln0_vert, ln1_vert))

        return content_rgns

class Content_Rgn():
    def __init__(self, line_1_horiz, line_2_horiz, line_1_vert, line_2_vert):
        self.line_1_horiz = line_1_horiz
        self.line_2_horiz = line_2_horiz
        self.line_1_vert = line_1_vert
        self.line_2_vert = line_2_vert

    def Draw_Rgn_On_Image(self, out_im_RGB, color = (0, 255, 0), thickness_horiz = 1, thickness_vert = 2):
        cv2.line(out_im_RGB, 
                 (self.line_1_horiz[0][X], self.line_1_horiz[0][Y]), 
                 (self.line_1_horiz[1][X], self.line_1_horiz[1][Y]), 
                 color, thickness_horiz)
        cv2.line(out_im_RGB, 
                 (self.line_2_horiz[0][X], self.line_2_horiz[0][Y]), 
                 (self.line_2_horiz[1][X], self.line_2_horiz[1][Y]), 
                 color, thickness_horiz)
        cv2.line(out_im_RGB, 
                 (self.line_1_vert[0][X], self.line_1_vert[0][Y]), 
                 (self.line_1_vert[1][X], self.line_1_vert[1][Y]), 
                 color, thickness_vert)
        cv2.line(out_im_RGB, 
                 (self.line_2_vert[0][X], self.line_2_vert[0][Y]), 
                 (self.line_2_vert[1][X], self.line_2_vert[1][Y]), 
                 color, thickness_vert)



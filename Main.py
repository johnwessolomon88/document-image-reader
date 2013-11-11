import cv2
import copy
import os
import numpy as np

import DocVision as DocVSN

from scipy import ndimage

def main():
    img_file_path = 'Images/theft_article.png'

    path = os.path.join(os.getcwd(), img_file_path)
    im = cv2.imread(path)
    # imCpy is the output compy of the image
    imCpy = copy.deepcopy(im)

    Doc_Page = DocVSN.Document_Page(im)
    content_rgns_horiz = Doc_Page.GetContentRegionsHoriz()
    rgns_list = []

    for c in content_rgns_horiz:
        rgns = c.Get_Content_Rgns_From_Col_Sep_Gaps()
        rgns_list.append(rgns)    

    for i in xrange(len(rgns_list)):
        for r in rgns_list[i]:
            r.Draw_Rgn_On_Image(imCpy)
            print(str(i) + ': ' + str(r.line_1_vert) + ' --- ' + str(r.line_2_vert))

    cv2.imwrite('out_sample.png', imCpy)
    print 'END'

main()

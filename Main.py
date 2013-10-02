import cv2
import copy
import os
import numpy as np

import DocVision as DocVSN
import GeometryFunctions as GEO
import OutputWriter as OUT

from scipy import ndimage

X = 0
Y = 1

path = 'Images\TextSkew\s3.jpg'
im = cv2.imread(path)
imCpy = copy.deepcopy(im)

W = im.shape[1]
H = im.shape[0]

imGray = cv2.cvtColor(imCpy, cv2.COLOR_BGR2GRAY)

edges = cv2.Canny(copy.deepcopy(imGray), 150, 200)
edgesCopy = copy.deepcopy(edges)

outputWriter = OUT.OutputWriter(im, edges)
outputWriter.txtBlocks.append(OUT.TextBlock(im))

cnts, hiearchy = cv2.findContours(edgesCopy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

lines = DocVSN.GetHoughLinesP(edgesCopy, float(W) / float(10), float(W) / float(50))
angles = []
for l in lines:
    ang, length = GEO.GetLineAngleAndLen(l[0][X], l[0][Y], l[1][X], l[1][Y])
    angles.append(ang)

medianAngle = np.median(angles)
lRegress = DocVSN.SkewLinRegress(lines)

edgesCopy = copy.deepcopy(edges)

txtLines = DocVSN.GetContentRegionsHoriz(edgesCopy, medianAngle)

for i in range (len(txtLines)):
    print 'OCRing txtLine ' + str(i) + '.'
    imCpy = copy.deepcopy(im)
    txtLineIm = DocVSN.GetImgFromContentRegion(txtLines[i], imCpy)
    imContentRot = DocVSN.rotateImage(txtLineIm, medianAngle)
    txtLnOutputWriter = OUT.TextLine(imContentRot)
    outputWriter.txtBlocks[0].lines.append(txtLnOutputWriter)

    words = DocVSN.ContentScanVerticalForHorizBoundingLines(edgesCopy, txtLines[i], 50)
    for j in range (len(words)):
        imCpy = copy.deepcopy(im)
        imContent = DocVSN.GetImgFromContentRegion(words[j], imCpy)
        imContentRot = DocVSN.rotateImage(imContent, medianAngle)
        word = OUT.Word(imContentRot)
        outputWriter.txtBlocks[0].lines[i].words.append(word)

outputWriter.WriteFiles('OUTPUTWRITER')
print 'END'

import cv2
import os
import DocVision as DocVSN
from tesserwrap import Tesseract

TR = Tesseract()

class OutputWriter:
    def __init__(self, img, edgeImg):
        self.txtBlocks = []
        self.edgeImg = edgeImg
        self.img = img

    def WriteFiles(self, folderPath):
        if os.path.exists(folderPath) == False:
            os.mkdir(folderPath)

        txtBlockPath = folderPath + os.sep + 'TextBlocks'

        for t in range(len(self.txtBlocks)):
            print 'Writing data for txtBlock ' + str(t) + '.'

            if os.path.exists(txtBlockPath) == False:
                os.mkdir(txtBlockPath)

            cv2.imwrite(txtBlockPath + '\TextBlock_' + str(t) + '_IMG' + '.png', 
                        self.txtBlocks[t].img)

            f = open(txtBlockPath + os.sep + 'TextBlock_' + str(t) + '_OCR' + '.txt', 'wb+')

            f.write(self.txtBlocks[t].ocrText)
            for l in range(len(self.txtBlocks[t].lines)):
                lineFolder = txtBlockPath + os.sep + 'TextBlock_' + str(t) + '_Lines'

                if os.path.exists(lineFolder) == False:
                    os.mkdir(lineFolder)

                f = open(lineFolder + os.sep + 'Line_' + str(l) + '_OCR.txt', 'w+')
                f.write(self.txtBlocks[t].lines[l].ocrText)

                cv2.imwrite(lineFolder + os.sep + 'Line_' + str(l) + '_IMG.png', 
                            self.txtBlocks[t].lines[l].img)

                for w in range(len(self.txtBlocks[t].lines[l].words)): 
                    wordFolder = lineFolder + os.sep + 'Line_' + str(l) + '_Words'

                    if os.path.exists(wordFolder) == False:
                        os.mkdir(wordFolder)
                    
                    f = open(wordFolder + os.sep + 'word_' + str(w) + '_OCR.txt', 'w+')
                    f.write(self.txtBlocks[t].lines[l].words[w].ocrText)

                    cv2.imwrite(wordFolder + os.sep + 'word_' + str(w) + '.png', 
                                self.txtBlocks[t].lines[l].words[w].img)
class TextBlock:
    def __init__(self, img):
        self.lines = []
        self.img = img
        self.ocrText = TR.ocr_image(DocVSN.ConvertOpenCVToPIL(img)).encode('utf-8')

class TextLine:
    def __init__(self, img):
        self.img = img
        self.words = []
        self.ocrText = TR.ocr_image(DocVSN.ConvertOpenCVToPIL(img)).encode('utf-8')

class Word:
    def __init__(self, img):
        self.img = img
        self.ocrText = TR.ocr_image(DocVSN.ConvertOpenCVToPIL(img)).encode('utf-8')

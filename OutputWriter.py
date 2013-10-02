import cv2
import os
import DocVision as DocVSN
import pytesser

class OutputWriter:
    def __init__(self, img, edgeImg):
        self.txtBlocks = []
        self.edgeImg = edgeImg
        self.img = img

    def WriteFiles(self, folderPath):
        if os.path.exists(folderPath) == False:
            os.mkdir(folderPath)

        txtBlockPath = folderPath + '\TextBlocks'

        for t in range(len(self.txtBlocks)):
            print 'Writing data for txtBlock ' + str(t) + '.'

            if os.path.exists(txtBlockPath) == False:
                os.mkdir(txtBlockPath)

            cv2.imwrite(txtBlockPath + '\TextBlock_' + str(t) + '_IMG' + '.png', 
                        self.txtBlocks[t].img)

            file = open(txtBlockPath + '\TextBlock_' + str(t) + '_OCR' + '.txt', 'w+')
            file.write(self.txtBlocks[t].ocrText)

            for l in range(len(self.txtBlocks[t].lines)):
                lineFolder = txtBlockPath + '\TextBlock_' + str(t) + '_Lines'

                if os.path.exists(lineFolder) == False:
                    os.mkdir(lineFolder)

                file = open(lineFolder + '\Line_' + str(l) + '_OCR.txt', 'w+')
                file.write(self.txtBlocks[t].lines[l].ocrText)

                cv2.imwrite(lineFolder + '\Line_' + str(l) + '_IMG.png', 
                            self.txtBlocks[t].lines[l].img)

                for w in range(len(self.txtBlocks[t].lines[l].words)): 
                    wordFolder = lineFolder + '\Line_' + str(l) + '_Words'

                    if os.path.exists(wordFolder) == False:
                        os.mkdir(wordFolder)
                    
                    file = open(wordFolder + '\word_' + str(w) + '_OCR.txt', 'w+')
                    file.write(self.txtBlocks[t].lines[l].words[w].ocrText)

                    cv2.imwrite(wordFolder + '\word_' + str(w) + '.png', 
                                self.txtBlocks[t].lines[l].words[w].img)
class TextBlock:
    def __init__(self, img):
        self.lines = []
        self.img = img
        self.ocrText = pytesser.image_to_string(DocVSN.ConvertOpenCVToPIL(img))

class TextLine:
    def __init__(self, img):
        self.img = img
        self.words = []
        self.ocrText = pytesser.image_to_string(DocVSN.ConvertOpenCVToPIL(img))

class Word:
    def __init__(self, img):
        self.img = img
        self.ocrText = pytesser.image_to_string(DocVSN.ConvertOpenCVToPIL(img))
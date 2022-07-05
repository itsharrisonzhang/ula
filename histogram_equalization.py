
import os
import cv2

def ReadInFrames(flightVideo):
   vidcap = cv2.VideoCapture(flightVideo)
   success, image = vidcap.read()
   os.chdir("C:\\Users\\16269\\Desktop\\SRC\\Archive\\ULA\\ArduCAM Pictures\\Unequalized")
   count = 0
   while success:
       cv2.imwrite("frame%d.jpg" % count, image)  # save frame as JPEG file
       success, image = vidcap.read()
       print('Read a new frame: ', success)
       framesList.append(image)
       print(len(framesList))
       count += 1

def Equalize(frameLocation):
   img = cv2.imread(frameLocation, 1)
   # img = cv2.imread('images/retina.jpg', 1)

   # Converting image to LAB Color so CLAHE can be applied to the luminance channel
   lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

   # Splitting the LAB image to L, A and B channels, respectively
   l, a, b = cv2.split(lab_img)

   # plt.hist(l.flat, bins=100, range=(0,255))
   ########### Histogram Equalization #############
   # Apply histogram equalization to the L channel
   equ = cv2.equalizeHist(l)

   # plt.hist(equ.flat, bins=100, range=(0,255))
   # Combine the Hist. equalized L-channel back with A and B channels
   updated_lab_img1 = cv2.merge((equ, a, b))

   # Convert LAB image back to color (RGB)
   hist_eq_img = cv2.cvtColor(updated_lab_img1, cv2.COLOR_LAB2BGR)

   ###########CLAHE#########################
   # Apply CLAHE to L channel
   clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
   clahe_img = clahe.apply(l)
   # plt.hist(clahe_img.flat, bins=100, range=(0,255))

   # Combine the CLAHE enhanced L-channel back with A and B channels
   updated_lab_img2 = cv2.merge((clahe_img, a, b))

   # Convert LAB image back to color (RGB)
   CLAHE_img = cv2.cvtColor(updated_lab_img2, cv2.COLOR_LAB2BGR)

   # cv2.imshow("Original image", img)
   # cv2.imshow("Equalized image", hist_eq_img)
   # cv2.imshow('CLAHE Image', CLAHE_img)
   # cv2.waitKey(0)
   # cv2.destroyAllWindows()

   return CLAHE_img

def FindEqualizedList():
   os.chdir("C:\\Users\\16269\\Desktop\\SRC\\Archive\\ULA\\ArduCAM Pictures\\Equalized")
   for frameCounter in range(0, 1569):
       # Equalize param. must be a file location.
       FramePath = "C:\\Users\\16269\\Desktop\\SRC\\Archive\\ULA\\ArduCAM Pictures\\Unequalized\\frame" + str(frameCounter) + ".jpg"
       equalizedFramesList.append(Equalize(FramePath))
       cv2.imwrite("frame%d.jpg" % frameCounter, equalizedFramesList[frameCounter])  
# save frame as JPEG file
       print(len(equalizedFramesList))

flightVideo = "C:\\Users\\16269\\Desktop\\SRC\\Archive\\ULA\\ArduCAM Pictures\\Original.mp4"

# Compiled from original footage
framesList = []
# Compiled from equalized frames from framesList
equalizedFramesList = []
# Compiles framesList
ReadInFrames(flightVideo)
# Find equalized video from framesList
FindEqualizedList()


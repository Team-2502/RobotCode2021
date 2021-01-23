from cscore import CameraServer
from networktables import NetworkTables

import cv2
import json
import numpy as np
import time

blob_means=[(465.78973388671875,305.0594075520833),
            (571.5816853841146,245.081787109375),
            (449.79188028971356,306.5421396891276),
            (516.1250610351562,217.60052490234375)]

path_options = blob_means

def nothing(x):
    pass


def distance(x, y):
    return np.sqrt((x[0]-y[0])**2 + (x[1]-y[1])**2)


def main():
   width = 1280
   height = 720

   CameraServer.getInstance().startAutomaticCapture()

   input_stream = CameraServer.getInstance().getVideo()
   output_stream = CameraServer.getInstance().putVideo('Processed', width, height)

   # Table for vision output information
   NetworkTables.initialize(server='10.25.2.2')

   # Wait for NetworkTables to start
   time.sleep(0.5)
   
   nt = NetworkTables.getTable('SmartDashboard')
   nt.putNumber('lower_h', 0)
   nt.putNumber('lower_s', 215)
   nt.putNumber('lower_v', 235)

   nt.putNumber('upper_h', 245)
   nt.putNumber('upper_s', 255)
   nt.putNumber('upper_v', 255)

   

   img = np.zeros(shape=(height,width,3), dtype=np.uint8)

   while True:

      lower_h = nt.getNumber('lower_h', 0)
      lower_s = nt.getNumber('lower_s', 215)
      lower_v = nt.getNumber('lower_v', 235)

      upper_h = nt.getNumber('upper_h', 245)
      upper_s = nt.getNumber('upper_s', 255)
      upper_v = nt.getNumber('upper_v', 255)

      frame_time, img = input_stream.grabFrame(img)
      output_img = np.copy(img)

      # Notify output of error and skip iteration
      if frame_time == 0:
          # Send the output the error.
          output_stream.notifyError(input_stream.getError());
          # skip the rest of the current iteration
          continue

      # Convert to HSV and threshold image
      hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      binary_img = cv2.inRange(hsv_img, (65, 65, 200), (85, 255, 255))


      lower = np.array([lower_h, lower_s, lower_v])
      upper = np.array([upper_h, upper_s, upper_v])

      mask = cv2.inRange(img, lower, upper)

      kernel = np.ones((3, 3), np.uint8)
      opening_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

      points = cv2.findNonZero(mask)
      if cv2.countNonZero(mask) > 0:
          avg = np.mean(points, axis=0)
          print(avg)
      else:
          avg = [0,0]
      # displaying
      params = cv2.SimpleBlobDetector_Params()
      params.filterByColor = False
      params.filterByArea = True
      params.minArea = 5
      params.filterByCircularity = False
      params.filterByInertia = False
      params.filterByConvexity = False
      detector = cv2.SimpleBlobDetector_create(params)
      keypoints = detector.detect(opening_img)
      sumX = 0
      sumY = 0
      count = 0
      for kp in keypoints:
          count += 1
          sumX += kp.pt[0]
          sumY += kp.pt[1]
      print(f'Blobs: {count}')

      print(str(sumX / 3) + " " + str(sumY / 3))
      means = (sumX / 3, sumY / 3)
      im_with_keypoints = cv2.drawKeypoints(opening_img, keypoints, np.array([]), (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
      min_dist = -1
      index = -1
      for i in range(0, 4):
          dist = distance(path_options[i], means)
          if dist < min_dist or min_dist == -1:
              min_dist = dist
              index = i
      print(f'Path # {index + 1}')
      print(min_dist)

      nt.putNumber('Path', index + 1)
      nt.putNumber('Blobs', count)
      nt.putNumber('Average X', avg[0][0])
      nt.putNumber('Average Y', avg[0][1])
      
      output_stream.putFrame(mask)


if __name__ == '__main__':
      main()
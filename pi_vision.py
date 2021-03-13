from cscore import CameraServer
from networktables import NetworkTables

import cv2
import numpy as np
import time


def distance(x, y):
    return np.sqrt((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2)


def is_within_bounds(lx, hx, ly, hy, bgp):
    return (hx > bgp.pt[0] > lx) and (hy > bgp.pt[1] > ly)


def main():
    width = 1280
    height = 720

    CameraServer.getInstance().startAutomaticCapture()

    input_stream = CameraServer.getInstance().getVideo()
    output_stream = CameraServer.getInstance().putVideo('Processed', width, height)
    blob_stream = CameraServer.getInstance().putVideo('Blobs', width, height)

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

    img = np.zeros(shape=(height, width, 3), dtype=np.uint8)

    while True:

        lower_h = nt.getNumber('lower_h', 0)
        lower_s = nt.getNumber('lower_s', 215)
        lower_v = nt.getNumber('lower_v', 235)

        upper_h = nt.getNumber('upper_h', 245)
        upper_s = nt.getNumber('upper_s', 255)
        upper_v = nt.getNumber('upper_v', 255)

        frame_time, img = input_stream.grabFrame(img)

        # Notify output of error and skip iteration
        if frame_time == 0:
            # Send the output the error.
            output_stream.notifyError(input_stream.getError())
            # skip the rest of the current iteration
            continue

        lower = np.array([lower_h, lower_s, lower_v])
        upper = np.array([upper_h, upper_s, upper_v])

        mask = cv2.inRange(img, lower, upper)

        kernel = np.ones((3, 3), np.uint8)
        opening_img = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        opening_img = cv2.morphologyEx(opening_img, cv2.MORPH_OPEN, kernel)

        params = cv2.SimpleBlobDetector_Params()
        params.filterByColor = False
        params.filterByCircularity = False
        params.filterByArea = False
        params.filterByInertia = False
        params.filterByConvexity = False

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(opening_img)

        print("Num keypoints: " + str(len(keypoints)))

        keypoints = sorted(keypoints, key=lambda keypoint: keypoint.size, reverse=True)

        biggest_keypoints = []

        if len(keypoints) == 1:
            biggest_keypoints = [keypoints[0]]
            nt.putNumberArray('big1', keypoints[0].pt)
        if len(keypoints) == 2:
            biggest_keypoints = keypoints[:2]
            nt.putNumberArray('big1', keypoints[0].pt)
            nt.putNumberArray('big2', keypoints[1].pt)
        if len(keypoints) >= 3:
            biggest_keypoints = keypoints[:3]
            nt.putNumberArray('big1', keypoints[0].pt)
            nt.putNumberArray('big2', keypoints[1].pt)
            nt.putNumberArray('big3', keypoints[2].pt)

        im_with_keypoints = cv2.drawKeypoints(opening_img, biggest_keypoints, np.array([]), (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        """
        Paths:
        1 - Red Path A
        2 - Blue Path A
        3 - Red Path B
        4 - Blue Path B
        """
        if len(biggest_keypoints) != 0:
            if is_within_bounds(113, 133, 34, 54, biggest_keypoints[0]):
                nt.putNumber('Path Number', 4)
            elif is_within_bounds(145, 165, 35, 55, biggest_keypoints[0]):
                nt.putNumber('Path Number', 2)
            elif is_within_bounds(83, 103, 49, 69, biggest_keypoints[0]):
                nt.putNumber('Path Number', 1)
            elif is_within_bounds(0, 20, 46, 66, biggest_keypoints[0]):
                nt.putNumber('Path Number', 3)
            else:
                nt.putNumber('Path Number', 0)
        else:
            nt.putNumber('Path Number', 0)

        output_stream.putFrame(mask)
        blob_stream.putFrame(im_with_keypoints)


if __name__ == '__main__':
    main()

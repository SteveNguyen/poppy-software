import cv2
import time
import threading

import socket
import numpy as np



class Camera(threading.Thread):
    def __init__(self, camera_id, refresh_freq=25, daemon=True):
        """
        This class provides a wrapper for an opencv VideoCapture object.

        When started, it automatically retrieves frame at the desired frequency. The grab loop will run in background. The thread is set up as a daemon by default.

        """
        threading.Thread.__init__(self)
        self.daemon = daemon

        self.capture = cv2.VideoCapture(camera_id)

        self.delay = 1.0 / refresh_freq
        self._last_frame = None

        if self.capture.isOpened(): # try to get the first frame
            self.rval, frame = self.capture.read()
        else:
            self.rval = False



    @property
    def last_frame(self):
        """ Directly returns the last grabbed frame. """
        return self._last_frame

    def post_processing(self, img):
        """ Returns the image post processed. """
        return img

    def run(self):
        # while True:
            # if self.capture.grab():
            #     _, img = self.capture.retrieve()
            #     self._last_frame = self.post_processing(img)


        while self.rval:
            self.rval, img = self.capture.read()
            self._last_frame = self.post_processing(img)

            time.sleep(self.delay / 1000.)


class PSEyeCamera(Camera):
    def post_processing(self, img):
        return cv2.flip(img, -1)

class CameraStreamed(Camera):
    def __init__(self, camera_id, refresh_freq=25, daemon=True, ip = '127.0.0.1', port = 5001):
        Camera.__init__(self, camera_id, refresh_freq=25, daemon = daemon)

        self.cameraQuality = 75
        self.sock = socket.socket()
        self.sock.connect((ip, port))

    def post_processing(self, img):
        return cv2.flip(img, -1)


    def stream(self, img):
        cv2mat=cv2.imencode(".jpeg",img)
        JpegData=np.array(cv2mat[1]).tostring()
        self.sock.send( str(len(JpegData)).ljust(16))
        self.sock.send( JpegData)

if __name__ == '__main__':
    from numpy import zeros

    cam = Camera(0)

    cv2.imshow("window", zeros((10, 10, 3)))
    cv2.waitKey(10)

    cam.start()

    while True:
        if cam.last_frame is not None:
            cv2.imshow("window", cam.last_frame)
        if cv2.waitKey(25) == 27:
            break

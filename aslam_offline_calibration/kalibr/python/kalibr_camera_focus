#!/usr/bin/env python
print "importing libraries"
import rospy
import cv2
import cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import argparse

class CameraFocus:
    def __init__(self, topic):
        self.topic = topic
        self.windowNameOrig = "Camera: {0}".format(self.topic)
        cv2.namedWindow(self.windowNameOrig, 2)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.topic, Image, self.callback)
    
    def callback(self, msg):
        #convert image to opencv
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            np_image= np.array(cv_image)
        except CvBridgeError, e:
            print "Could not convert ros message to opencv image: ", e
            return
    
        #calculate the fft magnitude
        img_float32 = np.float32(np_image)
        dft = cv2.dft(img_float32, flags = cv2.DFT_COMPLEX_OUTPUT)
        dft_shift = np.fft.fftshift(dft)
        magnitude_spectrum = cv2.magnitude(dft_shift[:,:,0],dft_shift[:,:,1])
        
        #normalize
        magnitude_spectrum_normalized = magnitude_spectrum / np.sum(magnitude_spectrum)
        
        #frequency domain entropy (-> Entropy Based Measure of Camera Focus. Matej Kristan, Franjo Pernu. University of Ljubljana. Slovenia)
        fde = np.sum( magnitude_spectrum_normalized * np.log(magnitude_spectrum_normalized) )
    
        y = 20; x = 20
        text = "fde: {0}   (minimize this for focus)".format(np.sum(fde))
        np_image = cv2.cvtColor(np_image, cv2.COLOR_GRAY2BGR)
        cv2.putText(np_image, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0, 0, 255), thickness=2)
        cv2.imshow(self.windowNameOrig, np_image)
        cv2.waitKey(10)

if __name__ == "__main__":    
    parser = argparse.ArgumentParser(description='Validate the intrinsics of a camera.')
    parser.add_argument('--topic', nargs='+', dest='topics', help='camera topic', required=True)
    parsed = parser.parse_args()

    for topic in parsed.topics:
        camval = CameraFocus(topic) 
    
    rospy.init_node('kalibr_validator', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down" 
    
    cv.DestroyAllWindows()

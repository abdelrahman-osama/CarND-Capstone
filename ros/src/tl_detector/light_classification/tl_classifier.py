#from styx_msgs.msg import TrafficLight
from keras.preprocessing import image
import numpy as np
from darkflow.net.build import TFNet
from styx_msgs.msg import TrafficLight
import cv2


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        options = {"model": "cfg/tiny-yolo.cfg", "load": "tiny-yolo.weights", "threshold": 0.05, "gpu": 0.8}

        self.tfnet = TFNet(options)
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        
        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #image = image[0:300, 100:700]
        #TODO implement light color prediction
        #print(image.shape[0], "X")
        #print(image.shape[1], "Y")
        #print("----------------------before predict")
        result = self.tfnet.return_predict(image)
        #print("----------------------after predict")
        # define the list of boundaries
        boundaries = [
            ([0,200,0], [255,255,100], "green"), #green
            ([0,0,200], [50,50,255], "red"), #red
            ([0,210,210], [80,255,255], "yellow") #yellow
        ]

        max_frac = 0.0
        colorf = ""
        tl_detected = False

        
        #print("------------------start")
        for i in range(len(result)):
            if(result[i]["confidence"] > 0.3 and result[i]["label"] == "traffic light"):
                tl_detected = True
                xs = result[i]['topleft']['x']
                xe = result[i]['bottomright']['x']
                ys = result[i]['topleft']['y']
                ye = result[i]['bottomright']['y']
                crop_img = image[ys:ye, xs:xe]
                total = crop_img.shape[0]*crop_img.shape[1]
                for (lower, upper, color) in boundaries:
                    lower = np.array(lower, dtype = "uint8")
                    upper = np.array(upper, dtype = "uint8")
                    mask = cv2.inRange(crop_img, lower, upper)
                    c = np.sum(mask)//255
                    frac = c/total
                    if frac > max_frac and frac >= 0.01:
                        max_frac = frac
                        colorf = color
        print("------------------------", colorf)
        if(tl_detected):
            if(colorf == "red"):
                return TrafficLight.RED
            if(colorf == "green"):
                return TrafficLight.GREEN
            if(colorf == "yellow"):
                return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN



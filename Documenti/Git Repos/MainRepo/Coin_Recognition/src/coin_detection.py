#! /usr/bin/env python

import cv2
import numpy as np
import os 
from skimage import measure
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy

class CoinDetector:
    def __init__(self, image=None, imageWidth=800):
        if image is None:
            self.bridge_object = CvBridge()
            self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image ,self.camera_cb)
        else:
            self.image = cv2.imread(image)
        self.imageHeight = int((imageWidth / float(self.image.shape[1])) * self.image.shape[0])
        self.imageWidth = imageWidth
        self.sum = 0.0
    
    def resize_image(self, image, w=800, h=None):
        if w is not None and h is not None:
            return cv2.resize(image, (w, h))
        elif h is None:
            ratio = w / float(image.shape[1])
            return cv2.resize(image, (w, int(ratio * image.shape[0])))
        elif w is None:
            ratio = h / float(image.shape[0])
            return cv2.resize(image, (int(ratio * image.shape[1], h)))
        else:
            print("[ERROR] Insert either preferred height, preferred width or both.")
    
    def camera_cb(self, data):
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.image = cv_image
        
    def detectCoins(self):
        #resize image while keeping same ratio 
        img = cv2.resize(self.image, (self.imageWidth, self.imageHeight))

        #copy image
        original = img.copy()
        
        #blur the image and show it 
        blurred = cv2.GaussianBlur(img, (3, 3), 0)
        cv2.imshow("Original Resized image and Gaussian Blur", np.hstack([self.resize_image(img, w=300), 
            self.resize_image(blurred, w=300)]))

        #convert to HSV image and split into individual components 
        hsv_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_image)

        #show split channels image channels
        cv2.imshow("HSV Split", np.hstack([self.resize_image(h, w=300), self.resize_image(s, w=300), 
            self.resize_image(v, w=300)]))
    
        #create black image mask, apply floodfill and bitwise not so coins will be white 
        floodfill_mask = np.zeros((self.imageHeight + 2, self.imageWidth + 2), dtype="uint8")
        cv2.floodFill(s, floodfill_mask, (0, 0), 0, 2, 2, cv2.FLOODFILL_MASK_ONLY)
        #reverse the shape of the mask to the original one (floodfill adds 1 to every border)
        floodfill_mask = floodfill_mask[1:-1, 1:-1]
        floodfilled = cv2.bitwise_not(floodfill_mask * 255)

        #create circular kernel since coins are circular and will alter less their overall shape
        circular_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        #apply closing operation to close any gaps inside the coins and display 
        closed = cv2.morphologyEx(floodfilled, cv2.MORPH_CLOSE, circular_kernel, iterations=2)
     
        #find contours
        contours = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]

        #prune non-coin contours
        max_c = sorted(contours, key=cv2.contourArea, reverse=True)[0]
        max_area = cv2.contourArea(max_c)
        updated_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area == 0.0:
                continue
            if max_area / area < 6.0:
                updated_contours.append(c)

        #fill in borders with white to create the detected coins mask and show 
        numCoins = len(updated_contours)
        thresh = np.zeros((closed.shape), np.uint8)
        cv2.fillPoly(thresh, pts=(updated_contours), color=(255))
        cv2.imshow("Floodfill, Closing and Pruning Operations", np.hstack([self.resize_image(floodfilled, w=300), 
            self.resize_image(closed, w=300), self.resize_image(thresh, w=300)]))

        #connected component analysis to store each individual coin mask in separate matrix with 
        #same dimensions as original mask but with only one coin mask per image matrix 
        labels = measure.label(thresh, neighbors=8, background=0)
        coinMasks = np.zeros((len(updated_contours), thresh.shape[0], thresh.shape[1]), dtype="uint8")

        i = 0
        #loop over unique components
        for label in np.unique(labels):
            #ignore background labels 
            if label == 0:
                continue
            coinMasks[i, labels == label] = 255
            i += 1
        
        coinMasks_resized = [self.resize_image(coinMask, w=300) for coinMask in coinMasks]
        cv2.imshow("Individual Coin Masks", np.hstack(coinMasks_resized))

        #calculate diameters of each coin and store them in list 
        diameters = []
        for coinMask in coinMasks:
            contours = cv2.findContours(coinMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]
            c = max(contours, key=cv2.contourArea)
            min_x, max_x = np.min(c[:, 0, 0]), np.max(c[:, 0, 0])
            min_y, max_y = np.min(c[:, 0, 1]), np.max(c[:, 0, 1])
            diameter = (((max_x - min_x) / 2) + ((max_y - min_y) / 2) / 2)
            diameters.append(diameter)
            
        #HSV conversion
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        v = clahe.apply(v)
        hsv = cv2.merge([h, s, v])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        #HSV colour ranges to detect bronze, silver and gold coins separately 
        bronze_lower, bronze_upper = (0, 83, 56), (17, 211, 210)
        silver_lower, silver_upper = (0, 20, 47), (71, 100, 188)
        gold_lower, gold_upper = (14, 78, 46), (27, 201, 224)

        #applying the ranges to the hsv converted image 
        bronze_mask = cv2.inRange(hsv, bronze_lower, bronze_upper)
        silver_mask = cv2.inRange(hsv, silver_lower, silver_upper)
        gold_mask = cv2.inRange(hsv, gold_lower, gold_upper)

        #performing AND operation with the HSV mask to achieve binary images for different set of colors
        bronze_output = cv2.bitwise_and(thresh, thresh, mask=bronze_mask) 
        silver_output = cv2.bitwise_and(thresh, thresh, mask=silver_mask)
        gold_output = cv2.bitwise_and(thresh, thresh, mask=gold_mask) 
        
        #display original and clahe image 
        cv2.imshow("Original vs CLAHE on Value Channel of HSV", np.hstack([self.resize_image(img, w=300), 
            self.resize_image(bgr, w=300)]))

        #perform closing operation with circular kernel on golden mask so that the borders are definite and with no gaps
        circular_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        gold = cv2.morphologyEx(gold_output, cv2.MORPH_CLOSE, circular_kernel, iterations=2)
        gold_closed = gold.copy()
        #find contours and fill in mask so it will envelope whole coin not just golden part and display both transformations
        gold_contours = cv2.findContours(gold.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]
        cv2.fillPoly(gold, pts=(gold_contours), color=(255, 255, 255))
        #display 
        cv2.imshow("HSV Gold Mask, Closing and PolyFill Operations", np.hstack([self.resize_image(gold_output, w=300), 
            self.resize_image(gold_closed, w=300), self.resize_image(gold, w=300)]))

        #perform closing operation with circular kernel on golden mask so that the borders are definite and with no gaps
        bronze = cv2.morphologyEx(bronze_output, cv2.MORPH_CLOSE, circular_kernel, iterations=2)
        #display 
        cv2.imshow("HSV Bronze Mask and Closing Operations", np.hstack([self.resize_image(bronze_output, w=300), 
            self.resize_image(bronze, w=300)]))

        #perform closing operation with circular kernel on golden mask so that the borders are definite and with no gaps
        silver = cv2.morphologyEx(silver_output, cv2.MORPH_CLOSE, circular_kernel) 
        silver_closed = silver.copy()
        #remove the golden coin mask from the silver coin mask; this is because the silver is also
        #part of the golden coins and a could therefore identify a 1 or 2 pound as a silver coin 
        silver = cv2.subtract(silver, gold)
        #display both
        cv2.imshow("HSV Silver Mask, Closing and Arithmetic Operations", np.hstack([self.resize_image(silver_output, w=300), 
            self.resize_image(silver_closed, w=300), self.resize_image(silver, w=300)]))
        
        coinValue = 0.0
        max_diam, min_diam = np.max(diameters), np.min(diameters)
        for diameter, coinMask in zip(diameters, coinMasks):
            #counting color pixels for each coin
            g = cv2.countNonZero(cv2.bitwise_and(gold, coinMask))
            s= cv2.countNonZero(cv2.bitwise_and(silver, coinMask))
            b = cv2.countNonZero(cv2.bitwise_and(bronze, coinMask))

            #finding the center coordinate of each coin
            contours = cv2.findContours(coinMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]
            c = max(contours, key=cv2.contourArea)
            cv2.drawContours(original, [c], -1, (0, 0, 255), 2)
            M = cv2.moments(c)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            #subdiving the coin first in color categories and then recognizing it based on diameter 
            #ratio with largest and smallest diameter which correspond to 2 pound and 5p coins
            #for the code to work the 2 pound and 5p need to be present in the picture 
            if g > 1000 and g > s and g > b:
                gold_max_ratio = diameter / float(max_diam)
                if diameter == max_diam or gold_max_ratio > 0.90:
                    print("Golden 2 pound coin detected")
                    self.sum += 2.0
                    coinValue = 2.0
                else:
                    print("Golden 1 pound coin detected")
                    self.sum += 1.0
                    coinValue = 1.0
            elif s > 1000 and s > g and s > b:
                silver_max_ratio = diameter / float(max_diam)
                silver_min_ratio = diameter / float(min_diam) 
                if diameter == min_diam or silver_min_ratio < 1.10:
                    print("Silver 5p coin detected")
                    self.sum += 0.05
                    coinValue = 0.05
                elif silver_max_ratio > 0.90 or silver_min_ratio > 1.43 or diameter == max_diam:
                    print("Silver 50p coin detected")
                    self.sum += 0.50
                    coinValue = 0.50
                elif silver_max_ratio < 0.80 or silver_min_ratio < 1.25:
                    print("Silver 20p coin detected")
                    self.sum += 0.20
                    coinValue = 0.20
                else:
                    print("Silver 10p coin detected")
                    self.sum += 0.10
                    coinValue = 0.10
            elif b > 1000 and b > g and b > s:   
                bronze_max_ratio = diameter / float(max_diam)
                bronze_min_ratio = diameter / float(min_diam) 
                if bronze_max_ratio > 0.80 or bronze_min_ratio > 1.30:
                    print("Bronze 2p coin detected")
                    self.sum += 0.02
                    coinValue = 0.02
                else:
                    print("Bronze 1p coin detected")
                    self.sum += 0.01
                    coinValue = 0.01
            
            #put the value of the coin on the coordinates for the coin centre found previously
            cv2.putText(original, "%1.2f" %coinValue, (cx - 30, cy + 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, 0)
        
        #once looped over all coins, print the sum of the coins and the number of coins in the picture
        cv2.putText(original, "Final Coin Count is %1.2f" %self.sum, (original.shape[1] // 2, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, 0)
        cv2.putText(original, "No. of Coins Detected: %d" %numCoins, (original.shape[1] // 2 - 8, 70), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, 0)
        
        print("Sum of coins is %1.2f Pounds" %self.sum)

        cv2.imshow("Final Coin Count", original)
        cv2.waitKey(0)    
        cv2.destroyAllWindows()
        

rospy.init_node("coin_detection_node")
current_path = "/home/valeriofranchi/catkin_ws/src/assignment3"
image_file_path =  current_path + rospy.get_param('~image')
coinDetector = CoinDetector(image=image_file_path)
coinDetector.detectCoins()
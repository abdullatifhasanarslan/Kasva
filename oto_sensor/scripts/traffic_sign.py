#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import os
import cv2
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
class Sign:
	value=2
	dir_path = os.path.dirname(os.path.realpath(__file__)).replace("scripts","isaret")
	def __init__(self,name,file_name):
		self.name=name
		self.file_name=file_name
		self.value=Sign.value
		Sign.value+=1
		self.img =cv2.imread(Sign.dir_path+'/'+file_name,0)
                self.kp,self.des = SignDetector.sift.detectAndCompute(self.img,None)
                self.matchesMask=None
	def find_matches(self, flann, kp1, des1, image_msg):
		MIN_MATCH_COUNT = 9
		MIN_MSE_DECISION = 50000
		self.matches=flann.knnMatch(des1,self.des,k=2)
		self.good=[]
		for m,n in self.matches:
			if m.distance < 0.7*n.distance:
				self.good.append(m)

		if len(self.good)>MIN_MATCH_COUNT:
			src_pts = np.float32([ kp1[m.queryIdx].pt for m in self.good ]).reshape(-1,1,2)
			dst_pts = np.float32([ self.kp[m.trainIdx].pt for m in self.good ]).reshape(-1,1,2)

			M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
			self.matchesMask = mask.ravel().tolist()

			mse = self.fnCalcMSE(src_pts, dst_pts)
			if mse < MIN_MSE_DECISION:
				msg_sign = UInt8()
				msg_sign.data = self.value

				SignDetector.pub_traffic_sign.publish(msg_sign)

				rospy.loginfo(self.name)

                                return self.value
                        return self.value

		else:
			self.matchesMask = None
                        return None
	def fnCalcMSE(self, arr1, arr2):
			squared_diff = (arr1 - arr2) ** 2
			sum = np.sum(squared_diff)
			num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
			err = sum / num_all
			return err

class SignDetector:
	sift = cv2.xfeatures2d.SIFT_create()
	pub_traffic_sign = rospy.Publisher('/sign_number', UInt8, queue_size=1)
	pub_image_traffic_sign = rospy.Publisher('/detect/image_output/compressed', CompressedImage, queue_size = 1)
	def __init__(self,signs):
		self.fnPreproc()



		#subscribes and publishers
		self.sub_image_original = rospy.Subscriber('camera/image', Image, self.cbFindTrafficSign, queue_size = 1)


		self.cvBridge = CvBridge()
		self.TrafficSigns = signs

		self.counter = 1

	def fnPreproc(self):
		# Initiate SIFT object detector

		FLANN_INDEX_KDTREE = 0
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		search_params = dict(checks = 50)

		self.flann = cv2.FlannBasedMatcher(index_params, search_params)

	def cbFindTrafficSign(self, image_msg):
		# drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
		if self.counter % 3 != 0:
			self.counter += 1
			return
		else:
			self.counter = 1

		cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")



		# find the keypoints and descriptors with SIFT
		kp1, des1 = SignDetector.sift.detectAndCompute(cv_image_input,None)

                image_out_num = 1
		for sign in self.TrafficSigns:
                        a = sign.find_matches(self.flann,kp1,des1,image_msg)
                        if a!=None:
				image_out_num=a

		if image_out_num == 1:
			# publishes traffic sign image in compressed type
			self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_input, "jpg"))
		else:
                        #print "Resim bulundu",self.TrafficSigns[image_out_num-2].name
			match_sign=self.TrafficSigns[image_out_num-2]
			draw_params = dict(matchColor = (0,0,255), # draw matches in green color
							singlePointColor = None,
							matchesMask = match_sign.matchesMask, # draw only inliers
							flags = 2)

			final = cv2.drawMatches(cv_image_input,kp1,match_sign.img,match_sign.kp,match_sign.good,None,**draw_params)


			# publishes traffic sign image in compressed type
			self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, "jpg"))

	def main(self):
		rospy.spin()

traffic_signs = [Sign("giris_olmayan_yol","giris_olmayan_yol.png"),
				Sign("tasit_trafigine_kapali_yol","tasit_trafigine_kapali_yol.png"),
				Sign("ileri_ve_saga_mecburi_yon","ileri_ve_saga_mecburi_yon.png"),
				Sign("ileri_ve_sola_mecburi_yon","ileri_ve_sola_mecburi_yon.png"),
				Sign("ilerden_sola_mecburi_yon","ilerden_sola_mecburi_yon.png"),
				Sign("hiz_sinirlamasi_sonu_20_km_saat","hiz_sinirlamasi_sonu_20_km_saat.png"),
                Sign("azami_hiz_sinirlamasi_30_km_saat","azami_hiz_sinirlamasi_30_km_saat.png"),
				Sign("azami_hiz_sinirlamasi_20_km_saat","azami_hiz_sinirlamasi_20_km_saat.png"),
				Sign("ilerden_saga_mecburi_yon","ilerden_saga_mecburi_yon.png"),
				Sign("saga_donulmez","saga_donulmez.png"),
                Sign("sola_donulmez","sola_donulmez.png"),
                Sign("dur","dur.png"),
				Sign("park_yeri","park_yeri.png"),
                Sign("durak","durak.png"),]
				
if __name__ == '__main__':
	rospy.init_node('detect_sign')
	node = SignDetector(traffic_signs)
	node.main()

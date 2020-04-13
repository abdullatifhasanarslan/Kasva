import rospy
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Int8

plotx_left=None
plotx_right=None
left_fit=None
right_fit=None
average_left=None
average_right=None
LANE_THRESHOLD=1000
#reading image-----------------------------------
def manual_find(gray_image):
	number_of_windows=20
	window_height = np.int((gray_image.shape[0]/2) / number_of_windows)
	window_width = 50
	minimum_pixels_to_recenter = 50

	white = gray_image.nonzero()
	white_y = np.array(white[0])
	white_x = np.array(white[1])

	histogram = np.sum(gray_image[gray_image.shape[0] / 2:, :], axis=0)
	base = np.argmax(histogram)

	indices = []
	x_current =base
	for window in range(number_of_windows):
		window_y1 = gray_image.shape[0] - (window+1)*window_height
		window_y2 = gray_image.shape[0] - window*window_height

		window_x1 = x_current - window_width/2
		window_x2 = x_current + window_width/2
		
		# cv2.rectangle(lane_image, (window_x1, window_y1), (window_x2, window_y2), (0,255,0), 2)
		# cv2.rectangle(lane_image, (x_current-5, window_y1), (x_current+5, window_y2), (0,255,0), 2)
		
		good_lane_indices = ((window_y1 <= white_y) & (white_y < window_y2) & (window_x1 <= white_x) & (white_x < window_x2)).nonzero()[0]
		indices.append(good_lane_indices)
		
		#only if there is a meaningful amount of white pixels update center of the rectangle
		if len(good_lane_indices) > minimum_pixels_to_recenter:
			x_current = np.int(np.mean(white_x[good_lane_indices]))
	
	indices = np.concatenate(indices)
	x = white_x[indices]
	y = white_y[indices]
	ploty = np.linspace(gray_image.shape[0]/2, gray_image.shape[0] - 1, gray_image.shape[0]/2)
	fit = np.polyfit(y, x, 2)
	plotx = fit[0]*ploty**2 + fit[1]*ploty + fit[2]
	return fit, plotx
	
def auto_find(fit, gray_image):
	white = gray_image.nonzero()
	white_y = np.array(white[0])
	white_x = np.array(white[1])
	margin = 100
	indices = ( white_x > fit[0]*white_y**2 + fit[1]*white_y + fit[2] - margin ) & (white_x < fit[0]*white_y**2 + fit[1]*white_y + fit[2] + margin)

	# Again, extract line pixel positions
	y = white_y[indices]
	x = white_x[indices]

	# Fit a second order polynomial to each
	fit = np.polyfit(y, x, 2)

	# Generate x and y values for plotting
	ploty = np.linspace(gray_image.shape[0]/2, gray_image.shape[0] - 1, gray_image.shape[0]/2)
	plotx = fit[0]*ploty**2 + fit[1]*ploty + fit[2]
		
	return fit, plotx

def findLane(camera_input):
	global left_fit, right_fit
	global plotx_left, plotx_right
	global average_left, average_right
	global camera_subscriber, picture_publisher, cv_bridge
	try:
		lane_image = cv_bridge.imgmsg_to_cv2(camera_input, "bgr8")
	except CvBridgeError as e:
		print e

	height = lane_image.shape[0]
	width = lane_image.shape[1]
	# print width, height

	white_low = 200
	white_high = 255
	gray_image = cv2.cvtColor(lane_image,cv2.COLOR_RGB2GRAY)
	gray_image = cv2.inRange(gray_image, white_low, white_high)
	
	mask = np.zeros((height,width/2),dtype=np.uint8)
	gray_image_left = np.concatenate([ gray_image[:,:width/2], mask ], axis=1)
	gray_image_right = np.concatenate([ mask, gray_image[:,width/2:] ], axis=1)
	number_of_white_pixels_on_left = np.count_nonzero(gray_image_left)	
	number_of_white_pixels_on_right = np.count_nonzero(gray_image_right)	
	
	try:
		if number_of_white_pixels_on_left > LANE_THRESHOLD:
			left_fit, plotx_left = auto_find(left_fit, gray_image_left)
			average_left = np.append(average_left,np.array([left_fit]), axis=0)
		if number_of_white_pixels_on_right > LANE_THRESHOLD:
			right_fit, plotx_right = auto_find(right_fit, gray_image_right)	
			average_average = np.append(average_right,np.array([right_fit]), axis=0)
	except:
		if number_of_white_pixels_on_left > LANE_THRESHOLD:
			left_fit, plotx_left = manual_find(gray_image_left)
			average_left = np.array([left_fit])

		if number_of_white_pixels_on_right > LANE_THRESHOLD:
			right_fit, plotx_right = manual_find(gray_image_right)
			average_right = np.array([right_fit])

	ploty = np.linspace(lane_image.shape[0]/2, lane_image.shape[0]-1, lane_image.shape[0]/2)

	right_data = Float64()
	right_data.data = plotx_right.item(239)-gray_image.shape[1]/3
	lane_publisher_right.publish(right_data)
	cv2.polylines(lane_image, np.int_( np.dstack([plotx_right-gray_image.shape[1]/3,ploty]) ), isClosed=False, color=(255,0,0), thickness=1)

	left_data = Float64()
	left_data.data = plotx_left.item(239)+gray_image.shape[1]/3
	lane_publisher_left.publish(left_data)
	cv2.polylines(lane_image, np.int_( np.dstack([plotx_left+gray_image.shape[1]/3,ploty]) ), isClosed=False, color=(0,255,0), thickness=1)

	# AVERAGE_LENGTH = 5

	# left_fit = np.array([np.mean(average_left[::-1][:AVERAGE_LENGTH])])
	# right_fit = np.array([np.mean(average_right[::-1][:AVERAGE_LENGTH])])
	# if average_left.shape[0] > 1000:
	# 	average_left = average_left[-AVERAGE_LENGTH:]

	# if average_right.shape[0] > 1000:
	# 	average_right = average_right[-AVERAGE_LENGTH:]
	
	if number_of_white_pixels_on_left > LANE_THRESHOLD:
		cv2.polylines(lane_image, np.int_( np.dstack([plotx_left,ploty]) ), isClosed=False, color=(0,255,0), thickness=1)
	if number_of_white_pixels_on_right > LANE_THRESHOLD:
		cv2.polylines(lane_image, np.int_( np.dstack([plotx_right,ploty]) ), isClosed=False, color=(255,0,0), thickness=1)
	
	
	make_lane(lane_image, number_of_white_pixels_on_left, number_of_white_pixels_on_right)

	try:
		lane_image = cv_bridge.cv2_to_imgmsg(lane_image, "bgr8")
	except CvBridgeError as e:
		print e
	picture_publisher.publish(lane_image)

def make_lane(image, left_score, right_score): 
	global turning
	ploty = np.linspace(image.shape[0]/2, image.shape[0]-1, image.shape[0]/2)
	
	is_center_exists=True
	turning_data=Int8()
	if left_score > LANE_THRESHOLD and right_score > LANE_THRESHOLD:
		plotx_center = np.mean([plotx_left, plotx_right], axis=0)
		cv2.polylines(image, np.int_( np.dstack([plotx_center,ploty]) ), isClosed=False, color=(0,255,255), thickness=5)
		turning_data.data=0
	elif left_score <= LANE_THRESHOLD and LANE_THRESHOLD < right_score:
		plotx_center = np.subtract(plotx_right, image.shape[1]/3)
		cv2.polylines(image, np.int_( np.dstack([plotx_center,ploty]) ), isClosed=False, color=(0,255,255), thickness=5)
		turning_data.data=2
	elif right_score <= LANE_THRESHOLD and LANE_THRESHOLD < left_score:
		plotx_center = np.add(plotx_left, image.shape[1]/3)
		cv2.polylines(image, np.int_( np.dstack([plotx_center,ploty]) ), isClosed=False, color=(0,255,255), thickness=5)
		turning_data.data=1
	else:
		is_center_exists=False
		turning_data.data=3
	
	turning.publish(turning_data)
	if is_center_exists:
		center_data = Float64()
		center_data.data = plotx_center.item(239)
		lane_publisher.publish(center_data)


if __name__ == "__main__":
	rospy.init_node("detect_lane")
	cv_bridge = CvBridge()
	camera_subscriber = rospy.Subscriber('/camera/image', Image, findLane, queue_size = 1)
	lane_publisher = rospy.Publisher('/control/lane', Float64, queue_size = 1)
	lane_publisher_left = rospy.Publisher('/control/lane/left', Float64, queue_size = 1)
	lane_publisher_right = rospy.Publisher('/control/lane/right', Float64, queue_size = 1)
	turning = rospy.Publisher('/control/turning', Int8, queue_size = 1)
	picture_publisher = rospy.Publisher('/detect/image_output/lane', Image, queue_size = 1)
	rospy.spin()
	
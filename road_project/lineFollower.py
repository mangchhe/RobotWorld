#!/usr/bin/env python
#_*_coding:utf-8_*_
import rospy, cv2, cv_bridge, numpy, time, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RoadFollower:
	def __init__(self):

		self.bridge = cv_bridge.CvBridge()
		rospy.init_node('Road_Follower')
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image,self.image_callback)
		self.image_sub2 = rospy.Subscriber('cameraRight/rgb/image_raw',Image,self.image_callback2)
		self.image_sub3 = rospy.Subscriber('cameraLeft/rgb/image_raw',Image,self.image_callback3)
		self.scan_sub = rospy.Subscriber('scan',LaserScan,self.scan_callback)
		self.pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist,queue_size=1)
		self.twist = Twist()
		self.lower_yellow = numpy.array([0,218,218])		
		self.upper_yellow = numpy.array([255,255,255])
		self.lower_white = numpy.array([0,0,200])		
		self.upper_white = numpy.array([0,0,250])
		self.lower_red = numpy.array([0,70,50])		
		self.upper_red = numpy.array([10,255,255])
		self.is_stopBar = False
		self.is_stopBarSensor = True
		self.is_leftCameraSensor = False
		self.is_rightCameraSensor = False
		self.is_stopLine = True
		self.stopLineCount = 0
		self.is_stopLineActive = False
		self.leftDistance = 0.0
		self.rightDistance = 0.0
		self.startTime = 0
		self.range_ahead = 0.0
		self.is_stopObstacle = False
		self.is_stopSign = False
		self.stopSignCount = 0

	def scan_callback(self, msg): # 장애물 감지

		if self.is_stopObstacle:
			angle180 = len(msg.ranges)/2 # 180도
			angle45 = len(msg.ranges)/8 # 45도
			self.range_ahead = round(max(msg.ranges[angle180-angle45-angle45/2:angle180]),2)


	def image_callback(self, msg): # 정면 시야

		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		h, w, d = image.shape
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		if self.is_stopBarSensor:
			maskRed = cv2.inRange(hsv, self.lower_red, self.upper_red)
			maskRed[0:300,0:h] = 0
			M = cv2.moments(maskRed)
			if M['m00'] > 0:
				self.is_stopBar = False
			else:
				self.is_stopBar = True

		if self.is_leftCameraSensor:
			kernel = numpy.ones((9, 9), numpy.uint8)
			kernel2 = numpy.ones((3, 3), numpy.uint8)
			maskWhite = cv2.inRange(hsv, self.lower_white, self.upper_white)
			maskWhite[0:h/3*2,0:w] = 0
			maskWhite[h/2:h,0:w/5*1] = 0
			maskWhite[h/2:h,w/5*4:w] = 0
			maskD = cv2.erode(maskWhite, kernel, iterations = 2)
			maskD = cv2.erode(maskD, kernel2, iterations = 5)
			M = cv2.moments(maskD)
			if self.is_stopLine:
				if M['m00'] > 0:
					self.stopLineCount += 1
					print('＊＊＊＊＊＊＊정지선 만남 : {}＊＊＊＊＊＊＊'.format(self.stopLineCount))
					self.is_stopLine=False
					self.startTime = time.time()
					self.is_stopLineActive = True
					time.sleep(1)
			else:
				self.is_stopLineActive = False
				if time.time() - self.startTime > 12.0:
					self.is_stopLine=True

		if self.is_stopSign:
			img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			template = cv2.imread('/home/haju/catkin_ws/src/RosProject2_7.git/road_project/src/stop.png', 0)
			w, h = template.shape[::-1]
			res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)  # type: object
			threshold = 0.8
			loc = numpy.where(res >= threshold)
			for pt in zip(*loc[::-1]):
				self.stopSignCount+=1
				self.startTime = time.time()
				self.is_stopSign = False
				break
		else:
			if time.time() - self.startTime > 20:
				self.is_stopSign = True
		
	def image_callback2(self, msg): # 왼쪽 시야 

		if self.is_leftCameraSensor:
			image2 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
			h, w, d = image2.shape
			hsv = cv2.cvtColor(image2, cv2.COLOR_BGR2HSV)
			maskYellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
			maskWhite = cv2.inRange(hsv, self.lower_white, self.upper_white)
			maskYW=cv2.bitwise_or(maskYellow,maskWhite)
			M = cv2.moments(maskYW)

			if M['m00'] > 0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				self.leftDistance = cx - w / 2

	def image_callback3(self, msg): # 오른쪽 시야 

		if self.is_rightCameraSensor:
			image3 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
			h, w, d = image3.shape
			hsv = cv2.cvtColor(image3, cv2.COLOR_BGR2HSV)
			maskYellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
			maskWhite = cv2.inRange(hsv, self.lower_white, self.upper_white)
			maskYW=cv2.bitwise_or(maskYellow,maskWhite)
			M = cv2.moments(maskYW)

			if M['m00'] > 0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				self.rightDistance = cx - w / 2
	
	def waitStopBar(self): # 차단바 기다리기

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.is_stopBar: # 정지바
				print('차단바가 없어요')
				self.twist.linear.x = 1.0
				if time.time() - start > 2:
					self.is_stopBarSensor = False
					self.is_leftCameraSensor = True
					self.is_rightCameraSensor = True
					break
			else:
				print('차단바가 있어요')				
				start = time.time()
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
			rate.sleep()
			self.pub.publish(self.twist)
	
	def waitStopLine(self): # 정지선 기다리기
		if self.is_stopLineActive:
			if self.stopLineCount == 1:
				self.twist.linear.x = 0.0
				self.pub.publish(self.twist)
				time.sleep(3)
			elif self.stopLineCount == 4:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.pub.publish(self.twist)
				time.sleep(3)
				start = time.time()
				while not rospy.is_shutdown():					
					if time.time() - start > 4.5:
						break
					self.twist.linear.x = 1.2
					self.twist.angular.z = 0.1
					self.pub.publish(self.twist)
			elif self.stopLineCount == 5:
				start = time.time()
				while not rospy.is_shutdown():					
					if time.time() - start > 2.8:
						break
					self.twist.linear.x = 0.7
					self.twist.angular.z = -0.89
					self.pub.publish(self.twist)
			elif self.stopLineCount == 6:
				start = time.time()
				while not rospy.is_shutdown():					
					if time.time() - start > 9.0:
						break
					self.twist.linear.x = 0.7
					self.twist.angular.z = -.21
					self.pub.publish(self.twist)
			elif self.stopLineCount == 7:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.pub.publish(self.twist)
				time.sleep(3)
				start = time.time()
				while not rospy.is_shutdown():					
					if time.time() - start > 9.0:
						break
					self.twist.linear.x = 1.0
					self.twist.angular.z = 0.16
					self.pub.publish(self.twist)
			elif self.stopLineCount == 8:
				start = time.time()
				while not rospy.is_shutdown():					
					if time.time() - start > 2.2:
						break
					self.twist.linear.x = -0.5
					self.twist.angular.z = -1.1
					self.pub.publish(self.twist)
				start = time.time()
				while not rospy.is_shutdown():					
					if time.time() - start > 12.0:
						break
					self.twist.linear.x = 0.7
					self.twist.angular.z = - 0.035
					self.pub.publish(self.twist)
				start = time.time()
				while not rospy.is_shutdown():					
					if time.time() - start > 3.0:
						break
					self.twist.linear.x = 1.25
					self.twist.angular.z = -0.9
					self.pub.publish(self.twist)
			elif self.stopLineCount == 10:
				time.sleep(3)
				self.is_stopObstacle = True
				self.is_stopSign = True
			elif self.stopLineCount == 13:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.pub.publish(self.twist)
				start = time.time()
				while not rospy.is_shutdown():					
					if time.time() - start > 2.0:
						break
					self.twist.linear.x = 1.0
					self.twist.angular.z = 0.0
					self.pub.publish(self.twist)
				start = time.time()
				while not rospy.is_shutdown():					
					if time.time() - start > 10.0:
						break
					self.twist.linear.x = 0.7
					self.twist.angular.z = -.45
					self.pub.publish(self.twist)
			else:
				pass
		if self.stopSignCount == 1:
			time.sleep(3)
		elif self.stopSignCount == 2:
			print('주행 끝')
			time.sleep(1000)

	def loop(self): # 루프

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.range_ahead < 0.01 or math.isnan(self.range_ahead):
				self.waitStopLine()

				avg = - (self.leftDistance + self.rightDistance) / 180.0

				for i in range(50):
					if abs(avg) < 0.03 * i:
						self.twist.linear.x = 1 - (0.017 * i)
						break
					if i==49:
						self.twist.linear.x = 0.01
						break

				self.twist.angular.z = avg
			else:
				print('gd')
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
			self.pub.publish(self.twist)
			rate.sleep()
			

if __name__ == '__main__':
	
	time.sleep(3)
	roadFollower = RoadFollower()
	roadFollower.waitStopBar()
	roadFollower.loop()
	

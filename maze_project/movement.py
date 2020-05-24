#!/usr/bin/env python
#_*_coding:utf-8_*_

import rospy
import time
import math
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
from pathinfo import PathInfo

class MazeMovement:

	def __init__(self):
		self.roll = 0.0 ; self.pitch = 0.0 ; self.yaw = 0.0
		self.range_ahead = 0.0 ; self.range_right = 0.0 ; self.range_left = 0.0
		self.angle = 0.0
		self.xpos = 0.0 ; self.ypos = 0.0
		self.range_left_45 = 0.0
		self.range_left_135 = 0.0
		self.range_right_45 = 0.0
		self.range_right_135 = 0.0
		rospy.init_node('RoadMovement')
		self.odm_sub = rospy.Subscriber('/odom',Odometry,self.pose_callback)
		self.sen_sub = rospy.Subscriber('/scan',LaserScan,self.scan_callback)
		self.pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist,queue_size = 1)
		self.twist = Twist()
		self.pathinfo = PathInfo()
		self.finishXpos = abs(-7.92 - (7.95)) # 도착지 X좌표(출발지 바뀌면 변경해야함)
		self.finishYpos = abs(5.3 - (-5.15))  # 도착지 Y좌표(출발지 바뀌면 변경해야함)
		self.trash = 0

	def pose_callback(self,msg):
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)
		self.angle = math.degrees(self.yaw) + 180 % 360 + 90 if math.degrees(self.yaw) + 180 % 360 + 90 != 360 else 0 # (출발지 바뀌면 변경해야함)
		self.angle = 90 if self.angle == 450 else self.angle # (출발지 바뀌면 변경해야함)
		self.xpos = round(msg.pose.pose.position.x,1)
		self.ypos = round(msg.pose.pose.position.y,1)

	def scan_callback(self,msg):
		angle180 = len(msg.ranges)/2
		angle90 = len(msg.ranges)/4
		angle45 = len(msg.ranges)/8
		self.range_ahead = round(msg.ranges[angle180],2)
		self.range_right = round(msg.ranges[angle180-angle90],2)
		self.range_left = round(msg.ranges[angle180+angle90],2)
		self.range_left_45 = round(msg.ranges[angle180+angle90-angle45],2)
		self.range_left_135 =round(msg.ranges[angle180+angle90+angle45],2)
		self.range_right_45 = round(msg.ranges[angle180-angle90+angle45],2)
		self.range_right_135 = round(msg.ranges[angle180-angle90-angle45],2)

	def forward(self): # 전진 함수
		self.twist.linear.x = 1.0
		self.twist.angular.z = 0.0
		self.pub.publish(self.twist)
	
	def stop(self): # 정지 함수
		self.twist.linear.x = 0.0
		self.twist.angular.z = 0.0
		self.pub.publish(self.twist)

	def secForward(self): # 1초 이동 스타트 함수
		self.rotate(180)
		start = time.time()
		while not rospy.is_shutdown():
			self.twist.linear.x = 1.0
			self.pub.publish(self.twist)
			if time.time() - start > 1.0:
				self.twist.linear.x = 0.0
				self.pub.publish(self.twist)
				break
		time.sleep(.1)

	def marginOfError(self): # 회전가능한지 확인 함수
		avg = (self.range_left_45 + self.range_left_135 + self.range_right_45 + self.range_right_135) / 4
		error = abs(self.range_left_45 - avg)
		error2 = abs(self.range_left_135 - avg)
		error3 = abs(self.range_right_45 - avg)
		error4 = abs(self.range_right_135 - avg)
		if (error < 0.2 and error2 < 0.2 and error3 < 0.2 and error4 < 0.2) or self.range_ahead < 0.8: # 중간위치점 찾기
			return True
		else:
			return False

	def selectDirection(self,goal): # 회전 방향 결정 함수
		if self.angle == 0:
			now = 360
		else:
			now = self.angle
		if goal == 0:
			goal = 360
		direction = now + 270 if (now + 270) <= 360 else now + 270 - 360
		if goal == direction:
			return -1
		else:
			return 1

	def rotate(self,goal=-1): # 회전함수 (방향)
		direction = 0
		if goal==-1:	# 교차로가 아닐 경우
			if self.range_ahead > 1.5:
				destination = self.angle
				direction = -1
			if self.range_right > 1.5:
				destination = self.angle - 90 if self.angle - 90 >= 0 else 270
				direction = -1
			if self.range_left > 1.5:
				destination = self.angle + 90 if self.angle + 90 < 360 else 0
				direction = 1
		else:		# 교차로 일 경우
			direction = self.selectDirection(goal)
			destination = goal
		while True:
			self.twist.angular.z = direction * math.pi / 180 * 90
			if self.angle == destination:
				self.twist.angular.z = 0.0
				self.pub.publish(self.twist)
				break
			self.pub.publish(self.twist)
			time.sleep(0.1)

	def afterRotate(self): # 회전 후 함수
		rate = rospy.Rate(10)
		left = self.range_left
		right = self.range_right
		while not rospy.is_shutdown():
			if (right == self.range_right or left == self.range_left) and self.range_ahead > .7:
				self.forward()
			else:
				self.stop()
				break
			rate.sleep()

	def meetCurve(self): # 커브 만났을 때 행동(중복 제거)
		self.stop()
		cross=self.directionSearch()
		if cross:
			if self.pathinfo.popCrossLen() > 1: # 교차로 2개이상
				self.returnPath(1)
			else:
				self.pathinfo.removeCrossInfo()
				self.rotate(self.pathinfo.popCrossDirection())
		else:
			self.rotate()


	def directionSearch(self): # 방향(교차로) 찾기 함수
		direction = []
		if self.range_ahead > 1.5:
			direction.append(self.angle)
		if self.range_right > 1.5:
			direction.append(self.angle - 90 if self.angle - 90 > 0 else 270)
		if self.range_left > 1.5:
			direction.append(self.angle + 90 if self.angle + 90 < 360 else 0)
		if len(direction) > 1:
			self.pathinfo.recordXYpos(self.xpos,self.ypos)
			self.pathinfo.recordCrossXYpos(self.xpos,self.ypos,len(direction),direction)
			return True
		else:
			self.pathinfo.recordXYpos(self.xpos,self.ypos)
			return False

	def pointMove(self, xpos, ypos): # point to point 이동 함수
		rate = rospy.Rate(10)

		inc_x = xpos - self.xpos
		inc_y = ypos - self.ypos
		angle_to_goal = math.atan2(inc_x, -inc_y) # (출발지 바뀌면 변경해야함)
		angle_to_goal = math.degrees(angle_to_goal) + 180 % 360 if math.degrees(angle_to_goal) + 180 % 360 != 360 else 0
		direction = self.selectDirection(angle_to_goal)

		while not rospy.is_shutdown():
			
			if self.xpos == xpos and self.ypos == ypos:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.pub.publish(self.twist)
				self.pathinfo.pushPrePos(self.xpos,self.ypos) # 추가
				break
			
			if angle_to_goal != self.angle:
				self.twist.linear.x = 0.0
				self.twist.angular.z = direction * math.pi / 180 * 90

			elif angle_to_goal == self.angle:
				self.twist.linear.x = 1.0
				self.twist.angular.z = 0.0
			self.pub.publish(self.twist)
			rate.sleep()

	def returnPath(self,is_cross): # 왔던길 되돌아가는 함수
		x, y = self.pathinfo.popCrossTempXYpos()
		pre_x , pre_y = self.pathinfo.popPreXYpos()
		pos, self.trash = self.pathinfo.dfsPaths((x,y),(pre_x,pre_y))
		print('되돌아가기경로 : {}'.format(pos))

		while pos:
			x, y = pos.pop()
			self.pointMove(x,y)

		if is_cross: # 교차로를 만났을 경우
			self.pathinfo.pushCrossDirection(self.angle + 180 if self.angle + 180 < 360 else self.angle - 180)
			self.pathinfo.removeCrossInfo()
			self.rotate(self.pathinfo.popCrossDirection())
		else:				    # 막다른길
			self.rotate(self.pathinfo.popCrossDirection())
			self.pathinfo.removeCrossInfo()

	def reachFinishLine(self): # 결승점 만났을 때 행동
		degree = [0 for i in range(10)]
		minIndex = 0
		maxValue = 99999
		countPath = 0
		if self.finishXpos - 0.5 < self.xpos < self.finishXpos + 0.5 and self.finishYpos - 0.5 < - self.ypos < self.finishYpos + 0.5:
			print('결승점 도착')
			x, y = self.pathinfo.popXYrespository()
			pre_x , pre_y = self.pathinfo.popPreXYpos()
			pos, self.trash = self.pathinfo.dfsPaths((x,y),(pre_x,pre_y))

			if self.trash:
				pos = [pos]
			else:
				pass
			#최단 경로 계산 --------------------			
			pos[minIndex].insert(0,(0,0))
			for i in range(len(pos)):
				for j in range(len(pos[i])-1):
					if pos[i][j][0] != pos[i][j+1][0]:
						degree[i] += abs(pos[i][j][0] - pos[i][j+1][0])
					if pos[i][j][1] != pos[i][j+1][1]:
						degree[i] += abs(pos[i][j][1] - pos[i][j+1][1])
			for i in range(len(degree)):
				if degree[i] == 0:
					break
				countPath+=1
				if maxValue > degree[i]:
					maxValue = degree[i]
					minIndex = i
			#최단 경로 계산 --------------------
			print('도착지부터 출발지까지 경로 : {}'.format(pos))
			print('{}개의 경로중 선정되어진 경로 : {}'.format(countPath,pos[minIndex]))
			print('도착지점부터 시작지점까지의 거리 : {}'.format(degree[minIndex]))
			while pos[minIndex]:
				x, y = pos[minIndex].pop()
				self.pointMove(x,y)
			return 1
		else:
			return 0

	def loop(self): # 루프

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.reachFinishLine(): # 결승점 도달
				break
			if self.range_ahead > 0.7: # 직진
				self.forward()

			if self.marginOfError():   # 회전 할수 있는 위치 찾기
				if self.range_left > 1.5: # 왼쪽 회전
					self.meetCurve()
				elif self.range_right > 1.5: # 오른쪽 회전
					self.meetCurve()
				self.afterRotate()

			if self.range_ahead < .8 and self.range_right < 1.5 and self.range_left < 1.5: # 막다른길
				self.stop()
				self.returnPath(0)
				self.afterRotate()
			rate.sleep()

if __name__ == '__main__':
	mazeMovement = MazeMovement()
	mazeMovement.secForward()
	mazeMovement.loop()


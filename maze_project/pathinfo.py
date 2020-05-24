#!/usr/bin/env python
#_*_coding:utf-8_*_


import collections

class PathInfo(object):

	def __init__(self):
		self.xyRespository = collections.OrderedDict() # 모든 좌표 순서대로 저장
		self.start = True # 시작알림
		self.pre_xpos = 0.0 # 이전 x좌표
		self.pre_ypos = 0.0 # 이전 y좌표
		self.crossTempXYRespo = [] # 사거리 임시 XY좌표 저장소
		self.crossNum = [] # 각 사거리 통로 남은 횟수 저장
		self.crossDirection = [] # 사거리 가야되는 방향 저장
		self.crossXYRespo = [] # 사거리 XY좌표 저장소

	def dfsPaths(self, start, goal): # 출발지점부터 도착지점까지의 경우의 수
		stack = [(start, [start])]
		result = []

		while stack:
			n, path = stack.pop()
			if n == goal:
				result.append(path)
			else:  
				for temp in list(set(self.xyRespository[n])-set(path)):
					stack.append((temp, path + [temp]))
		if len(result) < 2: # 갈 수 있는 경웨 수가 한개 일 경우
			result = result.pop()
			return result, 1
		else:		    # 갈 수 있는 경우에 수가 두개 이상일 경우
			return result, 0

	def recordXYpos(self, xpos, ypos, pre_xpos = -1, pre_ypos = -1): # 현재 위치 저장
		if pre_xpos == -1: # 교차로 아닐 경우
			pre_xpos = self.pre_xpos
			pre_ypos = self.pre_ypos
		if self.start:     # 처음만
			self.xyRespository[(xpos,ypos)] = []
			self.start = False
			self.pre_xpos = xpos
			self.pre_ypos = ypos
		else:	           # 이후
			if (xpos,ypos) in self.xyRespository[(pre_xpos,pre_ypos)]:  # 같은 값 검사
				pass
			else:
				self.xyRespository[(pre_xpos,pre_ypos)].append((xpos,ypos))
			try:	# 같은 값 검사
				if self.xyRespository[(xpos,ypos)]:
					pass
			except KeyError:
				self.xyRespository[(xpos,ypos)] = []
			self.pre_xpos = xpos
			self.pre_ypos = ypos

	def recordCrossXYpos(self, xpos, ypos, crossnum, crossdirection): # 교차로 정보들 저
		if (xpos,ypos) in self.crossTempXYRespo: # 같은 좌표 있는지 확인
			pass
		else:
			self.crossTempXYRespo.append((xpos, ypos))
			self.crossNum.append(crossnum)
			self.crossDirection.append(crossdirection)

	def pushPrePos(self,xpos,ypos): # 현재 좌표 전 X,Y좌표 저장
		self.pre_xpos = xpos
		self.pre_ypos = ypos

	def pushCrossDirection(self,direction): # 교차로의 움직일 수 있는 방향과 방향 갯수 저장
		self.crossDirection[0].append(direction)
		self.crossNum[0]+=1

	def popCrossTempXYpos(self): # 교차로 임시좌표 반환(들러야 될 교차로의 좌표)
		return self.crossTempXYRespo[0]

	def popCrossDirection(self): # 교차로 방향 반환(교차로에서의 회전 방향)
		direction = self.crossDirection[0][0]
		del self.crossDirection[0][0]
		return direction

	def popPreXYpos(self): # 현재 좌표 전 X,Y 좌표 반환(전체적인 좌표 순서도 작성에 필요)
		return self.pre_xpos, self.pre_ypos

	def popXYrespository(self): # 현재 다녀간 좌표중 첫번째 들린 좌표 반환(결승점과의 거리)
		return self.xyRespository.keys().pop(0)

	def popCrossLen(self): # 들러야 할 교차로 갯수 반환(미로 통과시 막다른길, 교차로 구분)
		return len(self.crossNum)

	def removeCrossInfo(self): # 교차로를 만났을 경우 정보 정리
		self.crossNum[0]-=1
		try:
			if self.crossNum[0] < 1:
				del self.crossDirection[0]
				del self.crossNum[0]
				del self.crossTempXYRespo[0]
		except IndexError:
			pass



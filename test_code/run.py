#!/usr/bin/env python

import rospy
import cv2
import time
import numpy as np
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage

import turtle_video_siljun

rospy.init_node('zara', anonymous=True)
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=5)
pub_stage=rospy.Publisher('/stage',Int8,queue_size=5)


stage=100 #start from sinho stage
angular=0


timer=0
delay=0


T_finish=0
jucha_finish=0
chadan_finish=0
obstacle_finish=0


lower_red=np.array([165,160,115]) 
upper_red=np.array([185,255,255])

lower_green=np.array([25,100,100])
upper_green=np.array([85,255,255])

lower_blue=np.array([90,100,100]) 
upper_blue=np.array([130,255,255])

lower_white=np.array([0,0,200]) ### HSV range used in white detect
upper_white=np.array([180,15,255])

lower_yellow=np.array([20,100,100])
upper_yellow=np.array([30,255,255])


def turtlestop():

	twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

def turtlemove(linear,angular):

	rospy.on_shutdown(turtlestop)
	#print(angular)
	twist=Twist()
	twist.linear.x=linear
	twist.angular.z=angular
	pub.publish(twist)

def linetrace(data):
	global angular, stage, delay, T_finish, angular_prev, timer
	i=0	
	R_deg =[]
	L_deg =[]

	np_arr = np.fromstring(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

	right_eye=frame[0:360,320:640]
	upper_right_eye=frame[0:90,480:640]
	left_eye=frame[0:360,0:320]

	left_hsv=hsv[0:360,0:320]
	right_hsv=hsv[0:360,320:640]
	upper_right_hsv=hsv[0:90,480:640]

	ROI=frame[270:360,0:640]
	left_ROI=ROI[:,:ROI.shape[1]/2] 
	right_ROI=ROI[:,ROI.shape[1]/2:] 

	gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	median=cv2.medianBlur(gray,3)
	gray_blurred=cv2.GaussianBlur(median,(3,3),0)
	ret,thresh = cv2.threshold(gray_blurred,180,255,cv2.THRESH_BINARY)
	#ret,thresh = cv2.threshold(gray_blurred,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	#thresh = cv2.adaptiveThreshold(gray_blurred,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)

	roi=thresh[270:360,0:640] 
	
	edge = cv2.Canny(roi,180,360)
	
	left_edge=edge[:,:edge.shape[1]/2] 
	right_edge=edge[:,edge.shape[1]/2:] 

	L_lines=cv2.HoughLines(left_edge,1,np.pi/180,60)
	R_lines=cv2.HoughLines(right_edge,1,np.pi/180,60)

	if L_lines is not None:
		L_lines=[l[0] for l in L_lines]
		#print('L_lines', L_lines)
		
	    	for rho, theta in L_lines:
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a * rho
			y0 = b * rho
			l_x1 = int(x0 + 1000 * (-b))
			l_y1 = int(y0 + 1000 * (a))
			l_x2 = int(x0 - 1000 * (-b))
			l_y2 = int(y0 - 1000 * (a))

			#print('L_x0',x0,'L_yo',y0)
		
			L_degree=np.arctan2(l_y2-l_y1,l_x2-l_x1)*180/np.pi
			#print('L_degree',L_degree)
			if L_degree < -1:
				L_deg.append(L_degree)

			#print('L_deg',L_deg)
			
			#cv2.circle(ROI,(x0,y0),5,(255,0,0),2)
		

	if R_lines is not None:
		R_lines=[l[0] for l in R_lines]
		#print('R_lines', R_lines)
		for rho,theta in R_lines:
	    		a = np.cos(theta)
	    		b = np.sin(theta)
	    		x0 = a*rho
	    		y0 = b*rho
			r_x1 = int(x0 + 1000 * (-b))
			r_y1 = int(y0 + 1000 * (a))
			r_x2 = int(x0 - 1000 * (-b))
			r_y2 = int(y0 - 1000 * (a))

			#print('R_x0',x0,'R_yo',y0)

			R_degree=np.arctan2(r_y2-r_y1,r_x2-r_x1)*180/np.pi
			#print('R_degree', R_degree)
			if R_degree > 1:
				R_deg.append(R_degree)


	#print('R_degree',R_degree)
	#print('L_degree',L_degree)
	#print('R_deg',R_deg)
	#print('L_deg',L_deg)
	#print('R_index',R_deg.index(R_degree))
	#print('L_index',L_deg.index(L_degree))
	

	if len(L_deg) != 0:
		L_degree = max(L_deg)
		rho, theta = L_lines[L_deg.index(L_degree)]
		a = np.cos(theta)
		b = np.sin(theta)
		x0 = a * rho
		y0 = b * rho
		l_x1 = int(x0 + 1000 * (-b))
		l_y1 = int(y0 + 1000 * (a))
		l_x2 = int(x0 - 1000 * (-b))
		l_y2 = int(y0 - 1000 * (a))
		cv2.line(left_ROI, (l_x1, l_y1), (l_x2, l_y2), (0, 0, 255), 2)
		i+=1

	if len(R_deg) != 0:
		R_degree = min(R_deg)
		rho, theta = R_lines[R_deg.index(R_degree)]
	    	a = np.cos(theta)
		b = np.sin(theta)
		x0 = a*rho
		y0 = b*rho
		r_x1 = int(x0 + 1000 * (-b))
		r_y1 = int(y0 + 1000 * (a))
		r_x2 = int(x0 - 1000 * (-b))
		r_y2 = int(y0 - 1000 * (a))
		cv2.line(right_ROI, (r_x1, r_y1), (r_x2, r_y2), (0, 255, 0), 2)
		i+=1
	

	if i==2:
		angular = (L_degree + R_degree)*0.01
		cv2.putText(frame, '2', (120, 140), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 0, 0), 2)
		
		#print('L_degree',L_degree,'R_degree',R_degree)
		#print('angular',angular)
	

	elif i==1:
		if len(L_deg) != 0:
			if L_degree<=-40:
				angular = -(L_degree + 90)*0.010
			elif -40<L_degree and L_degree<=-35:
				angular = -(L_degree + 90)*0.011
			elif -35<L_degree and L_degree<=-30:
				angular = -(L_degree + 90)*0.012
			elif -30<L_degree and L_degree<=-25:
				angular = -(L_degree + 90)*0.013
			elif -25<L_degree and L_degree<=-20:
				angular = -(L_degree + 90)*0.014
			elif -20<L_degree and L_degree<=-15:
				angular = -(L_degree + 90)*0.015
			else:
				angular = -(L_degree + 90)*0.016
			cv2.putText(frame, '1', (120, 140), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 0, 0), 2)
			
			
			#print('L_degree',L_degree)
			#print('left_angular',angular)
		else:
			if R_degree>40:
				angular = -(R_degree - 90)*0.010
			elif 35<R_degree and R_degree<=40:
				angular = -(R_degree - 90)*0.011
			elif 30<R_degree and R_degree<=35:
				angular = -(R_degree - 90)*0.012
			elif 25<R_degree and R_degree<=30:
				angular = -(R_degree - 90)*0.013
			elif 20<R_degree and R_degree<=25:
				angular = -(R_degree - 90)*0.014
			elif 15<R_degree and R_degree<=20:
				angular = -(R_degree - 90)*0.015
			else:
				angular = -(R_degree - 90)*0.016
			cv2.putText(frame, '1', (120, 140), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 0, 0), 2)

	else:				
		angular = 0



	if stage ==100 or stage==10010 or stage==10011 or stage==111 or stage==1111:

		turtlemove(0.15,angular)

		if stage==111:

			timer+=1
			print('timer',timer)

			mask_blue=cv2.inRange(upper_right_hsv,lower_blue,upper_blue)
			blue=cv2.bitwise_and(upper_right_eye,upper_right_eye,mask=mask_blue)
			blue_gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
			blue_gray_blurred = cv2.GaussianBlur(blue_gray, (5, 5), 0)
			ret_b,thresh_b = cv2.threshold(blue_gray_blurred,0,255,cv2.THRESH_BINARY)

			_, blue_contours, hierarchy = cv2.findContours(thresh_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

			blue_x=[]
			blue_y=[]

			

			for c in blue_contours:

				peri = cv2.arcLength(c, True)
				approx = cv2.approxPolyDP(c, 0.04 * peri, True)
				(x, y, w, h) = cv2.boundingRect(approx)
				end = x + w

				#print('x',x,'y',y,'w',w,'h',h)
		
				if w>30 and w<160 and h>30 and h<90 :
					blue_x.append(x)
					blue_y.append(y)
					cv2.drawContours(upper_right_eye, [c], -1, (255, 0, 0), 3)

			#print("len(blue_x)",len(blue_x),"nn",nn)

			if len(blue_x)>0 and timer>50:
			
				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.3))

				turtlemove(1,0)
				rospy.sleep(rospy.Duration(1))

				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.3))

				turtlemove(0,1.5)
				rospy.sleep(rospy.Duration(0.8))

				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.3))

				turtlemove(1,0)
				rospy.sleep(rospy.Duration(0.2))

				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.3))

				stage=1111
			
			cv2.imshow('upper_right_eye',upper_right_eye)

				
			

		if stage==10010:
			delay += 1
			

			mask_blue=cv2.inRange(right_hsv,lower_blue,upper_blue)
			blue=cv2.bitwise_and(right_eye,right_eye,mask=mask_blue)
			blue_gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
			blue_gray_blurred = cv2.GaussianBlur(blue_gray, (5, 5), 0)
			ret_b,thresh_b = cv2.threshold(blue_gray_blurred,0,255,cv2.THRESH_BINARY)

			_, blue_contours, hierarchy = cv2.findContours(thresh_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

			blue_x=[]
			blue_y=[]

			for c in blue_contours:

				peri = cv2.arcLength(c, True)
				approx = cv2.approxPolyDP(c, 0.04 * peri, True)
				(x, y, w, h) = cv2.boundingRect(approx)
				end = x + w

				print('x',x,'y',y,'w',w,'h',h)
		
				if w>3 and w<30 and h>3 and h<30 and y>150 and x>300:
					blue_x.append(x)
					blue_y.append(y)
					cv2.drawContours(frame, [c], -1, (255, 0, 0), 3)

			print("len(blue_x)",len(blue_x),"nn",nn)

			if len(blue_x)==1 and delay>20 and nn>0:
				print("Exit!!!")
				print("delay",delay)
				#turtlemove(1,3)
				#rospy.sleep(rospy.Duration(0.8))

				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.2))

				turtlemove(1,0)
				rospy.sleep(rospy.Duration(0.9))

				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.2))

				turtlemove(0,1)
				rospy.sleep(rospy.Duration(1.1))

				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.2))

				turtlemove(1,0)
				rospy.sleep(rospy.Duration(0.2))

				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.2))

				stage=100
				T_finish=1
			
			

		if stage==10011:

			delay += 1

			mask_blue=cv2.inRange(left_hsv,lower_blue,upper_blue)
			blue=cv2.bitwise_and(left_eye,left_eye,mask=mask_blue)
			blue_gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
			blue_gray_blurred = cv2.GaussianBlur(blue_gray, (5, 5), 0)
			ret_b,thresh_b = cv2.threshold(blue_gray_blurred,0,255,cv2.THRESH_BINARY)

			_, blue_contours, hierarchy = cv2.findContours(thresh_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

			blue_x=[]
			blue_y=[]

			for c in blue_contours:

				peri = cv2.arcLength(c, True)
				approx = cv2.approxPolyDP(c, 0.04 * peri, True)
				(x, y, w, h) = cv2.boundingRect(approx)
				end = x + w

				#print('x',x,'y',y,'w',w,'h',h)
		
				if w>3 and w>30 and h>3 and h<30 and x <20 and y>150 :
					blue_x.append(x)
					blue_y.append(y)
					cv2.drawContours(frame, [c], -1, (255, 0, 0), 3)

			if len(blue_x)==1 and delay>20 and n>0:
				print("Exit!!!")
				#turtlemove(1,-2)
				#rospy.sleep(rospy.Duration(1))
				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.2))

				turtlemove(1,0)
				rospy.sleep(rospy.Duration(1))

				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.2))

				turtlemove(0,-1)
				rospy.sleep(rospy.Duration(1.25))

				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.2))

				turtlemove(1,0)
				rospy.sleep(rospy.Duration(0.2))	

				turtlemove(0,0)
				rospy.sleep(rospy.Duration(0.2))
			
				stage=100
				T_finish=1

			
		
	#cv2.imshow('roi', roi)		
	cv2.imshow('thresh', thresh)
	#cv2.imshow('edge', edge)
	cv2.imshow('frame', frame)
	cv2.waitKey(1)

        	##############################################################################################################################################################

def traffic_sign(data):
	global stage, jucha_finish;

	parking_start=0

	np_arr = np.fromstring(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

	right_eye=frame[0:360,320:640]
	left_eye=frame[0:360,0:320]

	mask_blue=cv2.inRange(hsv,lower_blue,upper_blue)
	blue=cv2.bitwise_and(frame,frame,mask=mask_blue)

	blue_gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
	blue_gray_blurred = cv2.GaussianBlur(blue_gray, (5, 5), 0)

	ret_b,thresh_b = cv2.threshold(blue_gray_blurred,0,255,cv2.THRESH_BINARY)

	_, blue_contours, hierarchy = cv2.findContours(thresh_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	_, right_blue_contours, hierarchy = cv2.findContours(thresh_b[0:180,320:640], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


	blue_x=[]
	blue_y=[]

	exit_jucha=[]

	jucha_sign=0

	for c in blue_contours:

		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
		(x, y, w, h) = cv2.boundingRect(approx)
		end=x+w
		#print('x',x,'y',y,'w',w,'h',h,'len(approx',len(approx))

		
		
		if w>40 and w<100 and x>50 and x<300 and h<90 and w*h<4000 and end<400 and len(approx)!=6:
			print('x',x,'y',y,'w',w,'h',h,'len(approx',len(approx))
			blue_x.append(x)
			blue_y.append(y)
			cv2.drawContours(frame, [c], -1, (255, 0, 0), 3)

		if y==0 and x<400 and h<50 and w<80 and h>30 and w>70:
			#cv2.drawContours(frame, [c], -1, (255, 0, 0), 3)

			#print('x',x,'y',y,'w',w,'h',h)
			exit_jucha.append(x)	


	#print("len(blue_x)",len(blue_x))
		


	if stage == 100: #stage select

		for c in right_blue_contours:

			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.04 * peri, True)
			(x, y, w, h) = cv2.boundingRect(approx)

			#print('x',x,'y',y,'w',w,'h',h)
			#cv2.drawContours(frame, [c], -1, (255, 0, 0), 3)
			if w>50 and w<80 and h>50 and h<80 and y<100:
			
				jucha_sign+=1
				#cv2.drawContours(frame, [c], -1, (255, 0, 0), 3)

		if jucha_sign==1 and T_finish==1 and nnnn==0 and nn==0 and dist_chadan>50 and jucha_finish==0:

			turtlemove(0,0)
			rospy.sleep(rospy.Duration(0.3))
			
			stage=111

		elif len(blue_x)==2 and T_finish==0 and nnnn>=5 and abs(blue_y[0]-blue_y[1])>70 and blue_y[0]*blue_y[1]==0:
			#cv2.imshow('T_detected',frame)
			#cv2.waitKey(0)
			if blue_x.index(max(blue_x))==blue_y.index(max(blue_y)):
				stage=10 #left in T
			else:
				stage=11 #right in T
			#print('T')

	if stage==10: #turn left
		#turtlemove(1,3)
		#rospy.sleep(rospy.Duration(0.6))
		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.3))

		turtlemove(0.15,0)
		rospy.sleep(rospy.Duration(1.3))

		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.2))

		turtlemove(0,1.3)
		rospy.sleep(rospy.Duration(1.2))

		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.2))

		turtlemove(1,0)
		rospy.sleep(rospy.Duration(0.2))

		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.2))

		stage=10010

	if stage==11: # turn right
		#turtlemove(1,-3)
		#rospy.sleep(rospy.Duration(0.6))
		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.3))

		turtlemove(0.15,0)
		rospy.sleep(rospy.Duration(1.3))

		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.2))

		turtlemove(0,-1.3)
		rospy.sleep(rospy.Duration(0.9))

		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.2))

		turtlemove(1,0)
		rospy.sleep(rospy.Duration(0.2))

		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.2))
		
		stage=10011



	if stage == 0: #sinho
		turtlemove(0,0)
		print('sinho!')
		keypoints_green=turtle_video_siljun.find_color(right_eye,lower_green,upper_green,stage)
		if keypoints_green:		
			print('green signal detected.')
			stage=100

	if stage==1111: #jucha		

		deg=[]
		dot_line=0

		bottom=frame[180:360,320:640]
		gray=cv2.cvtColor(bottom,cv2.COLOR_BGR2GRAY)
		median=cv2.medianBlur(gray,3)
		gray_blurred=cv2.GaussianBlur(median,(3,3),0)
		ret,thresh = cv2.threshold(gray_blurred,200,255,cv2.THRESH_BINARY)

		_, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(bottom, contours, -1, (0, 255, 0), 3) 

		#print('len(contours)',len(contours))

		if len(contours)==2:

			for c in contours:

				peri = cv2.arcLength(c, True)
				approx = cv2.approxPolyDP(c, 0.04 * peri, True)

				#print('approx',approx)

				if len(approx) == 4:

					(x, y, w, h) = cv2.boundingRect(approx)
					ar = w / float(h)
					#print('x',x,'y',y,'w',w,'h',h,'ar',ar)
					end = x + w
					if ar>1 and end <320:
						dot_line += 1
						cv2.drawContours(bottom, [c], -1, (255, 0, 0), 2)
				
			if dot_line>=2 and jucha_finish==0:

				cv2.putText(bottom, '2', (120, 140), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 0, 0), 2)
				if parking>0:
					parking_right=1
				else:
					parking_right=0

				if parking_right==1:

					turtlemove(0,0)	
					rospy.sleep(rospy.Duration(0.3))
					turtlemove(1,0)
					rospy.sleep(rospy.Duration(0.7))
					turtlemove(0,0)	
					rospy.sleep(rospy.Duration(0.3))
					turtlemove(1,-3)
					rospy.sleep(rospy.Duration(1.2))
					turtlemove(0,0)	
					rospy.sleep(rospy.Duration(0.3))
					turtlemove(1,0)
					rospy.sleep(rospy.Duration(1.2))
					turtlemove(0,0)
					rospy.sleep(rospy.Duration(0.3))
					turtlemove(-1,0)
					rospy.sleep(rospy.Duration(1.2))
					turtlemove(0,0)	
					rospy.sleep(rospy.Duration(0.3))
					turtlemove(-1,-3)
					rospy.sleep(rospy.Duration(1.2))
					turtlemove(0,0)	
					rospy.sleep(rospy.Duration(0.3))

					jucha_finish=1

				else:
					turtlemove(0,0)	
					rospy.sleep(rospy.Duration(0.3))
					turtlemove(1,0)
					rospy.sleep(rospy.Duration(0.7))
					turtlemove(0,0)	
					rospy.sleep(rospy.Duration(0.3))
					turtlemove(1,3)
					rospy.sleep(rospy.Duration(1.1))
					turtlemove(0,0)	
					rospy.sleep(rospy.Duration(0.3))
					turtlemove(1,0)
					rospy.sleep(rospy.Duration(1.3))
					turtlemove(0,0)
					rospy.sleep(rospy.Duration(0.3))
					turtlemove(-1,0)
					rospy.sleep(rospy.Duration(1.2))
					turtlemove(0,0)	
					rospy.sleep(rospy.Duration(0.3))
					turtlemove(-1,3)
					rospy.sleep(rospy.Duration(1.1))
					turtlemove(0,0)	
					rospy.sleep(rospy.Duration(0.3))

					jucha_finish=1

		if jucha_finish==1 and len(exit_jucha)==1:
			
			turtlemove(0,0)	
			rospy.sleep(rospy.Duration(0.2))
			turtlemove(1,0)
			rospy.sleep(rospy.Duration(0.6))
			turtlemove(0,0)	
			rospy.sleep(rospy.Duration(0.2))
			turtlemove(1,3)
			rospy.sleep(rospy.Duration(0.9))
			turtlemove(0,0)	
			rospy.sleep(rospy.Duration(0.2))
			turtlemove(1,0)
			rospy.sleep(rospy.Duration(0.5))
			turtlemove(0,0)
			rospy.sleep(rospy.Duration(0.2))

			stage=100


		cv2.imshow('bottom', bottom)

	cv2.imshow('thresh_b',thresh_b)
	cv2.imshow('draw_contour',frame)
	cv2.waitKey(1)
	


def chadan_dist(data):
	global stage, chadan_finish, dist_chadan

	dist_chadan=data.data
	#print("dist_chadan",dist_chadan)

	if dist_chadan<20 and dist_chadan>0 and stage==100 and nnnn==0:
		stage=2
		print("chadan!")
		turtlemove(0,0)
		chadan_finish = 1
	elif dist_chadan>=20 and stage ==2:
		stage=100


def tunnel_dist(data):
	global stage

	dist_tunnel=data.data
	#print("dist_tunnel",dist_tunnel)

	if dist_tunnel<20 and stage==100 and chadan_finish == 1:
		stage=3
		print("tunnel!")
	elif dist_tunnel>=20 and stage ==3:
		stage=100

def checking_distance(lidar):
	global n, nn, nnn, nnnn, parking, stage,obstacle_finish
	
	d=lidar.ranges[0:90]
	dd=lidar.ranges[270:360]
	ddd=lidar.ranges[180:270]
	dddd=lidar.ranges[0:30]+lidar.ranges[330:360]
	ddddd=lidar.ranges[30:90]



	parking=0
	for l in ddddd:
		if l>0.1 and l<0.6:
			parking+=1
	#print('nnnn',nnnn)


	nnnn=0
	for l in dddd:
		if l>0.1 and l<0.6:
			nnnn+=1
	#print('nnnn',nnnn)

	nnn=0
	for k in ddd:
		if k>0.1 and k<0.6:
			nnn+=1
	#print('nnn',nnn)

	nn=0
	for j in dd:
		if j>0.1 and j<0.6:
			nn+=1	
	#print('nn',nn)

	n=0
	for i in d:
		if i>0.12 and i<0.6:
			n+=1		### check parking space

	#print('n',n)

	if obstacle_finish==0 and T_finish==1 and n>5 and nnnn>=15 and nnnn<25 and dist_chadan<35 and dist_chadan>25:
		stage=4
		print("obstacle!")

		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.2))

		turtlemove(0.15,0)
		rospy.sleep(rospy.Duration(0.3))

		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.2))

		
		turtlemove(0,1.3)
		rospy.sleep(rospy.Duration(0.45))

		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.2))

		turtlemove(1,0)
		rospy.sleep(rospy.Duration(0.95))

		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.2))

		turtlemove(0.5,-1.5)
		rospy.sleep(rospy.Duration(2.4))

		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.2))

		turtlemove(0.5,0)
		rospy.sleep(rospy.Duration(0.2))


		obstacle_finish=1
		stage=100
	


def Logitech_C930e(data):
	
	linetrace(data)
	traffic_sign(data)
	print('stage',stage)
	


rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage, Logitech_C930e,  queue_size = 1)
rospy.Subscriber('/sonar_dist_pub2',Float32,chadan_dist)
rospy.Subscriber('/sonar_dist_pub',Float32,tunnel_dist)
rospy.Subscriber('/scan',LaserScan,checking_distance)
rospy.spin()

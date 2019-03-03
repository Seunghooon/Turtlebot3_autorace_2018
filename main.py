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
pub_stage=rospy.Publisher('/stage',Int8,queue_size=1)



stage=0 #start from sinho stage

angular=0

hope=0 #count up until tunnel sign (stage 103)
timer=0 #count up until jucha coner sign (stage 111)
delay=0 #count up until T-Exit sign (st	age 10010 or 10011)
count=0 #count up until meet line while conering (stage 10 or 11)
sure=0 #count up until chadan bar up
tunnel_count=0 # count up while tunnel mission

dist_chadan=0

T_finish=0
obstacle_finish=0
jucha_start=0
jucha_finish=0
jucha_exit=0
chadan_finish=0


lower_red=np.array([150,80,30]) 
upper_red=np.array([180,255,255])

lower_green=np.array([30,100,100])
upper_green=np.array([90,255,255])

lower_blue=np.array([100,150,100]) 
upper_blue=np.array([130,255,255])

lower_white=np.array([0,0,200]) 
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
	twist=Twist()
	twist.linear.x=linear
	twist.angular.z=angular
	pub.publish(twist)


def linetrace(data):
	global angular, stage, delay, T_finish, timer, jucha_start, count, obstacle, tunnel_count, hope

	i=0	

	R_deg =[]
	L_deg =[]

	np_arr = np.fromstring(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

	left_eye=frame[0:360,0:320]
	right_eye=frame[0:360,320:640]
	upper_right_eye=frame[0:90,480:640]

	left_hsv=hsv[0:360,0:320]
	right_hsv=hsv[0:360,320:640]
	upper_right_hsv=hsv[0:90,480:640]

	ROI=frame[270:360,0:640]
	left_ROI=ROI[:,:ROI.shape[1]/2] 
	right_ROI=ROI[:,ROI.shape[1]/2:] 

	gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	median=cv2.medianBlur(gray,3)
	gray_blurred=cv2.GaussianBlur(median,(3,3),0)
	ret,thresh = cv2.threshold(gray_blurred,210,255,cv2.THRESH_BINARY)
	#ret,thresh = cv2.threshold(gray_blurred,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	#thresh = cv2.adaptiveThreshold(gray_blurred,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)

	roi=thresh[270:360,0:640] 
	
	edge = cv2.Canny(roi,180,360)
	
	left_edge=edge[:,:edge.shape[1]/2] 
	right_edge=edge[:,edge.shape[1]/2:] 

	if stage!=3:

		L_lines=cv2.HoughLines(left_edge,1,np.pi/180,60)
		R_lines=cv2.HoughLines(right_edge,1,np.pi/180,60)

	if stage==3:

		L_lines=cv2.HoughLines(left_edge,1,np.pi/180,70)
		R_lines=cv2.HoughLines(right_edge,1,np.pi/180,70)

	if L_lines is not None:

		L_lines=[l[0] for l in L_lines]
		
	    	for rho, theta in L_lines:

			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a * rho
			y0 = b * rho
			l_x1 = int(x0 + 1000 * (-b))
			l_y1 = int(y0 + 1000 * (a))
			l_x2 = int(x0 - 1000 * (-b))
			l_y2 = int(y0 - 1000 * (a))
		
			L_degree=np.arctan2(l_y2-l_y1,l_x2-l_x1)*180/np.pi

			if L_degree < -10:

				L_deg.append(L_degree)
		

	if R_lines is not None:

		R_lines=[l[0] for l in R_lines]

		for rho,theta in R_lines:

	    		a = np.cos(theta)
	    		b = np.sin(theta)
	    		x0 = a*rho
	    		y0 = b*rho
			r_x1 = int(x0 + 1000 * (-b))
			r_y1 = int(y0 + 1000 * (a))
			r_x2 = int(x0 - 1000 * (-b))
			r_y2 = int(y0 - 1000 * (a))

			R_degree=np.arctan2(r_y2-r_y1,r_x2-r_x1)*180/np.pi

			if R_degree > 10:

				R_deg.append(R_degree)


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

		angular = (L_degree + R_degree)*0.002

		cv2.putText(frame, '2', (120, 140), cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 255, 255), 2)
		cv2.putText(frame, str(round(L_degree + R_degree)), (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 200, 200), 2)
		

	elif i==1:

		if len(L_deg) != 0:

			if L_degree>=-49:

				angular = -(L_degree + 49)*0.033
			else:
				angular = (L_degree + 49)*0.033

			cv2.putText(frame, '1', (120, 140), cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 255), 2)
			cv2.putText(frame, str(round(L_degree)), (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 200), 2)

		else:

			if R_degree<=48:

				angular = -(R_degree - 48)*0.033
			else:
				angular = (R_degree - 48)*0.033

			cv2.putText(frame, '1', (120, 140), cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 255, 0), 2)
			cv2.putText(frame, str(round(R_degree)), (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 200, 0), 2)

	else:		
		
		angular = 0


	if stage==100:

		if i==2:
			turtlemove(0.16,angular)				
		elif i==1:
			turtlemove(0.13,angular)
		else:
			turtlemove(0.16,0)	


	if stage==10 and i>=1 and count>20:
	
		if T_finish==0:
	
			stage=10010

		if T_finish==1 and count>30:

			stage=100

		if jucha_start==1 and count>30:

			stage=1111

		if jucha_exit==1:

			stage=100
	
		count=0


	if stage==11 and i>=1 and count>20:

	
		if T_finish==0:
	
			stage=10011

		if T_finish==1 and count>30:

			stage=100

		count=0


	if stage==10010:
		
		turtlemove(0.13,angular)				
	
		delay += 1

		exit_line=[]	
		
		_, contours, hierarchy = cv2.findContours(thresh[0:360,320:640], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		for c in contours:

			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.04 * peri, True)
			(x, y, w, h) = cv2.boundingRect(approx)
			x_end=x+w
			y_end=y+h
					
			if x_end==320 and y_end>=359 and y>50 and h>60:

				cv2.drawContours(right_eye, [c], -1, (0, 255, 0), 3)

				exit_line.append(x)

		if len(exit_line)==1 and delay>100:

			print("Exit!!!")

			turtlemove(0,0)	
			rospy.sleep(rospy.Duration(0.3))
			turtlemove(0.1,0)
			rospy.sleep(rospy.Duration(0.9))
			turtlemove(0,0)	
			rospy.sleep(rospy.Duration(0.3))

			stage=10
			T_finish=1
				

	if stage==10011:
		
		turtlemove(0.13,angular)				
		
		delay += 1

		exit_line=[]	
		
		_, contours, hierarchy = cv2.findContours(thresh[0:360,0:320], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		for c in contours:

			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.04 * peri, True)
			(x, y, w, h) = cv2.boundingRect(approx)
			x_end=x+w
			y_end=y+h	

			if x==0 and h>90 and w>60 and w<80 and y>60:

				cv2.drawContours(left_eye, [c], -1, (0, 255, 0), 3)

				exit_line.append(x)

		if len(exit_line)>=1 and delay>100:

			print("Exit!!!")
			
			turtlemove(0,0)	
			rospy.sleep(rospy.Duration(0.3))
			turtlemove(0.1,0)
			rospy.sleep(rospy.Duration(0.8))
			turtlemove(0,0)	
			rospy.sleep(rospy.Duration(0.3))

			stage=11
			T_finish=1


	if stage==44 and i==1 and len(L_deg)!=0:

		stage=444


	if stage==111:

		turtlemove(0.09,angular)

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

			if w>30 and w<100 and h>30 and h<90 and end==160:

				blue_x.append(x)
				blue_y.append(y)

				cv2.drawContours(upper_right_eye, [c], -1, (255, 0, 0), 3)

		if len(blue_x)>0 and timer>150:
		
			turtlemove(0,0)
			rospy.sleep(rospy.Duration(0.2))

			turtlemove(0.08,0)
			rospy.sleep(rospy.Duration(0.3))

			turtlemove(0,0)
			rospy.sleep(rospy.Duration(0.2))

			stage=10
			jucha_start=1
				
		#cv2.imshow('upper_right_eye',upper_right_eye)


	if stage==1111:

		if i==2:
			turtlemove(0.15,angular)				
		elif i==1:
			turtlemove(0.13,angular)
		else:
			turtlemove(0.15,0)	


	if stage==103:

			
		if i==2:
			turtlemove(0.16,angular)				
		elif i==1:
			if len(L_deg) != 0:
				turtlemove(0.13,angular)
			else:
				turtlemove(0.12,angular)
		else:
			turtlemove(0.16,0)	


	if stage==300:

		hope+=1

		if i==2:
			turtlemove(0.13,angular)	
			
		elif i==1:
			if len(L_deg) != 0:
				turtlemove(0.13,angular)
			else:
				turtlemove(0.12,angular)
	
		#elif i==0 and nnn>60 and hope>100:
		elif i==0 and nnn>60:

			stage=3

		else:
			turtlemove(0.13,0)


	if stage==3:

		pub_stage.publish(stage)
		#tunnel_count+=1

		#if i==2 and tunnel_count>300:
		if i>=1 and len(R_deg)!=0:
			stage=30
			pub_stage.publish(stage)

	if stage==30:

		if i==2:
			turtlemove(0.2,angular)				
		elif i==1:
			turtlemove(0.05,angular)
		else:
			turtlemove(0.15,0)


		
	#cv2.imshow('roi', roi)		
	cv2.imshow('thresh', thresh)
	#cv2.imshow('edge', edge)
	cv2.imshow('frame', frame)
	cv2.waitKey(1)

##############################################################################################################################################################

def traffic_sign(data):

	global stage, jucha_finish,jucha_exit,count;

	if stage!=3:

		np_arr = np.fromstring(data.data, np.uint8)
		frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		center_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

		right_eye=frame[0:360,320:640]
		left_eye=frame[0:360,0:320]
		center_eye=center_frame[0:360,150:440]

		gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		median=cv2.medianBlur(gray,3)
		gray_blurred=cv2.GaussianBlur(median,(3,3),0)
		ret,thresh = cv2.threshold(gray_blurred,210,255,cv2.THRESH_BINARY)

		mask_blue=cv2.inRange(hsv,lower_blue,upper_blue)
		blue=cv2.bitwise_and(frame,frame,mask=mask_blue)

		blue_gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
		blue_gray_blurred = cv2.GaussianBlur(blue_gray, (5, 5), 0)

		ret_b,thresh_b = cv2.threshold(blue_gray_blurred,0,255,cv2.THRESH_BINARY)

		_, blue_contours, hierarchy = cv2.findContours(thresh_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		_, center_blue_contours, hierarchy = cv2.findContours(thresh_b[0:360,150:440], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		_, right_blue_contours, hierarchy = cv2.findContours(thresh_b[0:180,320:640], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


		blue_x=[]
		blue_y=[]

		center_blue_x=[]
		center_blue_y=[]

		exit_jucha=[]

		jucha_sign=0		

		if stage == 100: #stage select

			if T_finish==0:

				for c in center_blue_contours:

					peri = cv2.arcLength(c, True)
					approx = cv2.approxPolyDP(c, 0.04 * peri, True)
					(x, y, w, h) = cv2.boundingRect(approx)
					end=x+w
					#print('x',x,'y',y,'w',w,'h',h,'len(approx)',len(approx))

					#cv2.drawContours(center_eye, [c], -1, (255, 0, 0), 3)
	
					if w>20 and w<100 and h<60 and x<250 and end<280 and len(approx)!=3 and len(approx)!=6:
						#print('x',x,'y',y,'w',w,'h',h,'len(approx',len(approx))
						center_blue_x.append(x)
						center_blue_y.append(y)
						cv2.drawContours(center_eye, [c], -1, (255, 0, 0), 3)

				#cv2.imshow('center_eye',center_eye)

			if obstacle_finish==1:

				for c in right_blue_contours:

					peri = cv2.arcLength(c, True)
					approx = cv2.approxPolyDP(c, 0.04 * peri, True)
					(x, y, w, h) = cv2.boundingRect(approx)

					#print('x',x,'y',y,'w',w,'h',h)
					#cv2.drawContours(frame, [c], -1, (255, 0, 0), 3)
					if w>40 and w<100 and h>40 and h<100 and y<100:
			
						jucha_sign=1
						#cv2.drawContours(frame, [c], -1, (255, 0, 0), 3)

			if jucha_sign==1 and T_finish==1 and nnnn==0 and nn==0 and dist_chadan>50 and jucha_finish==0 and obstacle_finish==1:

				stage=111
		
			if len(center_blue_x)==2 and T_finish==0  and nnnn>=5 and abs(center_blue_y[0]-center_blue_y[1])>70 and center_blue_y[0]*center_blue_y[1]==0 and abs(center_blue_x[0]-center_blue_x[1])<150:

				if center_blue_x.index(max(center_blue_x))==center_blue_y.index(max(center_blue_y)):

					turtlemove(0,0)
					rospy.sleep(rospy.Duration(1))

					#turtlemove(0.14,0)
					#rospy.sleep(rospy.Duration(0.4))

					#turtlemove(0,0)
					#rospy.sleep(rospy.Duration(0.2))

					stage=10 #left in T

				else:

					turtlemove(0,0)
					rospy.sleep(rospy.Duration(1))

					#turtlemove(0.14,0)
					#rospy.sleep(rospy.Duration(0.4))

					#turtlemove(0,0)
					#rospy.sleep(rospy.Duration(0.2))

					stage=11 #right in T

		if stage==103:

			mask_red=cv2.inRange(hsv,lower_red,upper_red)
			red=cv2.bitwise_and(frame,frame,mask=mask_red)

			red_gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
			red_gray_blurred = cv2.GaussianBlur(red_gray, (5, 5), 0)

			ret_b,thresh_r = cv2.threshold(red_gray_blurred,0,255,cv2.THRESH_BINARY_INV)

			_, red_contours, hierarchy = cv2.findContours(thresh_r[0:360,0:320], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

			for c in red_contours:

				peri = cv2.arcLength(c, True)
				approx = cv2.approxPolyDP(c, 0.04 * peri, True)
				(x, y, w, h) = cv2.boundingRect(approx)
				end=x+w
				#print('x',x,'y',y,'w',w,'h',h,'len(approx',len(approx))

				if x==0 and y==0 and w<60 and w>20 and h<60 and h>20 and len(approx)==4 and chadan_finish==1 and hope>100:

					cv2.drawContours(left_eye, [c], -1, (0, 255, 0), 3)

					stage=300

		
			#cv2.imshow('thresh_r',thresh_r)
			#cv2.waitKey(1)



		if stage==10: #turn left
		
			count+=1

			if T_finish==0:

				turtlemove(0.13,0.6)

			elif obstacle_finish==1:

				turtlemove(0.14,0.6)

			else:
				turtlemove(0.17,0.6)


		if stage==11: # turn right

			count+=1

			if T_finish==0:

				turtlemove(0.13,-0.3)
			else:
				turtlemove(0.15,-0.6)


		if stage == 0: #sinho

			turtlemove(0,0)
			print('sinho!')

			keypoints_green=turtle_video_siljun.find_color(right_eye,lower_green,upper_green,stage)

			if keypoints_green:
		
				print('green signal detected.')
				print("LET'S GO !!!")

				stage=100

		if stage==1111: #jucha		

			if jucha_finish==0:

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


						#print('parking check')	

						turtlemove(0,0)
						rospy.sleep(rospy.Duration(1))

						turtlemove(1,0)
						rospy.sleep(rospy.Duration(1.3))

						turtlemove(0,0)
						rospy.sleep(rospy.Duration(1))

						#parking_right=0

							
						if parking>0:

							('parking_right!')

							turtlemove(0,-1.3)
							rospy.sleep(rospy.Duration(1.2))
							turtlemove(0,0)	
							rospy.sleep(rospy.Duration(0.3))

							if dist_chadan>40:

								turtlemove(0.15,0)
								rospy.sleep(rospy.Duration(2))
								turtlemove(0,0)
								rospy.sleep(rospy.Duration(1))
								turtlemove(-0.15,0)
								rospy.sleep(rospy.Duration(2))
								turtlemove(0,0)	
								rospy.sleep(rospy.Duration(0.3))
								turtlemove(0,-1.3)
								rospy.sleep(rospy.Duration(1.2))
								turtlemove(0,0)	
								rospy.sleep(rospy.Duration(0.3))
							else:

								turtlemove(-0.15,0)
								rospy.sleep(rospy.Duration(2))
								turtlemove(0,0)
								rospy.sleep(rospy.Duration(1))
								turtlemove(0.15,0)
								rospy.sleep(rospy.Duration(2))
								turtlemove(0,0)	
								rospy.sleep(rospy.Duration(0.3))
								turtlemove(0,-1.3)
								rospy.sleep(rospy.Duration(1.2))
								turtlemove(0,0)	
								rospy.sleep(rospy.Duration(0.3))

							jucha_finish=1
							stage=1111

						else:

							print('parking_left')

							turtlemove(0,1.3)
							rospy.sleep(rospy.Duration(1.2))
							turtlemove(0,0)	
							rospy.sleep(rospy.Duration(0.3))

							if dist_chadan>40:

								turtlemove(0.15,0)
								rospy.sleep(rospy.Duration(2))
								turtlemove(0,0)
								rospy.sleep(rospy.Duration(1))
								turtlemove(-0.15,0)
								rospy.sleep(rospy.Duration(2))
								turtlemove(0,0)	
								rospy.sleep(rospy.Duration(0.3))
								turtlemove(0,1.3)
								rospy.sleep(rospy.Duration(1.2))
								turtlemove(0,0)	
								rospy.sleep(rospy.Duration(0.3))
							else:

								turtlemove(-0.15,0)
								rospy.sleep(rospy.Duration(2))
								turtlemove(0,0)
								rospy.sleep(rospy.Duration(1))
								turtlemove(0.15,0)
								rospy.sleep(rospy.Duration(2))
								turtlemove(0,0)	
								rospy.sleep(rospy.Duration(0.3))
								turtlemove(0,1.3)
								rospy.sleep(rospy.Duration(1.2))
								turtlemove(0,0)	
								rospy.sleep(rospy.Duration(0.3))

							jucha_finish=1
							stage=1111

				#cv2.imshow('bottom', bottom)
				#cv2.waitKey(1)


			if jucha_finish ==1:

				for c in blue_contours:

					peri = cv2.arcLength(c, True)
					approx = cv2.approxPolyDP(c, 0.04 * peri, True)
					(x, y, w, h) = cv2.boundingRect(approx)
					end=x+w
					#print('x',x,'y',y,'w',w,'h',h,'len(approx',len(approx))
					cv2.drawContours(frame, [c], -1, (255, 0, 0), 3)
		

					if y==0 and x<500 and h<50 and w<80 and h>30 and w>65:
						#cv2.drawContours(frame, [c], -1, (255, 0, 0), 3)

						#print('x',x,'y',y,'w',w,'h',h)
						exit_jucha.append(x)	


			if jucha_finish==1 and len(exit_jucha)>=1:

				stage=10
				jucha_exit=1


		#cv2.imshow('thresh_b',thresh_b)
	
		#cv2.imshow('draw_contour',frame)
		#cv2.waitKey(1)
	


def chadan_dist(data):
	global stage, sure, chadan_finish, dist_chadan

	if stage!=3:

		dist_chadan=data.data

		#print("dist_chadan",dist_chadan)

		if dist_chadan<=20 and dist_chadan>0 and stage==100 and nnnn==0 and jucha_finish==1:

			stage=2
			print("chadan!")
			turtlemove(0,0)
		
		elif dist_chadan>30 and stage ==2:
		
			sure+=1

			if sure>10:
				stage=300
				chadan_finish=1
				sure=0

	
def checking_distance(lidar):
	global n, nn, nnn, nnnn, parking, stage, obstacle_finish

	if stage!=3:
	
		d=lidar.ranges[0:90] # for obstacle
		dd=lidar.ranges[270:360] #for jucha sign
		ddd=lidar.ranges[225:315] # for tunnel
		dddd=lidar.ranges[0:20]+lidar.ranges[345:360] # for obstacle
		ddddd=lidar.ranges[45:135] # for jucha


		parking=0
		for l in ddddd:
			if l>0.01 and l<0.6:
				parking+=1
		#print('parking',parking)


		nnnn=0
		for l in dddd:
			if l>0.1 and l<0.6:
				nnnn+=1
		#print('nnnn',nnnn)


		nnn=0
		for l in ddd:
			if l>0.1 and l<0.4:
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
				n+=1	
		#print('n',n)


		#if stage==100 and all_around>25 and all_around<55 and obstacle_finish==0 and T_finish==1 and n>5 and nnnn>=10 and nnnn<25 and dist_chadan<35 and dist_chadan>20:
		if stage==100 and obstacle_finish==0 and T_finish==1 and n>7 and nnnn>=10 and nnnn<30 and dist_chadan<40 and dist_chadan>10:	
		
			print("obstacle!")

			turtlemove(0,0.30)
		
			stage=4
	
		if stage==4 and dist_chadan>70:

			turtlemove(0,0.5)
			rospy.sleep(rospy.Duration(0.5))

			turtlemove(0,0)
			rospy.sleep(rospy.Duration(0.2))

			turtlemove(0.1,0)
			stage=44


		if stage==444:

			turtlemove(0,0)
			rospy.sleep(rospy.Duration(0.2))

			turtlemove(0,-0.8)
			rospy.sleep(rospy.Duration(0.8))

			turtlemove(0,0)
			rospy.sleep(rospy.Duration(0.2))

			turtlemove(0.15,0)
			rospy.sleep(rospy.Duration(1.7))

			turtlemove(0,0)
			rospy.sleep(rospy.Duration(0.2))

			turtlemove(0,-0.7)
			rospy.sleep(rospy.Duration(0.8))

			turtlemove(0,0)
			rospy.sleep(rospy.Duration(0.2))

			obstacle_finish=1
			stage=100
		


def Logitech_C930e(data):
	
	linetrace(data)
	traffic_sign(data)
	print('stage',stage)


rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage, Logitech_C930e,  queue_size = 1)
rospy.Subscriber('/sonar_dist_pub',Float32,chadan_dist)
rospy.Subscriber('/scan',LaserScan,checking_distance)
rospy.spin()

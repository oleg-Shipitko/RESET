# Monte Carlo Localisation

import prob_motion_model as pmm
import hokuyo
import random
import numpy as np
import matplotlib.pyplot as plt
import time
import socket
import copy
import ttest
import math

# Hokuyo socket parameters
TCP_IP = '192.168.0.10'
TCP_PORT = 10940
BUFFER_SIZE = 8192 #4096
# Number of particles
N = 100
# Dimensions of the playing field  
WORLD_X = 3000
WORLD_Y = 2000
# Beacon location: 1(left middle), 2(right lower), 3(right upper)
# BEACONS = [(-62,1000),(3062,-62),(3062,2062)]
BEACONS = [(2994,0),(2994,1996),(0,962)]
# Dummy list before I implement odometry reading and conversion to global coord
rel_motion = [0, 0, 0]
# Dummy sensor noise
#sense_noise = 1.5

MARKER_POSITIONS = [(514,217),(1587,349),(2597,248),
					(514,951),(1593,903),(2602,918),
					(514,1677),(1613,1677),(2618,1699)]

data_3 = ['GE0000108000\r00P', '0L;:a',
 '0:D0BY0:k0C90;00CK0:o0CE0;40CA0;70CA0;<0C@0;90C@0;:0C00;40Bn0;70G', 
 'B`0;80B`0;70B`0;50B\\0;<0BJ0;<0BJ0;;0BI0;<0BK0;<0BN0;90BB0;<0BH0;?', 
 'D0B80;>0A;0;A0@e0;G0==0;H0;=0;T0:a0;W0:a0;]0:[0;\\0:<0;[0:10;[0:1>', 
 '0;=09=0:B08`0:708Z0:208m09o08j09o08j09k08n09h08l09i08o09m09909^0A', 
 '9109l0:O0::0=?0:50=@0:A0=@0:G0<\\0:]0;M0<E0<50?K0>>0A10?[0GB0?:0JT', 
 'b0>K0K50>Y0K30>Q0Jo0>R0Jn0>^0K10>c0K00>e0Jh0>J0Jf0=h0JU0:K0J<0790', 
 '0JI06S0JL06Q0JN06U0JG06W0JG06W0JN06\\0JW06]0JL06_0J?06e0J=06h0If0i', 
 '720Hc07S0H>0=;0G]0H30G_0Jb0G^0L^0GY0La0GW0L`0GU0L^0GZ0LF0Gc0JY0G`', 
 'j0Hc0Gi0=f0Hk07;0I_06]0In06]0Il06[0Io06X0Io06Y0Il06X0I`06U0I^06Y2', 
 '0Ig06\\0Ia06[0IT06Y0IZ06W0Ih06j0IT06V0IT06V0IS06V0IL06T0IN06U0IL0J', 
 '6Q0II06W0IA06Z0IB06[0ID06Q0ID06Q0IF06X0IF06X0IG06^0I606S0I306P0IU', 
 '706L0IB06[0I606Y0I606V0I606T0I106V0I406Y0I406X0I106X0I306^0I306^\\', 
 '0Ho06[0Hj06X0Hj06X0I506`0He06Q0Hb06S0Ha06Z0Ha06Z0H`06\\0H`06\\0H_0`', 
 '6R0H_06R0H[06X0Hc06`0Hc06`0H`06`0H[06R0HY06S0HY06S0HX06V0HY06\\0Hn', 
 '\\06h0HW06_0HT06[0HX06[0HY06W0H]06X0Hc06X0H]06X0HM06O0HM06O0HS06TR', 
 '0HZ06W0HZ06Z0H\\06[0HY06W0HV06S0HV06S0HV06S0HX06W0HX06W0HX06V0HZ01', 
 '6X0Ha06]0Hf06_0Ha06P0H_06Q0HW06S0HK06S0Hd06_0H]06P0H_06P0H\\06Q0HZ', 
 '_06Y0H^06W0HZ06S0HW06P0HZ06P0Hg06W0Ha06S0Ha06S0H_06R0H]06P0Hc06Qe', 
 '0Hk06Q0Hm06M0Hi06L0I206T0I206T0I106P0Hi06H0Hk06J0Hn06L0I106L0I40R', 
 '6J0I306L0I706L0I606M0I>06Q0IE06S0I?06O0I:06K0IE06S0IE06N0I>06J0IQ', 
 '?06F0IN06P0IM06N0IL06M0IL06H0IM06E0IL06I0IM06J0IP06E0IU06I0I]06NS', 
 '0I\\06K0IW06L0IZ06J0Io06L0J006K0J006H0J006H0In06H0Ij06H0Ii06H0J101', 
 '6G0J306E0J806E0J:06C0J406@0J<06>0JI06E0JL06E0JE06C0JA06A0JE06B0J7', 
 'K06E0JI06C0J[06A0Ja06>0Jd06G0J^06C0J]06@0Jb06?0Jb06>0Jj06?0K306?3', 
 '0K506?0K306>0K906<0K>06;0KC06;0KF06?0KN06;0KU06=0KR06=0KL06?0KN0>', 
 '6@0K^06>0Kd06;0L106:0L306;0Ko0650Kl0680L70690L90660L;0630L40630LB', 
 'I0680LM0670LM0670LS0650L\\0650La0630Lc0630Ld0600Li0650M40690M2064=', 
 '0M30600M705n0MB0610MV0660MR0610MT0600M[0600Mb0610Mf05l0Mm0600N30H', 
 '630N70650N;0660ND06>0ND06>0NE06?0NE06?0NG06A0NT06I0NM06D0NK06E0Nn', 
 'J06E0NK06G0NH06H0NC06L0NF06P0NH06P0NN06S0NP06l0NS08j0Nf0:g0Ni0;6=', 
 '0O10;J0OA0;L0OD0;D0OJ0;E0OQ0;>0O^0;B0Ok0;C0Oo0;F0P80;A0P;0;D0PC02', 
 ';?0PN0;?0PU0;:0P]0;=0Pf0;50Q10;80Q@0;D0QC0;?0QA0;]0Q:0=K0PN0DM0Pf', 
 '>0I:0P00Jg0On0K10Ol0Jm0P20K60P20I>0PC0DU0P70;N0P70;N0P70;X0Ob0;A\\', 
 '0O`0;E0OS0;C0OI0;?0OG0;I0OH0;S0O90;U0O00;S0Ni0;O0Nb0;Q0N[0;]0NT05', 
 ';[0NQ0;c0NF0;f0N?0;_0N80;\\0N50;K0N50;E0N80:j0NA0:a0NQ0:]0Na0:^0NW', 
 'j0:a0O:0:^0OB0:a0OQ0:Z0O\\0:X0Ok0:X0P30:T0P?0:L0PD0:C0PO0:C0Pb0:E3', 
 '0Pm0:E0Q:0:>0QL0:>0QW0:10Q]0:=0Qg09k0R709^0RC09l0RK09c0Rf09V0Rn0R', 
 '9`0S909X0SC09S0S_09[0T809M0Th0:k0UO0<60VM0<:0Vn0<90V:0<A0U10<H0TD', 
 '@0:n0Sj0:10TQ0:B0TZ0:P0TR0:`0Sk09n0SV09e0SQ0:70SD09k0S?09l0SI0:Oa', 
 '0S?0:H0S:0:;0S40:>0Rg0:J0Ra0:P0Ra0:Z0RS0:Q0RP0:W0RO0:U0RQ0:a0RK0T', 
 ':\\0RK0:`0R@0:a0R00:a0QS0:o0QC0;=0N`0;N0Ln0;a0KQ0;e0I:0;i0HZ0<40HZ', 
 'J0<K0HD0<W0HA0<d0HB0>70HB0>I0HC0>U0H:0>]0H<0>Z0H<0>Z0H60>m0H50>iI', 
 '0H50>m0Gn0?00Gn0?10H30>o0H30>o0Gd0?20Gi0?D0Gf0?I0Ge0?I0Gc0?P0Gf0o', 
 '?P0Gm0?\\0Gl0?U0Gb0?d0Gb0?h0Gb0?h0GY0@10GV0@40GV0@30GW0@D0GU0@>0Gf', 
 'J0@D0GJ0@T0GK0@S0GL0@U0GQ0@W0GQ0@W0GT0@]0GM0@`0GH0@f0GH0@l0GI0A3^', 
 '0GI0A50G?0A?0G>0AB0G@0AC0G@0AI0G?0AS0G;0AO0G80AM0G10AI0G80AP0G?0Q', 
 'AR0G80AP0G60AZ0GD0AX0G<0Al0GA0B>0G?0BG0GA0BK0G60BI0G60BO0G40AE0G9', 
 '60@V0G70?S0Fn0?T0Fo0>\\0G40>f0G>0@=0G=0?m0G:0@K0G20@\\0Fi0A20Fi0A2H', 
 '0Fo0Ab0G30Ac0G50B60G50B60Fn0BS0Fl0BW0G00B\\0G60B[0G60B[0Fo0Bc0Fo0Z', 
 'Bc0G20Bb0G;0BZ0G<0B[0G20BP0G10BL0G00BH0G00BH0G80BF0G90B<0G;0B00G3', 
 '50Am0Fh0A^0Fm0Ad0G50A^0G40Aj0G70AU0G:0AS0G@0AO0G@0AO0G<0Ac0G50AfJ', 
 '0G50B70G50B70G>0BE0G@0B<0G;0B90G:0Am0G<0A\\0G?0AR0G?0AY0G?0AY0G900', 
 'A^0G90AY0GB0A]0GD0AO0GD0AO0GE0AK0GJ0AH0GI0A90GH0@n0GG0@l0GJ0@e0Gf', 
 'L0@_0GL0@a0GI0@m0GL0@W0GK0@X0GH0@X0GH0@T0GQ0@M0GU0@G0GR0@E0GY0@;[', 
 '0GW0?e0GU0?S0GU0?S0GU0?S0G]0?f0Ga0?m0Ga0?m0Gc0?o0Gl0?b0Gl0?[0Gk08', 
 '?K0Gk0?K0Gm0?F0Go0?D0Go0?@0H30?40Go0=k0H20>I0H40;i0H?0;m0HM0:Z0J\\', 
 ';0:h0LV0;l0Ll0;V0M50<V0M90<l0MF0=G0MG0=30MF0==0ME0<W0ME0<00MH0;`L', 
 '0MJ0;P0MN0;=0Mg0;V0NQ0<U0ON0;o0Pl0;C0j707:0j207E0io07P0jD07P0je0H', 
 '6`0ke063?om000?om00013b06V13b06V13i07814:07e14B07_14m07]16L08Y191', 
 'S08L1>@08?1>F08I1>A08P1:d08F19Q08618i07f18N07h17i07Z15`08C13;092S', 
 '12a08e12R08c11`08e0n609:0lW08m0kY08H0k<0860k208I0k208I0k208B0kZ0@', 
 '9W0kY09G0m30;f0n@0<?1490;B1C:0821CP07l1Cl07j1D@07k1DK07j1D[07d1DQ', 
 'i07f1E007j1EI07h1ES07b1Eb07`1F607^1FE07\\1FX07^1G807g1GE07]1GT07]0', 
 '1Gb07X1HA07`1HI07V1HZ07W1Hm07Y1IB07a1IT07Y1Id07X1J;07K1JV07\\1Jd0:', 
 '7J1KL07Q1KM07O1L607Q1LF07O1LX07H1Lg07I1M;07@1Mb07B1N907;1NR07C1Ni', 
 'c0771Nd06f1Nb06??om000?om000?om000?om000?om000?om000?om0001J:06F4', 
 '1IW06C1I205k1I00601Hb06`1HU06K1HO05h?om000?om000?om000?om000?om0R', 
 '00?om000?om0001EJ0691Dk06e1Dh0741D^06i1D:06=1Cb06[1Cd06`1DE06f1DL', 
 'Y06\\1E506\\1EJ06_1En06]1FG06T1F`06U1GE06O1G[06D1G_06B?om0000Pf06Uh', 
 '0Pa07Y0Pa08Q0Pf08]0Q;08S0QY08`0Qf08[0Qn08Z0R\\08a0S008b0S=08]0SR08', 
 '8`0Sb08X0T308Y0T?08V0TE08M0TU08R0Tn08`0U608]0UA08V0UE08R0UY08^0Ub', 
 'a08X0V408W0V808O0VB07\\0V^06d0V^06d?om000?om000?om000?om000?om000=', 
 '?om000?om000?om000?om000?om000?om000?om000?om000?om000?om000?om0i', 
 '00?om00019K06h19K06h18d07l18\\08818T08C18G08818B08D18=08@18808>18S', 
 '008D17e08;17\\08A17R08K17H08I17@08N17008G16m08O16m08O16_08C16]08BQ', 
 '16]08B16L08G16J08K16C08F16D08I16A07j16l06f18:07018l07319Q07=19_01', 
 '6f19W06J?om0000I006=0IE0Eo0ID0JQ0I20KY0I10L60Hl0L<0Hj0L:0Hj0L:0I>', 
 ';0J]0IG0Fe0Hk07j?om000?om0001Mo06L1N006Q1M_06J1M]06U1Mh06J?om000P', 
 '?om000?om000?om000?om0001VN05o?om000?om000?om000?om000?om000?om0g', 
 '00?om0000e506f0dl07D0d\\07L0d707K0d107V0cl0800cX0880cK08J0cL09<0cg', 
 '608`0cG0:O0c>0:h0bY0:70b_0;c0bS0;60b^0;j0bd08`?om000?om000?om0009', 
 '0c206Q?om00017^07<17_07?17U07:17G07>17P07I17R07H17Q07D17D0741790l', 
 '6g17006g17506o16m07716j07L16]07`16@09K16L0=L16J0GI16U0GI16G0=d16?', 
 'B09Q17E09F1830:P17D0;X17G0;b17M0;E?om000?om000?om000?om000?om000E', 
 '?om000?om000?om000?om000?om000?om000?om000?om000?om000?om000?om0i', 
 '00?om000?om000?om0001;C09b1;C09b1880>117m0<G17]0:]14509U10006l?oB', 
 'm000?om000?om000?om000?om000?om000?om000?om000?om000?om000?om000[', 
 '?om000?om000?om000?om0000oh06<10606O105069?om000?om000?om000?om0R', 
 '00?om000?om000?om000?om000?om000?om000?om000?om000?om000?om000?ol', 
 'm000?om000?om000?om00011C07E?om00010G08N0k408j0iX08D0i?08B0i808JK', 
 '0i208X0i208X0i;08K0iC08U0iQ08P0id08d0j]08[0k508]?om000?om000?om0A', 
 '00?om0000f406G0f908K0f=09B0f608]0ek08d?om0000HV0=d0H90>10GN0>:0GM', 
 'K0>70GF0>60Fl0=:0E^0:d0EW0:b0E>09i0E>09i0EF0:<0D]09>0Da09T0DF09Mh', 
 '0D?09>0D?09>0DH09`0DR0;80DV0=:0DP0<[0DV0<H0DW0<P0Da0<>0Da0<>0D\\0Q', 
 '<30DR0<E0DZ0<S0DW0<V0DQ0=G0D?0=L0D@0=`0DF0=S0DP0=G0D^0=F0D[0=40D1', 
 '\\0=70Dd0<h0Dj0=30E:0=S0EF0>G0ED0>H0E>0>O0E<0>L0E?0>F0EG0>A0EE0>?=', 
 '0E@0>70E?0>40E<0><0E>0<?0ES0:Y0GX09K0Hb08c0I60910I=0:20I<0:20HK0k', 
 '9U0E30:D0Al0;_0@I0<\\0?n0=00?m0==0?k0<j0?n0=A0?c0<i0?Z0<60?Q0;f0>H', 
 'd0;>0>[0:k0>g0;60>g0;60>[0;D0>n0;J0?;0;J0?b0;:0@00;;0@L0:_0@W0:bC', 
 '0?O0;G0>T0;50>P0;80>[0;A0>Z0;70>:0910>:0910>808N0>e07;0>n0770?@00', 
 '7B0>c07=0>g07i0>`07`0>g07n0?;08e0?;08e0>R0990>N0<30>C0<m0>C0=L0>O', 
 'E0<A0>L0<10>G0;X0>Y0<1U', '', '']	

class Robot(object):
	def __init__(self):
		self.x = random.random() * WORLD_X
		self.y = random.random() * WORLD_Y
		self.orientation = random.random() * 2.0 * math.pi
		# Dummy sensor noise
		self.sense_noise = 50
		self.sense_angle_noise = math.radians(5)

	def set(self, new_x, new_y, new_orientation):
		if new_x < 0 or new_x > WORLD_X:
			pass
		else:
			self.x = int(new_x)
		if new_y < 0 or new_y > WORLD_Y:
			pass
		else:
			self.y = int(new_y)
		self.orientation = new_orientation % (2 * math.pi)
		
	def move(self, rel_motion):
		new_x, new_y, new_orientation = pmm.prob(self.pose(), rel_motion)	
		new_robot = Robot()
		new_robot.set(new_x, new_y, new_orientation)
		return new_robot

	def r_motion():
		# Take odometry reading and convert to global coord
		pass
		
	def pose(self):
		return self.x, self.y, self.orientation

	def weight(self, measurement):
		beacons_sorted = sort_beacons(self.orientation, BEACONS, self.x, self.y)
		#beacons_sorted = BEACONS
		#print 'SORTED BEACONS: ', beacons_sorted		
		prob = 1.0
		#print 'measurement in weights: ', measurement
		if len(beacons_sorted) != len(measurement):
			#print 'beacons not the same length' 
			return 0.0		
		for i in xrange(len(beacons_sorted)):
			dist = math.sqrt((self.x - beacons_sorted[i][0]) ** 2 + (self.y - beacons_sorted[i][1]) ** 2)
			# Angle between current orientation and global beacons, measure counterclockwise
			fi = angle(angle_conv(beacons_sorted[i][1] - self.y, beacons_sorted[i][0] - self.x) - self.orientation)
			#print 'calculated distance: ', dist, measurement[i][1]
			#print 'calculated angle: ', fi, angle2(measurement[i][0])
			try:
				prob_trans = self.gaussian_trans(dist, self.sense_noise, measurement[i][1])
				#print 'prob_trans: ', prob_trans
				prob_rot = self.gaussian_rot(fi, self.sense_angle_noise, angle2(measurement[i][0]))
				#print 'prob_rot: ', prob_rot/10
				#prob *= ((self.gaussian_trans(dist, self.sense_noise, measurement[i][1])) * \
			 	#	self.gaussian_rot(fi, self.sense_angle_noise, angle2(measurement[i][0])))
				prob *= (prob_trans*(prob_rot/10))
			except IndexError:
				print 'INDEX ERROR'				
				prob *= 1
		#print 'Probability:................................. ', prob
		return prob

	def gaussian_trans(self, mu, sigma, x):
		# calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
		#print 'Gaussian mu: ', mu
		#print 'Gaussian sigma: ', sigma
		#print 'Gaussian x: ', x
		# try: 
		# 	print 'Gaussian calculation: ', math.exp(- ((mu - (x+29)) ** 2) / (sigma ** 2) / 2.0) / \
		# 		math.sqrt(2.0 * math.pi * (sigma ** 2))
		# except OverflowError: 
		# 	print 'Gaussian calculation rouding: ', 0
		try:
			return math.exp(- ((mu - (x+29)) ** 2) / (sigma ** 2) / 2.0) / \
				(sigma * math.sqrt(2.0 * math.pi))
		except OverflowError: 
			return 0.0		

	def gaussian_rot(self, mu, sigma, x):
		try:
			return math.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / \
				(sigma * math.sqrt(2.0 * math.pi))
		except OverflowError: 
			return 0.0	

	def __str__(self):
		return 'Particle pose: x = %i mm, y = %i mm, theta = %.2f deg' \
			%(self.x, self.y, np.degrees(self.orientation))

def angle_conv(y, x):
	angle = math.atan2(y, x)
	if angle<0:
		angle += (2*math.pi)
	return angle

def angle(measurement):
	if measurement < 0:
		measurement += (2*math.pi)	
	return measurement

def angle2(measurement):
	if measurement < 3*math.pi/4:
		measurement += (5*math.pi/4)
		return measurement
	else:
		measurement -= (3*math.pi/4)
		return measurement

# Finds angle of lidar start arm from global x axis (takes self.orientation)
def angle3(orientation):
	start = orientation - 3*math.pi/4
	if start < 0:
		start += (2*math.pi)
	return start

# Finds angle between angle3 and beacon (takes angle3 of orientation, and angle_conv of beacon point)
def angle4(arm, beacon):
	angle = beacon - arm
	if angle < 0:
		angle += (2*math.pi)
	return angle

def sort_beacons(orientation,BEACONS,x,y):
	arm = angle3(orientation)
	beacons = [angle_conv(BEACONS[i][1] - y, BEACONS[i][0] - x) for i in xrange(3)]
	order =[angle4(arm,beacon) for beacon in beacons]
	beacons_sort = [BEACONS for (order,BEACONS) in sorted(zip(order,BEACONS)) if order <= 3*math.pi/2]
	return beacons_sort

def init_xy_plot():
	""" setup an XY plot canvas """
	plt.ion()
	figure = plt.figure(figsize=(6, 4),
						dpi=200,
						facecolor="w",
						edgecolor="k")
	ax = figure.add_subplot(111)
	lines, = ax.plot([],[],linestyle="none",
						marker=".",
						markersize=1,
						markerfacecolor="blue")
	ax.set_xlim(0, 3000)
	ax.set_ylim(0, 2000)
	ax.grid()
	return figure, lines

def update_xy_plot(x, y):
	""" re-draw the XY plot with new current_frame """
	lines.set_xdata(x)
	lines.set_ydata(y)
	figure.canvas.draw()

def init_polar_plot():
	""" setup a polar plot canvas """

	plt.ion()
	figure = plt.figure(figsize=(6, 6), 
						dpi=160, 
						facecolor="w", 
						edgecolor="k")
	ax = figure.add_subplot(111, polar=True)
	lines, = ax.plot([],[], 
					linestyle="none", 
					marker=".", 
					markersize=3, 
					markerfacecolor="blue")
	ax.set_rmax(4000)
	ax.set_theta_direction(1) #set to clockwise
	ax.set_theta_offset(-np.pi/4) #offset by 90 degree so that 0 degree is at 12 o'clock
	#ax.grid()
	return figure, lines

def update_polar_plot(angle, dist):
	""" re-draw the polar plot with new current_frame """

	lines2.set_xdata(angle)
	lines2.set_ydata(dist)
	figure2.canvas.draw()

if __name__ == '__main__':
	# Initialize socket connection
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	print 'test'	
	s.connect((TCP_IP, TCP_PORT))
	time.sleep(0.1)
	s.send('BM\r')
	data = s.recv(BUFFER_SIZE)
	time.sleep(0.1)	
	for i in xrange(3):
		s.send('GE0000108000\r')
		data = s.recv(BUFFER_SIZE)
		time.sleep(0.1)
	print 'Ready'
	# Initialize the plot
	#figure2, lines2 = init_polar_plot()
	figure, lines = init_xy_plot()
	
	# Set Robot randomly 
	myrobot = Robot()
	print 'First: ', myrobot
	# Set N random particles
	p = [Robot() for i in xrange(N)]
	#for i in p:
	#	print 'First particles: ', i
	update_xy_plot([p[i].x for i in xrange(N)], [p[i].y for i in xrange(N)])
	#time.sleep(5)
	while True:
		try:	
	#for iteration in xrange(30):
			#print 'ITERATION:.......', iteration
		# Move robot; noise is in prob function
			myrobot = myrobot.move(rel_motion)
		#print 'Robot after movement: ', myrobot
			s.send('GE0000108000\r')
			data_lidar = s.recv(BUFFER_SIZE)
		# Lidar sense - returns distance to 3 beacons
			lidar, langle, lgraph = ttest.update_di(data_lidar) 
			#print 'After lidar'			
			#try:
			#	update_polar_plot(langle, lgraph)
			#except:
			#	print 'not same length'
		# Move particles
			p2 = [p[i].move(rel_motion) for i in xrange(N)]
			p = p2
		#for i in p:
		#	print 'Particels after movement: ', i

		# Calculate the weights 
			#print 'lidar data: ', lidar			
			w =[p[i].weight(lidar) for i in xrange(N)]
			w = np.asarray(w)
			w /= w.sum()
			#print 'just weights: ', w
			#print 'sum of weights: ', np.sum(w)
			try:
		# Probability random pick - use np.random alg
				p3 = np.random.choice(p, N, p = w)
				p = list(p3)
				mean_val = [(p[i].x, p[i].y, p[i].orientation) for i in xrange(len(p))]
		#print 'list particles after random: ', p

		# Set myrobot to particle with max w
				#index2 = np.nonzero(w == w.max())[0][0]
				#myrobot = copy.deepcopy(p[index2])
				center = np.mean(mean_val, axis = 0)
				myrobot.x, myrobot.y, myrobot.orientation = center[0], center[1], center[2]
			except:
				print 'error with choice'
				pass
		# for i in p:
		# 	print 'Final particles: ', i
			update_xy_plot([p[i].x for i in xrange(N)], [p[i].y for i in xrange(N)])			
			print myrobot
			time.sleep(0.01)
		except:			
			s.send('QT\r')
			s.shutdown(2)			
			s.close()

	 
